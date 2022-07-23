from __future__ import print_function

from collections import deque
from queue import Queue
from threading import Event, Semaphore
from time import time
from typing import List

import serial
from messages import CurrentState, CommandHeader, AxisConfig, ConfigCommand, MoveCommand
from proto import logger
from serial.threaded import ReaderThread, Packetizer


class ThreadedProto(ReaderThread):

    def __init__(self, port, baud, message_callback=None):

        raise Exception()

        self.step_ser = serial.Serial(port, baudrate=baud)

        self.p_queue = Queue()  # Packets, main data, commands, done messages

        self.f_queue = deque()  # Flow control. Acks and Nacks
        self.t_queue = deque()  # Text messages.

        self.seq = 0;
        self.last_done = 0
        self.current_state = CurrentState()

        self.empty_event = Event()
        self.next_rcv_event = Event()  # Cleared just before send, set on recieve

        self._is_empty = Semaphore()

        if message_callback is None:
            def _message_callback(proto, m):

                pl = m.payload.decode('ascii')

                if m.limit_code == CommandHeader.CC_DEBUG:
                    logger.debug(pl)
                elif m.limit_code == CommandHeader.CC_ERROR:
                    logger.error(pl)
                else:
                    logger.info(pl)

        self.message_callback = _message_callback

        def factory():
            return ProtoPacket(self)

        super(ThreadedProto, self).__init__(self.step_ser, factory)

    def start(self):
        super(ThreadedProto, self).start()
        self._connection_made.wait()
        if not self.alive:
            raise RuntimeError('connection_lost already called')

    def handle_packet(self, m):

        m.recieve_time = time()

        if m.limit_code in (CommandHeader.CC_ACK, CommandHeader.CC_NACK, CommandHeader.CC_ECHO):
            self.f_queue.append(m)
            return  # These don't go in p_queue

        elif m.limit_code in (CommandHeader.CC_DEBUG, CommandHeader.CC_ERROR, CommandHeader.CC_MESSAGE):
            if self.message_callback:
                self.message_callback(self, m)
            else:
                self.t_queue.append(m)
            return  # These don't go in p_queue

        elif m.limit_code == CommandHeader.CC_DONE:
            self.current_state = CurrentState(m.payload)
            self.last_done = m.seq

        elif m.limit_code == CommandHeader.CC_EMPTY:
            self.empty_event.set();

        self.p_queue.put(m)

        self.next_rcv_event.set()

    def _get_queue_response(self, seq, queue, timeout=1):
        t_start = time()

        while True:
            try:
                f = queue.pop()

                if f.limit_code in (CommandHeader.CC_ACK, CommandHeader.CC_ECHO) and f.seq == seq:
                    return f

                elif f.limit_code == CommandHeader.CC_NACK and f.seq == seq:
                    raise Exception('NACK')

            except IndexError:

                if (time() - t_start) > timeout:
                    raise TimeoutError

    def _get_flow_response(self, seq):
        return self._get_queue_response(seq, self.f_queue, timeout=1)

    def wait_empty(self, timeout=None):
        """Wait until the queue is empty"""
        self.empty_event.wait(timeout=timeout);

    def is_empty(self):
        return self.empty_event.is_set()

    def yield_recieve(self, timeout=30):
        while True:
            m = self.p_queue.get(timeout=timeout)
            yield m

    def wait_queue_time(self, v, timeout=None):
        """Return when the queue time is less than v"""

        if self.current_state.queue_time is None:
            return True

        for e in self.yield_recieve(timeout):
            if self.current_state.queue_time is not None and self.current_state.queue_time < v:
                return True

        return False

    def wait_recieve(self, timeout=None):
        """Wait until the next message is recieved"""
        pass

    def send(self, m):
        m.send_time = time()
        self.seq += 1
        m.seq = self.seq

        b = m.encode()

        self.write(b)
        self.next_rcv_event.clear()
        m.ack = self._get_flow_response(m.seq)

        return m

    def send_command(self, c):
        self.send(CommandHeader(seq=self.seq, code=c))

    def config(self, itr_delay: int, debug_print: bool = False, debug_tick: bool = False,
               axes: List[AxisConfig] = []):

        self.send(ConfigCommand(itr_delay, debug_print, debug_tick, axes))

    def move(self, axes: List[int]):
        m = MoveCommand(axes)

        m.done = False
        self.empty_event.clear();
        self.send(m)

    def resume(self):
        self.send_command(CommandHeader.CC_RUN)

    def stop(self):
        self.send_command(CommandHeader.CC_STOP)

    def info(self):
        self.send_command(CommandHeader.CC_INFO)

    def reset(self):
        self.send_command(CommandHeader.CC_RESET)


class ProtoPacket(Packetizer):

    def __init__(self, proto):
        super().__init__()
        self.proto = proto

    def handle_packet(self, packet):
        m = CommandHeader.decode(packet)
        self.proto.handle_stepper_message(m)

class EncoderReader():

    def __init__(self, port, serial_timeout = 1, yield_timeout=30) -> None:
        self.yield_timeout = yield_timeout
        self.ser = serial.serial_for_url(port, baudrate=115200, timeout=serial_timeout)

        self.reader = ReaderThread(self.ser, EncoderProto)


    def get(self, block=True, timeout=1):
        return self.reader.protocol.queue.get(block=block, timeout=timeout)

    def __enter__(self):
        self.reader.start();
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.reader.stop()

    def start(self):
        self.reader.start();

    def stop(self):
        self.reader.stop()

    def zero(self):
        self.ser.write(b'z')

    def clear_queue(self):
        try:
            while self.get(False):
                pass # clear the queues
        except:
            pass

    def read(self):
        try:
            m = self.get(False)
            return m
        except queue.Empty:
            return None

    def read_empty(self):
        l = []
        while True:
            m = self.read()
            if m:
                l.append(m)
            else:
                break

        return l


    def __iter__(self):
        yield from self.reader.protocol.yield_recieve(self.yield_timeout)

def main():

    with EncoderReader('/dev/cu.usbmodem6387471') as er:
        for i in range(10):
            m = er.get(20)
            print(m[0].direction, m[0].limit, m[0].position)




if __name__ == "__main__":
    main()
