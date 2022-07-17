from __future__ import print_function

import logging
from collections import deque
from queue import Queue
from threading import Event, Semaphore
from time import time

import serial
from serial.threaded import ReaderThread, Packetizer

from .messages import *

logger = logging.getLogger('message')

# These parameters must match those in the  firmware,
# in idx_stepper.h
TIMEBASE = 1_000_000  # microseconds
N_BIG = 2 ** 32 - 1  # ULONG_MAX
fp_bits = 8  # Bits in the fraction portion of the floating point representation


class TimeoutException(Exception):
    pass


class ProtocolException(Exception):
    pass


class ProtoPacket(Packetizer):

    def __init__(self, proto):
        super().__init__()
        self.proto = proto

    def handle_packet(self, packet):
        m = CommandHeader.decode(packet)
        self.proto.handle_packet(m)


class ThreadedProto(ReaderThread):

    def __init__(self, port, baud, message_callback=None):
        self.ser = serial.Serial(port, baudrate=baud)

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

                if m.code == CommandHeader.CC_DEBUG:
                    logger.debug(pl)
                elif m.code == CommandHeader.CC_ERROR:
                    logger.error(pl)
                else:
                    logger.info(pl)

        self.message_callback = _message_callback

        def factory():
            return ProtoPacket(self)

        super(ThreadedProto, self).__init__(self.ser, factory)

    def start(self):
        super(ThreadedProto, self).start()
        self._connection_made.wait()
        if not self.alive:
            raise RuntimeError('connection_lost already called')

    def handle_packet(self, m):

        m.recieve_time = time()

        if m.code in (CommandHeader.CC_ACK, CommandHeader.CC_NACK, CommandHeader.CC_ECHO):
            self.f_queue.append(m)
            return  # These don't go in p_queue

        elif m.code in (CommandHeader.CC_DEBUG, CommandHeader.CC_ERROR, CommandHeader.CC_MESSAGE):
            if self.message_callback:
                self.message_callback(self, m)
            else:
                self.t_queue.append(m)
            return  # These don't go in p_queue

        elif m.code == CommandHeader.CC_DONE:
            self.current_state = CurrentState(m.payload)
            self.last_done = m.seq

        elif m.code == CommandHeader.CC_EMPTY:
            self.empty_event.set();

        self.p_queue.put(m)

        self.next_rcv_event.set()

    def _get_queue_response(self, seq, queue, timeout=1):
        t_start = time()

        while True:
            try:
                f = queue.pop()

                if f.code in (CommandHeader.CC_ACK, CommandHeader.CC_ECHO) and f.seq == seq:
                    return f

                elif f.code == CommandHeader.CC_NACK and f.seq == seq:
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


class SyncProto(object):

    def __init__(self, port, baud, timeout=.1, message_callback=None):
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)

        if message_callback is None:
            def _message_callback(proto, m):

                pl = m.payload.decode('ascii')

                if m.code == CommandHeader.CC_DEBUG:
                    logger.debug(pl)
                elif m.code == CommandHeader.CC_ERROR:
                    logger.error(pl)
                else:
                    logger.info(pl)

        self.message_callback = _message_callback

        self.empty = True
        self.seq = 0;
        self.last_ack = -1;
        self.last_done = -1;
        self.current_state = CurrentState()

        self.buffer = bytearray()


    @property
    def queue_length(self):
        return float(self.current_state.queue_length)

    @property
    def queue_time(self):
        return float(self.current_state.queue_time)/1e6

    def handle_packet(self, m):

        m.recieve_time = time()

        if m.code in (CommandHeader.CC_ACK, CommandHeader.CC_NACK, CommandHeader.CC_ECHO):
            self.last_ack = m.seq
            return  # These don't go in p_queue

        elif m.code in (CommandHeader.CC_DEBUG, CommandHeader.CC_ERROR, CommandHeader.CC_MESSAGE):
            if self.message_callback:
                self.message_callback(self, m)
            else:
                pass
            return  # These don't go in p_queue

        elif m.code == CommandHeader.CC_DONE:
            self.current_state = CurrentState(m.payload)
            if self.queue_length == 0:
                self.empty = 0;
            self.last_done = m.seq

        elif m.code == CommandHeader.CC_EMPTY:
            self.empty = True;

    def read(self):
        TERMINATOR = b'\0'

        data = self.ser.read()

        if data == TERMINATOR:
            m = CommandHeader.decode(self.buffer)
            self.handle_packet(m)
            self.buffer = bytearray()

            return m
        else:
            self.buffer.extend(data)
            return False;

    def read_all(self,  cb=None, timeout = 1):
        """Read until the timeout"""
        start = time()
        while True:
            m = self.read();
            if m and cb:
                cb(self, m)
            if time()-start > timeout:
                break

    def read_empty(self, cb=None):

        while True:
            m = self.read();
            if m and cb:
                cb(self, m)
            if self.empty:
                break

    def read_to_queue_len(self, l, cb=None):

        while True:
            m = self.read();
            if m and cb:
                cb(self, m)

            if self.current_state.queue_length < l:
                break

    def read_to_queue_time(self, t, cb=None):

        while True:
            m = self.read();
            if m and cb:
                cb(self, m)
            if self.current_state.queue_time < t:
                break

    def send(self, m):

        m.send_time = time()
        self.seq += 1
        m.seq = self.seq

        b = m.encode()

        self.ser.write(b)

        # Read until we get the ack
        while True:
            self.read()
            if self.last_ack == m.seq:
                break

        return m

    def send_command(self, c):
        self.send(CommandHeader(seq=self.seq, code=c))

    def config(self, itr_delay: int = 4,
               enable_active=True, debug_print: bool = False, debug_tick: bool = False,
               axes: List[AxisConfig] = []):

        # Send the top level config, to set the number of
        # axes
        self.send(ConfigCommand(len(axes), itr_delay, enable_active, debug_print, debug_tick))

        # Then send the config for each axis.
        for ac in axes:
            self.send(ac)

    def _move(self, code:int, x: List[int], t=0):
        m = MoveCommand(code, x, t=t)

        m.done = False

        self.current_state.queue_length += 3;

        self.send(m)
        self.empty = False;

    def amove(self, x: List[int]):
        """Absolute position move"""
        self._move(CommandHeader.CC_AMOVE, x, t=0)

    def rmove(self, x: List[int]):
        "Relative position move"
        self._move(CommandHeader.CC_RMOVE, x, t=0)

    def jog(self, t:float, x: List[int]):
        """Jog move. A jog move replaces the last move on the (step generator side)
        planner, then becomes a regular relative move. """
        self._move(CommandHeader.CC_JMOVE, x, t=t)

    def run(self):
        self.send_command(CommandHeader.CC_RUN)

    def stop(self):
        self.send_command(CommandHeader.CC_STOP)

    def info(self):
        self.send_command(CommandHeader.CC_INFO)

    def reset(self):
        self.send_command(CommandHeader.CC_RESET)
