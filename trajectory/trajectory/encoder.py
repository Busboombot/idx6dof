import serial
import sys
import time
from serial.threaded import ReaderThread, Packetizer, Protocol
from cobs import cobs
import struct
from queue import Queue

msg_fmt = (
    '6c' +  # limit_states[6]
    'h'  + # directions
    '6i'   # Positions[6]
  )

from dataclasses import dataclass

@dataclass
class Encoder:
    """Class for keeping track of an item in inventory."""
    limit_state: int
    position: float

    @property
    def limit(self):
        return self.limit_state & 3

    @property
    def direction(self):
        return (self.limit_state & 4) >> 2


class EncoderProto(Packetizer):
    def __init__(self):
        super().__init__()

        self.queue = Queue()
        self.encoders  = []*6

    def connection_made(self, transport):
        super().connection_made(transport)
        print('port opened\n')

    def handle_packet(self, data):

        try:
            v = struct.unpack(msg_fmt, cobs.decode(data))
            self.encoders = [Encoder(int.from_bytes(l, "big"),p) for l, p in zip(v[:6], v[-6:])]

            self.queue.put(self.encoders)

        except Exception as e:
            print (e)

    def yield_recieve(self, timeout=30):
        while True:
            m = self.queue.get(timeout=timeout)
            yield m

    def connection_lost(self, exc):
        if exc:
            raise exc
        print('port closed\n')

class EncoderReader():

    def __init__(self, port, serial_timeout = 1, yield_timeout=30) -> None:
        self.yield_timeout = yield_timeout
        ser = serial.serial_for_url(port, baudrate=115200, timeout=serial_timeout)

        self.reader = ReaderThread(ser, EncoderProto)


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

    def clear_queue(self):
        try:
            while self.get(False):
                pass # clear the queues
        except:
            pass

    def __iter__(self):
        yield from self.reader.protocol.yield_recieve(self.yield_timeout)

def main():

    with EncoderReader('/dev/cu.usbmodem6387471') as er:
        for i in range(10):
            m = er.get(20)
            print(m[0].direction, m[0].limit, m[0].position)




if __name__ == "__main__":
    main()
