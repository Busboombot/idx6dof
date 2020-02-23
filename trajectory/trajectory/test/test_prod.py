import logging
import unittest

from trajectory.messages import *
from trajectory.proto import SyncProto
from trajectory.segments import Joint, SegmentList

# from trajectory.proto import PacketProto
if (False):
    # FTDI serial
    packet_port = '/dev/cu.usbserial-AO0099HV'
    baudrate = 3000000
else:
    # Builtin USB
    packet_port = '/dev/cu.usbmodem64213901'
    #packet_port = '/dev/cu.usbmodem64213801'
    baudrate = 3000000 #20_000_000


def make_sl(moves, a_max=300_000):
    sl = SegmentList([Joint(10000, a_max), Joint(10000, a_max)])

    for move in moves:
        sl.add_distance_segment(move)

    return sl

class TestBasic(unittest.TestCase):

    def test_basic(self):
        import saleae, time
        s = saleae.Saleae()
        s.capture_start()
        time.sleep(.5)

        def cb(p,m):
            print(m,p.current_state.positions)

        logging.basicConfig(level=logging.DEBUG)

        p = SyncProto(packet_port, baudrate)

        axes=[
            AxisConfig(0, 7,6,5, 50e3, 3e5),    # X
            AxisConfig(1, 4,3,2, 50e3, 3e5), # Y
            AxisConfig(2, 20,19,18, 50e3, 3e5), # Z
            AxisConfig(3, 10,9,8, 50e3, 3e5), # C
            AxisConfig(4, 23,22,21, 50e3, 3e5), # A
            AxisConfig(5, 17,16,15, 50e3, 3e5)   , # B
        ]

        p.config(axes=axes)

        d = 40000

        move = [0,0,0,0,d,0]

        while(True):
            p.amove(move)
            p.amove([-e for e in move])

            p.read_empty(cb);

        p.info()


if __name__ == '__main__':
    unittest.main()
