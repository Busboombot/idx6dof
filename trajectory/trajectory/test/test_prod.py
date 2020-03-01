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
        #import saleae, time
        #s = saleae.Saleae()
        #s.capture_start()
        #time.sleep(.5)

        def cb(p,m):
            print(m,p.current_state.positions)

        logging.basicConfig(level=logging.DEBUG)

        p = SyncProto(packet_port, baudrate)

        v_max = 5e3
        a_max = 3e6

        d = v_max / 3

        axes=[
            AxisConfig(0, 21,21,23, v_max, a_max),    # X
            AxisConfig(1, 18,19,20, v_max, a_max), # Y
            AxisConfig(2, 15,16,17, v_max, a_max), # Z
            AxisConfig(3, 5,6,7, v_max, a_max), # C
            AxisConfig(4, 8,9,10, v_max, a_max), # A
            AxisConfig(5, 2,3,4, v_max, a_max)   , # B
        ]

        p.config(axes=axes)

        move = [d,d,d,d,d,d]

        while(True):
            p.amove(move)
            p.amove([-e for e in move])

            p.read_empty(cb);

        p.info()


if __name__ == '__main__':
    unittest.main()
