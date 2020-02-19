import logging
import unittest
# from trajectory.proto import PacketProto
from time import sleep, time

import serial

from trajectory.messages import *
from trajectory.proto import ThreadedProto, SyncProto
from trajectory.segments import Joint, SegmentList
from random import randint
if (False):
    # FTDI serial
    packet_port = '/dev/cu.usbserial-AO0099HV'
    baudrate = 3000000
else:
    # Builtin USB
    packet_port = '/dev/cu.usbmodem64213901'
    #packet_port = '/dev/cu.usbmodem64213801'
    baudrate = 3000000 #20_000_000


class TestComplex(unittest.TestCase):

    def rand_header(self):

        from random import randint
        return CommandHeader(seq=randint(0, 255), code=randint(0, 255))

    def test_simple_send(self):

        ser = serial.Serial(packet_port, timeout=1, baudrate=baudrate)

        def do_test(seq):
            h1 = CommandHeader(seq=seq, code=CommandHeader.CC_ECHO)

            h1.payload = ('1234567890' * 30)[:10]

            b = h1.encode()

            ser.write(b)

            b2 = ser.read(len(b))

            self.assertEqual(b, b2);

            c = CommandHeader.decode(b2[:-1])

            self.assertEqual(h1.payload, c.payload.decode('ascii'))

        t_start = time()
        for i in range(10):
            do_test(i)

        print((time() - t_start) / 100)

    def test_echo_thread(self):

        logging.basicConfig(level=logging.DEBUG)

        rt = ThreadedProto(packet_port, baudrate);
        rt.start()

        h1 = CommandHeader(seq=1, code=CommandHeader.CC_ECHO)
        h1.payload = ('1234567890' * 30)[:1]

        t_s = time()
        for i in range(10):
            h1.seq = i
            rt.send(h1)

        r = (time() - t_s) / 10
        print("Time=", r)

    def test_sync_proto_config(self):

        def cb(p, m):
            print(m)

        logging.basicConfig(level=logging.DEBUG)

        p = SyncProto(packet_port, baudrate)

        p.info()

        p.read_all(timeout=.2) # Clear old messages.

        mx = (12e3, 3000e3)

        axes = [AxisConfig(0, 5, 2, 4, *mx),  # Y Axis
                AxisConfig(1, 6, 7, 4, *mx),  # Z Axis
                AxisConfig(2, 8, 9, 10, *mx),  # B Axis
                AxisConfig(3, 11, 12, 13, *mx),
                AxisConfig(4, 11, 12, 13, *mx),
                AxisConfig(5, 11, 12, 13, *mx)
                ]

        #axes = [AxisConfig(0, 2, 3, 4, 10e3, 3e5), AxisConfig(1, 5, 6, 7, 10e3, 3e5)]

        p.config(axes=axes)

        p.info()

        p.read();

    def test_sync_proto(self):

        import saleae, time
        s = saleae.Saleae()
        s.capture_start()
        time.sleep(.5)

        def cb(p, m):
            print(m)

        logging.basicConfig(level=logging.DEBUG)

        p = SyncProto(packet_port, baudrate)

        p.config(axes=[AxisConfig(0, 2, 3, 4, 30e3, 3e5),
                       AxisConfig(1, 5, 6, 7, 30e3, 3e5)])

        p.info()

        x = 5e3

        for j in range(100):
            for i in range(50):
                p.move([x, 0])
                p.move([0, x])

            x = -x

            for i in range(50):
                p.move([x, 0])
                p.move([0, x])

            x = randint(1e3,40e3)

            p.read_empty(cb);
            p.info()
            print(p.current_state.positions)


        sleep(.1);
        s.capture_stop()

    def test_r_move(self):

        import saleae, time
        s = saleae.Saleae()
        s.capture_start()
        time.sleep(.5)

        def cb(p,m):
            print(m)

        logging.basicConfig(level=logging.DEBUG)

        p = SyncProto(packet_port, baudrate)

        p.config(axes=[AxisConfig(0, 2, 3, 4, 10e3, 3e5)])

        p.rmove((5000,))
        p.rmove((10000,))
        p.rmove((20000,))
        p.rmove((40000,))
        p.rmove((80000,))
        p.rmove((16000,))

        p.read_empty(cb);

        p.info()

    def test_a_move(self):

        import saleae, time
        s = saleae.Saleae()
        s.capture_start()
        time.sleep(.5)

        def cb(p,m):
            print(m,p.current_state.positions)

        logging.basicConfig(level=logging.DEBUG)

        p = SyncProto(packet_port, baudrate)

        p.config(axes=[AxisConfig(0, 2, 3, 4, 50e3, 3e5)])

        p.amove((500,))
        p.amove((1000,))
        p.amove((2000,))
        p.amove((4000,))
        p.amove((8000,))
        p.amove((16000,))

        p.read_empty(cb);

        p.info()

    def test_jog_move(self):

        import saleae, time
        s = saleae.Saleae()
        s.capture_start()
        time.sleep(.5)

        def cb(p,m):
            print(m,p.current_state.positions)

        logging.basicConfig(level=logging.DEBUG)

        p = SyncProto(packet_port, baudrate)

        p.config(axes=[AxisConfig(0, 2, 3, 4, 50e3, 3e5)])

        p.amove((0,))
        p.jog(.1,(500,))
        p.jog(.1,(1000,))
        p.jog(.1,(2000,))
        p.jog(.1,(4000,))
        p.jog(.1,(8000,))
        p.jog(.1,(16000,))

        p.read_empty(cb);

        p.info()




    def test_stepped_moves(self):

        import saleae, time
        s = saleae.Saleae()
        s.capture_start()
        time.sleep(.5)

        def cb(p, m):
            print(m, p.queue_length,  p.queue_time)

        logging.basicConfig(level=logging.DEBUG)

        p = SyncProto(packet_port, baudrate)
        mx = (12e3, 3000e3)

        p.config(itr_delay=10,
                 axes=[AxisConfig(0, 5, 2, 4, *mx),  # Y Axis
                       AxisConfig(1, 6, 7, 4, *mx),  # Z Axis
                       AxisConfig(2, 8, 9, 10, *mx),  # B Axis
                       AxisConfig(3, 11, 12, 13, *mx)])  # ,

        r = list(range(10,800, 5))
        t = .05
        while True:
            for x in r:
                p.move([AxisSegment(0, x)], t=t)
                p.read_to_queue_len(4, cb);

            for x in reversed(r):
                p.move([AxisSegment(0, x)], t=t)
                p.read_to_queue_len(4, cb);

        p.info()

    def test_backforth(self):

        import saleae, time
        s = saleae.Saleae()
        s.capture_start()
        time.sleep(.5)

        def cb(p, m):
            print(m, p.queue_length,  p.queue_time)

        logging.basicConfig(level=logging.DEBUG)

        p = SyncProto(packet_port, baudrate)
        mx = (100e3, 5e6)

        p.config(itr_delay=4,
                 axes=[AxisConfig(0, 5, 2, 4, *mx),  # Y Axis
                       AxisConfig(1, 6, 7, 4, *mx),  # Z Axis
                       AxisConfig(2, 8, 9, 10, *mx),  # B Axis
                       AxisConfig(3, 11, 12, 13, *mx)])  # ,


        for i in range(5):
            for x in range(1000,150000,5000):
                p.move([AxisSegment(0, x) for _ in range(4)])
                p.move([AxisSegment(0, -x) for _ in range(4)])
                p.read_empty()

        p.info()

    def test_config(self):
        logging.basicConfig(level=logging.ERROR)

        #import saleae, time
        #s = saleae.Saleae()
        #s.capture_start()
        #time.sleep(.5)

        rt = ThreadedProto(packet_port, baudrate);
        rt.start()

        rt.send(CommandHeader(seq=1, code=CommandHeader.CC_INFO))

        sleep(.5)

        m = ConfigCommand(4, True, True, [
            AxisConfig(0, 2, 3, 4),
            AxisConfig(0, 5, 6, 7)  # ,
            # AxisConfig(0, 8, 9, 10)
        ])

        rt.send(m)
        rt.send(CommandHeader(seq=1, code=CommandHeader.CC_INFO))

        sleep(.5)

        rt.send(CommandHeader(seq=1, code=CommandHeader.CC_RUN))
        rt.send(CommandHeader(seq=1, code=CommandHeader.CC_INFO))

        rt.wait_recieve()

        def send_seg_list(sl):

            sl.update()
            sl.validate()

            def r(v):
                return int(round(v))

            for e in sl.sub_segments:
                moves = [AxisSegment(i, r(axis.x), r(axis.v_i), r(axis.v_f)) for i, axis in enumerate(e)]
                rt.move(moves)

        sl = SegmentList([Joint(80000, 300_000, 300_000), Joint(80000, 300_000, 300_000)])
        for i in range(2):
            sl.add_distance_segment([80_000, 10_000])

        send_seg_list(sl)

        rt.wait_recieve()

        rt.send(CommandHeader(seq=1, code=CommandHeader.CC_RUN))

        rt.wait_queue_time(1.5)

        for m in rt.yield_recieve():

            print('XXX', rt.last_done, (rt.current_state or object).__dict__, m)

            if rt.is_empty():
                break

        rt.wait_empty()

    def test_tones(self):

        logging.basicConfig(level=logging.DEBUG)

        rt = ThreadedProto(packet_port, baudrate);
        rt.start()

        rt.config(4, True, True, [AxisConfig(0, 2, 3, 4)])

        rt.send(CommandHeader(seq=1, code=CommandHeader.CC_INFO))

        rt.move([AxisSegment(0,50_000, 0, 10_000 )])
        rt.move([AxisSegment(0, 50_000, 10_000, 0)])

        rt.resume()

        rt.wait_empty()





if __name__ == '__main__':
    unittest.main()
