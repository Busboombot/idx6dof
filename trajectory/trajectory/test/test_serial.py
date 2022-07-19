import logging
import queue
import unittest
# from trajectory.proto import PacketProto
from time import sleep, time

import serial
from test import make_axes

from trajectory.messages import *
from trajectory.proto import ThreadedProto, SyncProto
from random import randint
if (False):
    # FTDI serial
    packet_port = '/dev/cu.usbserial-AO0099HV'
    baudrate = 115200
else:
    # Builtin USB
    packet_port = '/dev/tty.usbmodem6421381'
    #packet_port = '/dev/cu.usbmodem64213801'
    baudrate = 115200 #20_000_000

logging.basicConfig(level=logging.DEBUG)


# Axis configurations for the robot
# Tuples are: step pin, dir pin, enable pin, max_v, max_a


class TestSerial(unittest.TestCase):
    # Determines wether the steppers are enables with an output value of high or low
    # Different for different stepper drivers
    ENABLE_OUTPUT = False

    def setUp(self) -> None:
        p = SyncProto(packet_port, baudrate)

        d = make_axes(500, 1_000, usteps=1, steps_per_rotation=48) # Just for stopping all axes

        p.config(4, self.ENABLE_OUTPUT, False, False, axes=d['axes6']);
        p.stop()

    def tearDown(self) -> None:
        p = SyncProto(packet_port, baudrate)

        d = make_axes(500, 1_000, usteps=1, steps_per_rotation=48) # Just for stopping all axes

        p.config(4, self.ENABLE_OUTPUT, False, False, axes=d['axes6']);
        p.stop()

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
        h1.payload = ('1234567890' * 30)[:10]

        t_s = time()
        for i in range(10):
            h1.seq = i
            rt.send(h1)


        r = (time() - t_s) / 10
        print("Time=", r)

    def test_info(self):

        def cb(p, m):
            print(m)

        logging.basicConfig(level=logging.DEBUG)

        p = SyncProto(packet_port, baudrate)
        d = make_axes(300, .1, usteps=32, steps_per_rotation=48)

        p.config(4, ENABLE_OUTPUT, False, False, axes=d['axes6']);
        p.info()

        return

        p.run()
        sleep(.5);

        p.info()

        p.stop()

        p.info()

        p.read_all(timeout=.2)  # Clear old messages.

    def test_sync_proto_config(self):

        # Causes board to crash

        def cb(p, m):
            print(m)

        logging.basicConfig(level=logging.DEBUG)

        p = SyncProto(packet_port, baudrate)

        p.info()

        p.read_all(timeout=.2) # Clear old messages.

        p.config(axes=axes6)

        p.info()

        p.read();

    def test_sync_proto(self):

        #import saleae, time
        #s = saleae.Saleae()
        #s.capture_start()
        #time.sleep(.5)

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

    def test_simple_r_1_move(self):
        """A simple move with 1 axis"""
        def cb(p,m):
            print(m, p.queue_time)

        logging.basicConfig(level=logging.DEBUG)

        d = make_axes(1000, 1, usteps=16, steps_per_rotation=200)

        p = SyncProto(packet_port, baudrate)
        p.config(4, self.ENABLE_OUTPUT, False, False, axes=d['axes1']);

        p.run()
        s = d['x_1sec']
        dur = 2
        print(1*s)

        for i in range(2):
            p.rmove((dur/2*s,))
            p.rmove((-dur * s,))
            p.rmove((dur / 2 * s,))
            p.read_empty(cb);
            p.info()

    def test_r_move_a1(self):

        #import saleae, time
        #s = saleae.Saleae()
        #s.capture_start()
        #time.sleep(.5)

        def cb(p,m): # Callback for messages from the sg
            print(m)

        logging.basicConfig(level=logging.DEBUG)

        p = SyncProto(packet_port, baudrate)

        p.config(4, True, False, False, axes=axes1);

        s = 5*x_1sec
        for i in range(1):
            p.rmove((s,))
            p.rmove((-s,))

        p.run()
        p.read_empty(cb);

        p.info()

    def test_r_move_a2_reversing(self):

        def cb(p,m):
            print(m, p.queue_time)

        logging.basicConfig(level=logging.DEBUG)

        p = SyncProto(packet_port, baudrate)

        p.config(4, True, False, False, axes=axes2);

        s = x_1sec

        for i in range(3):

            p.rmove((s, -s/2))
            p.rmove((-s, s/2))
            p.rmove((s/3, -s))
            p.rmove((-s/3, s))

        p.info()
        t1 = time()
        p.run()
        p.read_empty(cb)
        td = time() - t1
        print("Time: ", td)
        p.stop()
        p.info()

    def test_r_move_a3(self):


        def cb(p,m):
            print(m)

        logging.basicConfig(level=logging.DEBUG)

        p = SyncProto(packet_port, baudrate)

        d = make_axes(300, .1, usteps=32, steps_per_rotation=48)
        s = d['x_1sec']

        p.config(4, True, False, False, axes=d['axes3']);

        p.rmove((   1*s,   .5*s,   1*s))
        p.rmove((   -1*s,  .25*s, -1*s))
        p.rmove((   1*s,   1*s,   1*s))
        p.rmove((   -1*s,  .5*s,  -1*s))
        p.run()
        p.read_empty(cb);

        p.info()

    def test_a_move(self):

        #import saleae, time
        #s = saleae.Saleae()
        #s.capture_start()
        #time.sleep(.5)

        def cb(p,m):
            print(m,p.current_state.positions)

        logging.basicConfig(level=logging.DEBUG)

        p = SyncProto(packet_port, baudrate)

        mx = (20000, 500000)  # For Stepper Trainer
        #p.config(axes=[AxisConfig(0, 2, 3, 4, *mx)])
        p.config(axes=[AxisConfig(0, 5, 6, 7, *mx)])

        for i in range(10):
            p.rmove((10000,))
            p.rmove((-10000,))

        p.run()
        p.read_empty(cb);

        p.info()

    def test_encoder(self):

        from encoder import EncoderReader

        def cb(p,m):
            print(m,p.current_state.positions)

        d = make_axes(500, 1, usteps=16, steps_per_rotation=200)

        p = SyncProto(packet_port, baudrate)
        p.config(4, self.ENABLE_OUTPUT, False, False, axes=d['axes1']);

        logging.basicConfig(level=logging.DEBUG)

        er = EncoderReader('/dev/cu.usbmodem6387471')
        er.start()

        mspr = d['mspr'] # Pulses per rotation
        divs = 4
        x = mspr/divs

        for i in range(divs*2):
            p.amove((x,))
            p.amove((x*2,))
            p.amove((x*3,))

        p.run()
        p.info()

        p.read_empty(cb);

        p.info()

    def test_find_limit(self):

        from encoder import EncoderReader

        def cb(p,m):

            print(m,p.current_state.positions)

        d = make_axes(1500, 1, usteps=16, steps_per_rotation=200)

        p = SyncProto(packet_port, baudrate)
        p.config(4, self.ENABLE_OUTPUT, False, False, axes=d['axes1']);

        logging.basicConfig(level=logging.DEBUG)

        er = EncoderReader('/dev/cu.usbmodem6387471')
        er.start()

        #print(m[0].direction, m[0].limit, m[0].position)

        p.rmove((1000,))

        p.run()


        def jog_to_next_limit(x):
            er.clear_queue()
            for i in range(1000):
                p.jog(.1, (x,))

                try:
                    m = er.get(True, timeout=0.1)
                    return m

                except queue.Empty:
                    pass

            er.clear_queue()

        jog_to_next_limit(500)
        jog_to_next_limit(-100)
        jog_to_next_limit(50)
        jog_to_next_limit(-10)

        if False:
            p.jog(.1, (200,))
            p.jog(.1, (200,))
            p.jog(.1,(500,))
            p.jog(.1,(1000,))
            p.jog(.1,(2000,))
            p.jog(.1,(4000,))
            p.jog(.1,(-8000,))

        er.stop()


    def test_stepped_moves(self):

        #import saleae, time
        #s = saleae.Saleae()
        #s.capture_start()
        #time.sleep(.5)

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






if __name__ == '__main__':
    unittest.main()
