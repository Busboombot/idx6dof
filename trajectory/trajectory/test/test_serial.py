import logging
import queue
import unittest
from time import sleep, time

import serial
from test import make_axes

from trajectory.messages import *
from trajectory.proto import SyncProto


from random import randint

packet_port = '/dev/tty.usbmodem6421381'
encoder_port = '/dev/cu.usbmodem6387471'

baudrate = 115200 #20_000_000

logging.basicConfig(level=logging.DEBUG)


# Axis configurations for the robot
# Tuples are: step pin, dir pin, enable pin, max_v, max_a


class TestSerial(unittest.TestCase):
    # Determines wether the steppers are enables with an output value of high or low
    # Different for different stepper drivers
    ENABLE_OUTPUT = False

    def setUp(self) -> None:
        p = SyncProto(packet_port, encoder_port, baudrate)

        d = make_axes(500, 1_000, usteps=1, steps_per_rotation=48) # Just for stopping all axes

        p.config(4, self.ENABLE_OUTPUT, False, False, axes=d['axes6']);
        p.stop()
        p.close()

    def tearDown(self) -> None:
        p = SyncProto(packet_port, encoder_port, baudrate)

        d = make_axes(500, 1_000, usteps=1, steps_per_rotation=48) # Just for stopping all axes

        p.config(4, self.ENABLE_OUTPUT, False, False, axes=d['axes6']);
        p.stop()
        p.close()

    def init(self, v=500):
        d = make_axes(v, .1, usteps=16, steps_per_rotation=200)

        p = SyncProto(packet_port, encoder_port)
        p.encoder_multipliers[0] = 1 + (1 / 3)

        p.config(4, False, False, False, axes=d['axes1']);

        p.mspr = d['mspr']

        return p

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

        def cb(p, m):
            print(m)

        p = self.init(100)
        r = p.mspr

        p.zero()
        p.reset()

        p.run()
        p.amove((-r/2,))
        p.amove((-r/2,))
        p.amove((0,))
        p.runout(cb)
        return

        p.amove((r/2,))
        p.amove((-r/2,))
        p.amove((0,))
        p.runout()

        p.amove((r,))
        p.amove((0,))
        p.amove((0,))

        p.info()
        p.run()

        p.runout()

    def test_simple_r_move(self):

        def cb(p, m):
            print(p.queue_length, m)

        p = self.init(100)

        p.zero()
        p.reset()

        p.rmove((636,))

        p.runout(cb)

        sleep(.1)
        p.update()
        for m in p:
            print('!!!', m)

    def test_zero_move(self):

        def cb(p, m):
            print(m)

        p = self.init(100)

        p.zero()
        p.reset()

        p.rmove((0,))
        p.info()
        p.run()

        p.runout(cb)
        p.info()


    def test_find_limit(self):

        def cb(p, m):
            print(m)
        p = self.init(800)

        p.zero()
        p.reset()
        p.run()

        for i in range(0,p.mspr, int(p.mspr/8)):
            p.amove((i,))
            p.amove((-i,))
            p.runout()
            ll = p.axis_state[0].hl_limit
            if(ll):
                print("Found", ll)
                break

        p.amove((ll + p.mspr/4,))
        p.runempty(cb)


    def test_reset(self):

        def cb(p, m):
            if isinstance(m,list):
                print(m[0], p.axis_state[0])

        p = self.init(1000)

        p.zero()
        p.reset()
        p.runout()

        print(p.axis_state)


    def home_seek(self):

        p = self.init(1000)

        p.zero()
        p.reset()
        p.run()

        for i in range(0, p.mspr, int(p.mspr / 16)):
            p.amove((i,))
            p.amove((-i,))
            p.runout()
            ll = p.axis_state[0].hl_limit
            if (ll):
                break

        p.amove((ll + int(p.mspr / 4),))
        p.runout()

    def test_home(self):
        self.home_seek();

    def home_poll(self, p):

        lc = p.pollEncoders().encoders[0].limit_code
        if  lc == LimitCode.LL:
            direction = -1
        elif lc == LimitCode.HH:
            direction = +1
        else:
            assert(False)

        for i in range(0, int(p.mspr/.6), int(p.mspr / 8)):
            p.amove((direction*i,))
            p.runout()
            ll = p.axis_state[0].hl_limit
            if (ll):
                break

        p.amove((ll + int(p.mspr / 4),))
        p.runout()

    def test_home_poll(self):

        p = self.init(1500)

        p.zero()
        p.reset()
        p.run()

        def rand_move():
            from random import randint
            return randint(-p.mspr, p.mspr)

        for i in range(10):
            p.rmove((rand_move(),))

            self.home_poll(p)

            sleep(.3)





if __name__ == '__main__':
    unittest.main()
