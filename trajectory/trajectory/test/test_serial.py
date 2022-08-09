import logging
import queue
import unittest
from time import sleep, time

import serial
from test import make_axes

from trajectory.messages import *
from trajectory.proto import SyncProto

from random import randint

#packet_port = '/dev/cu.usbmodem64213801'  # Production
packet_port = '/dev/cu.usbmodem64213901' # Test
# encoder_port = '/dev/cu.usbmodem6387471'
encoder_port = '/dev/cu.usbmodem63874601'  # Production

baudrate = 115200  # 20_000_000

logging.basicConfig(level=logging.DEBUG)


# Axis configurations for the robot
# Tuples are: step pin, dir pin, enable pin, max_v, max_a

class TestSerial(unittest.TestCase):
    # Determines wether the steppers are enables with an output value of high or low
    # Different for different stepper drivers
    ENABLE_OUTPUT = False

    def setUp(self) -> None:
        pass

    def tearDown(self) -> None:
        pass

    def init(self, v=800, axes_name='axes1', usteps=16, a=.1,
             highvalue=OutVal.HIGH, outmode=OutMode.OUTPUT_OPENDRAIN,
             segment_pin=27, limit_pint=29, period=4,
             use_encoder=True):

        d = make_axes(v, a, usteps=usteps, steps_per_rotation=200,
                      highval=highvalue, output_mode=outmode)

        p = SyncProto(packet_port, encoder_port if use_encoder else None)
        p.encoder_multipliers[0] = 1 + (1 / 3)

        p.config(period, segment_pin, limit_pint, False, False, axes=d[axes_name]);

        p.mspr = d['mspr']
        p.x_1sec = d['x_1sec']

        return p

    def rand_header(self):

        from random import randint
        return CommandHeader(seq=randint(0, 255), code=randint(0, 255))

    def test_read_message(self):

        p = SyncProto(packet_port, None)

        while True:
            p.update()
            for m in p:
                if cb:
                    cb(p, m)

    def test_config(self):
        """Test changing the configuration"""

        p = SyncProto(packet_port, None)

        d = make_axes(500, .1, usteps=16, steps_per_rotation=200)
        p.config(4, 18, 32, False, False, axes=d['axes1']);
        p.info()

        d = make_axes(1000, .2, usteps=16, steps_per_rotation=200,
                      output_mode=OutMode.OUTPUT_OPENDRAIN, highval=OutVal.LOW)
        p.config(4, 7, 9, False, False, axes=d['axes1']);
        p.info()

    def test_info(self):

        def cb(p, m):
            print(m)

        logging.basicConfig(level=logging.DEBUG)

        p = self.init(100, 'axes1', a=0.1, use_encoder=False)
        p.reset()

        p.run()
        sleep(5)
        p.stop()
        p.rmove((1000,))
        sleep(5)
        p.run()
        sleep(5)
        p.stop()

    def test_open_poll(self):
        def cb(p, m):
            print(m)

        p = self.init(600, 'axes6', a=.3, usteps=10, use_encoder=True,
                      highvalue=OutVal.HIGH, outmode=OutMode.OUTPUT_OPENDRAIN, period=5)

        p.run()
        p.stop()

        while True:
            p.runout(cb, timeout=1)

    def test_run_axis(self):

        def cb(p, m):
            if m.name != 'MESSAGE':
                print(m)

        p = self.init(800, 'axes6', a=.3, usteps=10, use_encoder=True,
                      highvalue=OutVal.HIGH, outmode=OutMode.OUTPUT_OPENDRAIN,
                      period=5)

        r = p.mspr
        p.reset()
        p.run()

        s = p.x_1sec * 1

        axis = 3
        try:
            while True:
                for i in range(10):
                    p.rmove({axis: s})
                    p.rmove({axis: -s})
                    p.runout(cb)
        except KeyboardInterrupt:
            p.runout()

        p.reset()


    # noinspection PyTypeChecker
    def test_simple_r_move(self):

        def cb(p, m):
            if m.name != 'MESSAGE':
                print(m)

        p = self.init(1000, 'axes6', a=.3, usteps=10, use_encoder=True,
                      highvalue=(OutVal.HIGH, OutVal.HIGH, OutVal.LOW),
                      segment_pin=27, limit_pint=29,
                      outmode=OutMode.OUTPUT, period=5
                      )
        p.reset()
        p.run()

        r = p.mspr
        s = p.x_1sec

        # Random Move, forward or back 1/2 rodataion.
        rr = randint(int(-r / 2.5), int(r / 2.5))
        p.rmove((rr,) * 6)
        p.runout()

        def find_limit(p, encoder=0, direction_mod=1):

            r = p.mspr

            lc = p.pollEncoders().encoders[encoder].limit_code
            if lc == LimitCode.LL:
                direction = -1 * direction_mod
            elif lc == LimitCode.HH:
                direction = +1 * direction_mod
            else:
                assert False

            # Step toward the LH limit
            for i in range(200):
                p.hmove((direction * r / 200,) * 6)

            p.runout()

        find_limit(p, 0)

        # Now we've found the limit, but to get an accurate
        # reading, we always need to read it from the same side.

        p.rmove((r / 20,) * 6)
        p.runout()

        lc = p.pollEncoders().encoders[0].limit_code

        # Step back toward the LH limit
        for i in range(40):
            p.hmove((-1 * r / 400,) * 6)
        p.runout()

        print("!!!", lc)

        print(p.pollEncoders().encoders[0])
        p.zero()
        p.runout()

    def test_simple_r_1_move(self):
        """A simple move with 1 axis"""

        def cb(p, m):
            print(m, p.queue_time)

        logging.basicConfig(level=logging.DEBUG)

        d = make_axes(1000, 1, usteps=16, steps_per_rotation=200)

        p = SyncProto(packet_port, None, baudrate)
        p.config(4, self.ENABLE_OUTPUT, False, False, axes=d['axes1']);

        p.run()
        s = d['x_1sec']

        for i in range(100):
            p.rmove((s,))
            p.rmove((-s,))
            p.runout(cb, timeout=1);

        p.info()
        p.stop()

    def test_r_move_a1(self):

        # import saleae, time
        # s = saleae.Saleae()
        # s.capture_start()
        # time.sleep(.5)

        def cb(p, m):  # Callback for messages from the sg
            print(m)

        logging.basicConfig(level=logging.DEBUG)

        p = SyncProto(packet_port, baudrate)

        p.config(4, True, False, False, axes=axes1);

        s = 5 * x_1sec
        for i in range(1):
            p.rmove((s,))
            p.rmove((-s,))

        p.run()
        p.read_empty(cb);

        p.info()

    def test_r_move_a2_reversing(self):

        def cb(p, m):
            print(m, p.queue_time)

        logging.basicConfig(level=logging.DEBUG)

        p = SyncProto(packet_port, baudrate)

        p.config(4, True, False, False, axes=axes2);

        s = x_1sec

        for i in range(3):
            p.rmove((s, -s / 2))
            p.rmove((-s, s / 2))
            p.rmove((s / 3, -s))
            p.rmove((-s / 3, s))

        p.info()
        t1 = time()
        p.run()
        p.read_empty(cb)
        td = time() - t1
        print("Time: ", td)
        p.stop()
        p.info()

    def test_r_move_a3(self):

        def cb(p, m):
            print(m)

        logging.basicConfig(level=logging.DEBUG)

        p = SyncProto(packet_port, baudrate)

        d = make_axes(300, .1, usteps=32, steps_per_rotation=48)
        s = d['x_1sec']

        p.config(4, True, False, False, axes=d['axes3']);

        p.rmove((1 * s, .5 * s, 1 * s))
        p.rmove((-1 * s, .25 * s, -1 * s))
        p.rmove((1 * s, 1 * s, 1 * s))
        p.rmove((-1 * s, .5 * s, -1 * s))
        p.run()
        p.read_empty(cb);

        p.info()

    def test_simple_r_move_x(self):

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

        for i in range(0, p.mspr, int(p.mspr / 8)):
            p.amove((i,))
            p.amove((-i,))
            p.runout()
            ll = p.axis_state[0].hl_limit
            if (ll):
                print("Found", ll)
                break

        p.amove((ll + p.mspr / 4,))
        p.runempty(cb)

    def test_reset(self):

        def cb(p, m):
            if isinstance(m, list):
                print(m[0], p.axis_state[0])

        p = self.init(1000)

        p.zero()
        p.reset()
        p.runout()

        print(p.axis_state)

    def home_poll(self, p):
        """Poll the limit swtich, then advance in the direction of the HL limit"""

        lc = p.pollEncoders().encoders[0].limit_code
        if lc == LimitCode.LL:
            direction = -1
        elif lc == LimitCode.HH:
            direction = +1
        else:
            assert False

        for i in range(0, int(p.mspr / .6), int(p.mspr / 8)):
            p.amove((direction * i,))
            p.runout()
            ll = p.axis_state[0].hl_limit
            if (ll):
                break

        p.amove((ll + int(p.mspr / 4),))
        p.runout()

    def home_poll_hall(self, p, step_size=1, max_range=100):
        """Poll the limit swtich, then advance in the direction of the HL limit,
        but don't rely on the encoders"""

        step_size = int(step_size)

        p.runout(timeout=.5)

        lc = p.pollEncoders().encoders[0].limit_code
        if lc == LimitCode.LL:
            direction = -1
            other = LimitCode.HH
        elif lc == LimitCode.HH:
            direction = +1
            other = LimitCode.LL
        else:
            assert (False)

        for i in range(0, int(max_range), int(step_size)):

            p.rmove((direction * i,))
            p.runout(timeout=.05)

            this_lc = p.pollEncoders().encoders[0].limit_code

            if this_lc == other:
                break

        p.runout()

    def test_home_poll_hall(self):

        p = self.init(1000)
        p.run()

        p.zero()
        p.reset()

        def rand_move():
            from random import randint
            return randint(-p.mspr / 2, p.mspr / 2)

        for i in range(10):
            p.rmove((rand_move(),))
            p.runout()

            self.home_poll_hall(p, p.mspr / 32, p.mspr / 1.5)
            self.home_poll_hall(p, 1, p.mspr / 16)

            lc = p.pollEncoders().encoders[0].limit_code
            if lc == LimitCode.LL:
                self.home_poll_hall(p, 1, p.mspr / 16)

            print(p.axis_state[0], p.axis_state[0].epos % p.mspr)

        sleep(2)

    def home_seek(self, p):

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
        p = self.init(1000)
        p.run()

        p.zero()
        p.reset()

        self.home_seek();

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
