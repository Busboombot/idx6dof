import logging
import unittest
# from trajectory.proto import PacketProto
from time import sleep, time

from test import make_axes
from trajectory.joystick import PygameJoystick
from trajectory.proto import SyncProto
from . import TestStepper

packet_port = '/dev/cu.usbmodem64213801'  # Production
encoder_port = '/dev/cu.usbmodem63874601'  # Production
baudrate = 115200

logging.basicConfig(level=logging.DEBUG)
from trajectory.messages import OutMode

class TestComplex(TestStepper):

    def test_joystick(self):


        def cb(p, m):
            if m.name != 'MESSAGE':
                print(m)

        p = self.init(packet_port, encoder_port,
                      300, 'axes6', a=1, usteps=10, use_encoder=False,
                      outmode=OutMode.OUTPUT_OPENDRAIN, period=5
                      )
        p.reset()
        p.run()

        r = p.mspr
        s = p.x_1sec
        jog_interval = .1


        for e in PygameJoystick(t=.1):
            moves = [a / 1000 * s for a in e.axes]
            print(moves)
            p.jog(jog_interval * 2, moves)

            p.runout(cb)

        p.info()
        p.stop()

    def test_random_jog(self):

        from random import random

        def cb(p, m):
            print(m)

        p = SyncProto(packet_port, baudrate)

        d = make_axes(250, .2, usteps=32, steps_per_rotation=48)
        jog_interval = .5  # Secs between jog messages
        s = d['x_1sec'] * jog_interval  # Max steps between jog intervals, but for a bit longer than the

        p.config(4, True, False, False, axes=d['axes6']);
        p.run()

        for i in range(10_000_000):
            moves = [2 * (random() - .5) * s for i in range(6)]
            print(jog_interval * 1.5, moves)
            p.jog(jog_interval * 1.5, moves)
            sleep(jog_interval)

        p.read_empty(cb);

        p.info()
        p.stop()

    def test_joy_move(self):

        def get_joy():
            while True:
                with open('/tmp/joystick') as f:
                    return [float(e) for e in f.readline().split(',')]

        last_time = time()
        last_velocities = [0] * 6
        seq = 0

        while True:

            e = get_joy()

            dt = time() - last_time

            if dt >= .20 and len(proto) <= 2:
                last_time = time()

                velocities = e + [0, 0]

                x = [.5 * (v0 + v1) * dt for v0, v1 in zip(last_velocities, velocities)]

                msg = Command(seq, 10, dt * 1e6, last_velocities, velocities, x)

                # proto.write(msg)

                seq += 1

                last_velocities = velocities
            elif dt < .20:
                sleep(.20 - dt)


if __name__ == '__main__':
    unittest.main()
