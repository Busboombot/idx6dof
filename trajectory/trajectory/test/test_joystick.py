import unittest
from time import sleep, time
from trajectory.joystick import PygameJoystick


usb_prog_port = '/dev/cu.usbmodem1464101'
usb_native_port = '/dev/cu.usbmodem14644201'
usb_baud = 115200
# usb_port = '/dev/ttyACM0'
n_axes = 6


class TestComplex(unittest.TestCase):

    def test_joystick(self):

        for e in PygameJoystick():
            print(e)

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

                #proto.write(msg)

                seq += 1

                last_velocities = velocities
            elif dt < .20:
                sleep(.20 - dt)

if __name__ == '__main__':
    unittest.main()
