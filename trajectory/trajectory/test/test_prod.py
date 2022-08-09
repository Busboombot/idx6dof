import logging
import unittest
from time import sleep

from test import make_axes
from trajectory.messages import *
from trajectory.proto import SyncProto
from . import TestStepper

packet_port = '/dev/cu.usbmodem64213801'  # Production
encoder_port = '/dev/cu.usbmodem63874601'  # Production

baudrate = 115200  # 20_000_000

logging.basicConfig(level=logging.DEBUG)


# Axis configurations for the robot
# Tuples are: step pin, dir pin, enable pin, max_v, max_a

class TestSerial(TestStepper):

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

        p = self.init(packet_port, encoder_port, 100, 'axes1', a=0.1, use_encoder=False)
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

    def test_simple_fix(self):

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

        p.rmove({1: 5000})

        p.runempty(cb, timeout=1)

        p.info()

    # noinspection PyTypeChecker
    def test_simple_r_move(self):

        def cb(p, m):
            if m.name != 'MESSAGE':
                print(m)

        p = self.init(300, 'axes6', a=.5, usteps=10, use_encoder=False,
              outmode=OutMode.OUTPUT_OPENDRAIN, period=5
              )
        p.reset()
        p.run()

        r = p.mspr
        s = p.x_1sec


        x = 40000
        p.rmove({0: x})
        p.rmove({0: -2*x})
        p.rmove({0: x})


        if True:
            p.rmove({1:-16000})
            p.rmove({1: 16000})

            p.rmove({2:16000})
            p.rmove({2: -16000})
            p.rmove({2: -16000})
            p.rmove({2: 16000})

            p.rmove({3:32000})
            p.rmove({3:-32000})
            p.rmove({3: -32000})
            p.rmove({3: 32000})

            p.rmove({4: -10000})
            p.rmove({4: 10000})

            #p.rmove({5: 32000})
            #p.rmove({5: -32000})

        p.runout(cb, timeout=1)

        p.info()

    def test_move_one_axis(self):

        def cb(p, m):
            if m.name != 'MESSAGE':
                print(m)

        p = self.init(packet_port, encoder_port, 1000, 'axes6', a=.5, usteps=10, use_encoder=True,
                      outmode=OutMode.OUTPUT_OPENDRAIN, period=5
                      )
        p.reset()
        p.stop()

        r = p.mspr
        s = p.x_1sec

        x = 50000
        axis = 0
        p.rmove({axis: x})
        p.rmove({axis: -2*x})
        p.rmove({axis: x})
        p.info()
        p.run()
        p.runempty(cb, timeout=1)



if __name__ == '__main__':
    unittest.main()
