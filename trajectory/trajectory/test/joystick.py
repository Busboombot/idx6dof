# from trajectory.proto import PacketProto

import logging
from threading import Thread
from time import time, sleep

from trajectory.joystick import PygameJoystick
from trajectory.messages import *
from trajectory.proto import SyncProto

# packet_port = '/dev/cu.usbmodem64213901'
packet_port = '/dev/cu.usbmodem64213801'
baudrate = 3000000  # 20_000_000

logging.basicConfig(level=logging.INFO)

p = SyncProto(packet_port, baudrate, timeout=0.05);

# mx = (15e3, 300e3)
mx = (50e3, 5e6)

axes=[AxisConfig(0, 5, 2, 3, *mx),  # Y Axis
               AxisConfig(0, 6, 7, 4, *mx),  # Z Axis
               AxisConfig(0, 8, 9, 10, *mx),  # B Axis
               AxisConfig(0, 11, 12, 13, *mx)]  # ,

axes=[AxisConfig(0, 2, 3, 4, *mx),  # Y Axis
      AxisConfig(1, 5, 6, 7, *mx),  # Z Axis
      AxisConfig(2, 7, 8, 9, *mx),  # B Axis
      AxisConfig(3,10,11,12, *mx)]  # ,

p.config(itr_delay=4, axes=axes)

# AxisConfig(4, 14,15,16, *mx),
# AxisConfig(5, 17,18,19, *mx)])

print("Starting")

# logging.basicConfig(level=logging.INFO)

p.info()


def sign(x):
    if x == 0:
        return 0
    elif x > 0:
        return 1
    else:
        return -1


def same_sign(a, b):
    return int(a) == 0 or int(b) == 0 or sign(a) == sign(b)


class JoystickValues(Thread):

    def __init__(self):
        super(JoystickValues, self).__init__()
        self.daemon = True

        self.js = PygameJoystick()

        self.last = self.js.last

    def run(self):
        for e in self.js:
            self.last = e

jv = JoystickValues()
jv.start()


t = .15

while True:

    p.read()

    l = [int(v) * t * 5 for v in jv.last.axes[:4]]

    if sum(e*e for e in l):
        print(l, p.queue_length, p.queue_time)
        p.jog(t, l)

    sleep(t-.05)


