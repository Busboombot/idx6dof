
from trajectory.messages import AxisConfig, OutVal, OutMode
from trajectory.proto import SyncProto


def rpm_to_usps(rpm, usteps, steps_per_rotation=200):
    """Return the number of microsteps per second at full velocity"""
    rps = rpm / 60  # rotations per second
    fsps = rps * steps_per_rotation  # Full steps per second
    usps = fsps * usteps
    return usps


def make_axes(rpm, accel, usteps=1, steps_per_rotation=200,
              output_mode=OutMode.OUTPUT, highval=OutVal.HIGH):
    "rpm is target RPM. acell is, I think, the time in sec to reach full velocity. "
    s = rpm_to_usps(rpm, usteps)

    mx = ( highval, output_mode, s, s / accel)  # Max Velocity , Max Acceleration

    _axes = {
        'x': (18, 19, 20, *mx),  # X Waist, Axis 0
        'y': (21, 22, 23, *mx),  # Y Shoulder 1, Axis 1
        'z': (5, 6, 7, *mx),     # Z Shoulder 2, Axis 2
        'a': (15, 16, 17, *mx),  # A Elbow, Axis 3
        'b': (8, 9, 10, *mx),    # B Wrist 1, Axis 4
        'c': (2, 3, 4, *mx),     # C Wrist 2, Axis 5
    }


    return {
        "axes1": [AxisConfig(0, *_axes['a'])],
        "axesb": [AxisConfig(0, *_axes['b'])],
        "axesz": [AxisConfig(0, *_axes['z'])],
        "axes2": [AxisConfig(0, *_axes['a']), AxisConfig(1, *_axes['z'])],
        "axes3": [AxisConfig(0, *_axes['a']), AxisConfig(1, *_axes['z']), AxisConfig(2, *_axes['b'])],
        "axes6": [AxisConfig(i, *e) for i, e in enumerate(_axes.values())],
        "x_1sec": rpm_to_usps(rpm, usteps),
        "mspr": usteps * steps_per_rotation,  # microsteps per rotation
        "axes": _axes
    }

import unittest

class TestStepper(unittest.TestCase):
    # Determines wether the steppers are enables with an output value of high or low
    # Different for different stepper drivers
    ENABLE_OUTPUT = False

    def setUp(self) -> None:
        pass

    def tearDown(self) -> None:
        pass

    def init(self, packet_port, encoder_port, v=800, axes_name='axes1', usteps=16, a=.1,
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
