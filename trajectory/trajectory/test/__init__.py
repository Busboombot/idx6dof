from trajectory.messages import AxisConfig


def rpm_to_usps(rpm, usteps, steps_per_rotation=200):
    """Return the number of microsteps per second at full velocity"""
    rps = rpm/60 # rotations per second
    fsps =  rps*steps_per_rotation # Full steps per second
    usps = fsps*usteps
    return usps


def make_axes(rpm, accel, usteps=1, steps_per_rotation=200):
    "rpm is target RPM. acell is, I think, the time in sec to reach full velocity. "
    s = rpm_to_usps(rpm, usteps)

    mx = (s, s/accel)  # Max Velocity , Max Acceleration

    _axes = {
    'x': (18, 19, 20, *mx),  # X
    'y': (21, 21, 23, *mx),  # Y
    'z': (5, 6, 7, *mx),  # Z
    'a': (2, 3, 4, *mx),  # A
    'b': (8, 9, 10, *mx),  # B
    'c': (15, 16, 17, *mx)  # C
    }

    return {
    "axes1": [AxisConfig(0, *_axes['a'])],
    "axesb": [AxisConfig(0, *_axes['b'])],
    "axesz": [AxisConfig(0, *_axes['z'])],
    "axes2": [AxisConfig(0, *_axes['a']), AxisConfig(1, *_axes['z'])],
    "axes3": [AxisConfig(0, *_axes['a']), AxisConfig(1, *_axes['z']), AxisConfig(2, *_axes['b'])],
    "axes6": [AxisConfig(i, *e) for i, e in enumerate(_axes.values())],
    "x_1sec" : rpm_to_usps(rpm, usteps),
    "mspr": usteps*steps_per_rotation # microsteps per rotation
    }
