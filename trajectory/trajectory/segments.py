#
"""
Create a list of movements for a set of axes, then calculate
an optimum trapezoidal velocity profile.

Each movement is a segment, a period of time in which all axes
must start and stop at the same time. The segments have three periods:

    a: Acceleration
    c: Constant velocity
    d: Deceleration

( the names for the A and D periods refer to an idealized trapezoidal profile,
where a is the first part of the profile and d is the last. in actual cases, a
may have a deceleration, and d an acceleration. The c period is always 0
acceleration )

The optimization procedure attempts to execute every segment in the minimum
time, by ensuring that the longest axis is accelerate to the max velocity
as the max acceleration, and that there is a minimal change in velocity between
adjacent segments.

The algorithms for SimSegment and the c and cn parameters are based on
the article, 'Generate stepper-motor speed profiles in real time',
from https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/

"""
from copy import deepcopy
from itertools import count
from math import sqrt
from typing import List
from collections import deque

## Parameters for simulation/step generation

# NUmber of ticks of the step function per second
TIMEBASE = 1_000_000  # ticks per second

# Shapes
TRAPEZOID = 1
TRIANGLE = 2


def binary_search(f, v_min, v_guess, v_max):

    for i in range(20):

        x = f(v_guess)

        if round(x) > 0:
            v_guess, v_min = (v_max + v_guess) / 2, v_guess

        elif round(x) < 0:
            v_guess, v_max = (v_min + v_guess) / 2, v_guess

        else:
            return v_guess

        if abs(v_min-v_max) <1:
            return v_guess

    else:
        return None

def sign(x):
    if x == 0:
        return 0
    elif x > 0:
        return 1
    else:
        return -1


def same_sign(a, b):
    return int(a) == 0 or int(b) == 0 or sign(a) == sign(b)


class SegmentError(Exception):
    pass


class ConvergenceError(SegmentError):
    pass


class ConstraintError(SegmentError):
    pass

class ValidationError(SegmentError):
    pass



class Joint(object):
    """Represents a single joint, with a maximum velocity and acceleration"""

    def __init__(self, v_max: float = None, a_max: float = None, d_max: float = None):
        self.n = None
        self.v_max = v_max
        self.a_max = a_max
        self.d_max = d_max if d_max is not None else self.a_max


class SubSegment(object):
    """A sub segment is a portion of the trajectory with a constant
    acceleration,  one of the aceleration, constant (cruise) or decleration portions. """

    def __init__(self, t: float, v_i: float, v_f: float, x: float, ss: float) -> None:
        self.t = t  # Total ssubsegment time
        self.v_i = v_i  # Initial velocity
        self.v_f = v_f  # final velocity
        self.x = x  # Segment distance
        self.ss = ss  # Section, 'a'->accleration, 'c'-> constant, 'd'->deceleration
        self.direction = 1  # 1 for forward, -1 for reverse

        assert v_i == 0 or v_f == 0 or sign(v_i) == sign(v_f), f"Inconsistent directions {v_i} {v_f}"

        # Step function parameters.
        self.sim = SimSegment(self.v_i, self.v_f, self.t)
        self.n, self.ca, self.cn = self.sim.initial_params()

    def set_direction(self, sign) -> None:

        self.direction = sign
        self.v_i *= sign
        self.v_f *= sign
        self.x *= sign

    def __repr__(self):
        dir = '' if self.direction > 0 else '-'
        n = str(int(self.n)) if self.n < N_BIG else 'C'
        if self.cn < 1:
            cn = f"{self.cn:0.5f}"
        else:
            cn = f"{self.cn:8.0f}"

        return f"[{self.ss} {self.t:2.5f} {int(self.x):5d} {self.v_i:5.0f}->{self.v_f:5.0f}  ({n:5s}, {cn})] "


class JointSegment(object):
    """One segment for one joint"""

    x: float
    x_err: float  # Extra step value to add, somewhere

    x_a: float = 0
    x_c: float = 0
    x_d: float = 0

    v_0: float = 0
    v_0_max: float
    v_c: float = 0
    v_1: float = 0
    v_1_max: float  # Don't allow updating v_1

    shape: int = TRAPEZOID

    joint: Joint
    next_js: 'JointSegment'
    prior_js: 'JointSegment'

    def __init__(self, joint: Joint = None, x: float = None, v0: float = 0):

        self.joint = joint
        self.segment = None  # set after adding to a segment

        self.x = abs(x)
        self.s = sign(x)

        self.v_0 = v0
        self.v_c = self.joint.v_max
        self.v_1 = 0

        self.v_0_max = self.joint.v_max
        self.v_1_max = self.joint.v_max

        self.next_js = None
        self.prior_js = None

        self.x_err = 0


    @property
    def id(self):
        return f"{self.segment.seg_number}/{self.joint.n}"

    def update_sub_segments(self):

        self.x_a = (self.v_0 + self.v_c) * self.segment.t_a / 2.
        self.x_d = (self.v_1 + self.v_c) * self.segment.t_d / 2.
        self.x_c = self.segment.t_c * self.v_c + self.x_err

        return self.x_a + self.x_c + self.x_d + self.x_err

    def calc_x(self, v_c):
        """Calculate the distance of the segment for a given v_c, and all
        other parameters fixed. """
        x_a = (self.v_0 + v_c) * self.segment.t_a / 2.
        x_d = (self.v_1 + v_c) * self.segment.t_d / 2.
        x_c = self.segment.t_c * v_c

        return x_a + x_c + x_d

    def search_v_c(self):
        """Binary search for v_c"""

        f = lambda v_c: self.x - self.calc_x(v_c)

        v_mean = self.x/self.segment.t

        return binary_search(f,0,v_mean,self.joint.v_max )

    def update_t_min(self):
        '''minimum time to run the segment'''

        def accel_t_for_x(x, v_0, a_max):
            """Time to cover distance x, at max acceleration
            x = 1/2 at^2 + v_0 t
             -> 1/2 at^2 + v_0 t - x = 0
            Yeah for 8th grade algebra!
            """

            a = .5 * a_max
            b = v_0
            c = -x

            term_1 = -b
            term_2 = sqrt((b ** 2) - 4 * a * c)

            root_1 = (term_1 - term_2) / (2 * a)
            root_2 = (term_1 + term_2) / (2 * a)

            return max(root_1, root_2)

        v_c = self.v_c

        # These are over estimates of the t_a and t_d,
        # the max that the would need to be in any case.

        # Time to accelerate to max speed from initial speed
        t_a = (abs(v_c - self.v_0)) / self.joint.a_max
        # Time to decel from max speed to final speed.
        t_d = (abs(v_c - self.v_1)) / self.joint.d_max

        # Distances for the accel and decel phases
        x_a = int(round((self.v_0 + v_c) * t_a / 2., 0))
        x_d = int(round((self.v_1 + v_c) * t_d / 2., 0))

        # Not enough distance to accel to max speed -> triangle profile
        if x_a + x_d > self.x:
            x_a = x_d = self.x / 2 # assumes a_max and d_max are equal!
            assert self.joint.a_max == self.joint.d_max
            t_a = accel_t_for_x(x_a, self.v_0, self.joint.a_max)
            t_d = accel_t_for_x(x_d, self.v_1, self.joint.d_max)

            self.shape = TRIANGLE

        x_c = self.x - x_a - x_d

        try:
            t_c = x_c / v_c
        except ZeroDivisionError:
            t_c = 0

        self.t_min =   t_a + t_c + t_d
        self.min_t_a = t_a
        self.min_t_c = t_c
        self.min_t_d = t_d

    def update_v_c(self):
        """Find a new v_c that satisfies the constraints on t_a, t_c, t_d from
        the segment"""

        self.x_err = 0

        if self.x == 0:
            self.v_c = 0
            self.update_sub_segments()
            return 0

        v_c = self.search_v_c()

        if v_c is None:
            raise ConvergenceError(
                f"Velocity changes failed to converge x: \n"
                f"{str(self)}\n{self.debug_str()}\n" )

        self.x_err = self.x - self.calc_x(v_c)
        self.v_c = v_c
        self.update_sub_segments()

        if round(self.x_a + self.x_d) > self.x:
            raise ConstraintError(f"Acceleration periods are longer than segment"
                                   f" {self.x_a}+{self.x_d}>{self.x} v_c={self.v_c}")

        # This should not be able to happen.
        if round(self.v_c, 0) > self.joint.v_max:
            raise ConstraintError(f"V_C is too big {round(self.v_c, 0)} > {self.joint.v_max}" )

        return self.x_err



    def update_start_velocity_limit(self, is_first: bool, sign_change: bool):
        """Update entry velocity"""

        # Force zero for first segment. Second term is for triangle profile
        if is_first or sign_change or self.x == 0 or self.x_a + self.x_d >= self.x:

            self.v_0_max = 0
        else:
            # Limit to either max joint velocity, or the small-x case.
            # The second term, x/t_a, is for when x is small, and v_c will be driven down to 0,
            # so all movement is in the accel and decel phase. x/t_a is
            #  reduced from x=v_mean*t_a and v_mean  = 1/2 * (v_0 + v_c) for v_c = 0, etc.
            self.v_0_max = min(self.joint.v_max, self.x / self.segment.t_a)


    def update_end_velocity_limit(self, is_last: bool):
        """Update the exit velocity"""
        if is_last or self.x == 0 or self.x_a + self.x_d >= self.x:
            self.v_1_max = 0
        else:
            # See comment in update_start_velocity()
            self.v_1_max = min(self.joint.v_max, self.x / self.segment.t_d)

    def update_boundary_velocity(self, prior_js:'JointSegment' = None, next_js:'JointSegment' = None):

        # prior_js and next_js signals wether this function is called on the
        # last segment, or the next to last.
        assert prior_js is None or next_js is None


        if True:
            v = lambda o : o.v_c
        else:
            v = lambda o: o.mean_v()


        if prior_js:
            mean_bv = ( v(prior_js) + v(self) ) / 2.
            self.v_0 =  min(mean_bv, self.v_0_max,    prior_js.v_1_max)

        elif next_js:
            mean_bv = ( v(self) + v(next_js)) / 2.
            self.v_1 = min(mean_bv, next_js.v_0_max, self.v_1_max)
        else:
            assert False

    def mean_v(self):
        return self.x/self.segment.t


    def validate_accel_periods(self, exc=True):

        t = (round(self.x_a + self.x_d) <= self.x)

        if not t and exc:
            raise ValidationError(f"A/D Periods longer than segment {self.x_a + self.x_d}>{self.x}: {str(self)} ")

        return t

    def validate_velocity_limits(self, exc=True):
        t = (round(self.v_0, 0) <= self.joint.v_max) and \
            (round(self.v_c, 0) <= self.joint.v_max) and \
            (round(self.v_1, 0) <= self.joint.v_max)

        if not t and exc:
            raise ValidationError(f"Velocity limits error {self.v_0},{self.v_c},{self.v_1} v_max={self.joint.v_max} ")

        return t

    def validate_distance(self, exc=True):
        """Ensure that the velocity parameters result in traveling the given distance."""

        _x = self.x_a + self.x_c + self.x_d;

        t = (round(self.x, 0) == round(_x, 0))

        if not t and exc:
            raise ValidationError(f"Distance error {round(self.x, 3)} != {round(_x, 3)} x_err={self.x_err}")

        return t

    def validate_velocity_signs(self, exc=True):
        t = same_sign(self.v_0, self.v_c) and same_sign(self.v_0, self.v_1)

        if not t and exc:
            raise ValidationError('Velocity signs error {self.v_0},{self.v_c},{self.v_1}')

        return t

    def validate_distance_signs(self, exc=True):
        t = same_sign(self.x_a, self.x_c) and same_sign(self.x_a, self.x_d)

        if not t and exc:
            raise ValidationError(f"Distance signs error {self.x_a},{self.x_c},{self.x_d} x_err={self.x_err}")

        return t

    def validate(self, exc=True):
        return self.validate_velocity_limits(exc) and \
               self.validate_distance(exc) and \
               self.validate_velocity_signs(exc) and \
               self.validate_distance_signs(exc)

    def _sub_segments(self):
        if round(self.segment.t_a, 7) > 0:
            yield SubSegment(round(self.segment.t_a, 7), round(self.v_0, 2), round(self.v_c, 2), round(self.x_a, 0),
                             'a')

        if round(self.segment.t_c, 7) > 0:
            yield SubSegment(round(self.segment.t_c, 7), round(self.v_c, 2), round(self.v_c, 2), round(self.x_c, 0),
                             'c')

        if round(self.segment.t_d, 7) > 0:
            yield SubSegment(round(self.segment.t_d, 7), round(self.v_c, 2), round(self.v_1, 2), round(self.x_d, 0),
                             'd')

    @property
    def sub_segments(self):
        """Apply the direction to parts of the subsegment"""
        for ss in self._sub_segments():
            ss.set_direction(self.s)

            yield ss

    def __str__(self):
        from colors import color, bold

        a = f"{self.v_0:<6.0f}"
        xa = f"{self.s * self.x_a:^5.0f}"
        c = f"{int(round(self.s * self.x_c)):6d}"
        vc = f"{self.v_c:<6.0f}"
        xd = f"{self.s * self.x_d:^5.0f}"
        d = f"{self.v_1:>6.0f}"

        return f"[{color(str(a), fg='green')} {bold(xa)} : {bold(c)}@{color(str(vc), fg='blue')} : {bold(xd)} {color(str(d), fg='red')}]"

    def debug_str(self):
        from colors import color

        a = f"{self.v_0_max:<6.0f}"
        ta = f"{round(self.min_t_a * 1000, 1):^5.1f}"
        c = f"{round(self.min_t_c * 1000, 3):^13.3f}"
        td = f"{round(self.min_t_d * 1000, 1):^5.1f}"
        d = f"{self.v_1_max:>6.0f}"

        return f"[{color(str(a), bg='green')} {ta} : {c} : {td} {color(str(d), bg='red')}]"


class Segment(object):
    """One segment, for all joints"""

    def __init__(self, prior:'Segment', n: int, joint_segments: List[JointSegment]):
        self.joint_segments = joint_segments

        self.seg_number = n

        for js in self.joint_segments:
            js.segment = self

        self.t_a = 0
        self.t_c = 0
        self.t_d = 0

        if prior:
            self.set_prior_js(prior)
            prior.set_next_js(self)

            self.sign_change = any(not same_sign(a.s, b.s) for a, b
                              in zip(prior.joint_segments, prior.joint_segments))

            prior.update_second_to_last()

        else:
            self.sign_change = False


        self.update_last(prior)


    @property
    def t(self):
        """Total run time for the segment. Only valid after an update. """
        return self.t_a + self.t_c + self.t_d

    def set_next_js(self, next_seg):
        """Set the next_js for each joint segment"""
        for j, next_js in zip(self.joint_segments, next_seg.joint_segments):
            j.next_js = next_js

    def set_prior_js(self, prior_seg):
        """Set the prior_js for each joint segment"""
        for j, prior_js in zip(self.joint_segments, prior_seg.joint_segments):
            j.prior_js = prior_js

    @property
    def joint_segments_sorted(self):
        """Joint segments sorted by time required to execute with current parameters """
        return [e[1] for e in reversed(sorted([(j.t_min, j) for j in self.joint_segments], key=lambda x: x[0]))]

    def update_last(self, prior:'Segment',):
        """Updates for when this is the last ( most recent) in the queue"""

        is_first = prior is None

        for js in self.joint_segments:
            js.update_t_min()

        longest = self.joint_segments_sorted[0]
        self.t_a = longest.min_t_a
        self.t_c = longest.min_t_c
        self.t_d = longest.min_t_d

        for js in self.joint_segments:
            js.update_sub_segments()
            js.update_end_velocity_limit(is_last=True)
            js.update_start_velocity_limit(is_first, self.sign_change)

        if prior:
            for prior_js, next_js in zip(prior.joint_segments,self.joint_segments):
                next_js.update_boundary_velocity(prior_js, None)
                prior_js.update_boundary_velocity(None, next_js)

        for js in self.joint_segments:
            js.update_v_c()

    def update_second_to_last(self):

        for js in self.joint_segments:
            js.update_sub_segments()
            js.update_end_velocity_limit(is_last=False)

        for js in self.joint_segments:
            js.update_v_c()


    def validate(self):
        for i, js in enumerate(self.joint_segments):

            try:
                js.validate_accel_periods()
                js.validate_velocity_limits()
                js.validate_distance()
                js.validate_velocity_signs()
                js.validate_distance_signs()
            except:
                ref = f"Error in Seg {self.seg_number} Joint {js.joint.n} \n  {str(self)}"
                print(ref)
                raise

    @property
    def sub_segments(self):

        ss = [js.sub_segments for js in self.joint_segments]

        return zip(*ss)

    def __str__(self):

        return f"|{self.t_a:1.4f} {self.t_c:1.4f} {self.t_d:1.4f}|" + ' '.join(str(js) for js in self.joint_segments)

    def debug_str(self):

        return str(self) + '\n' + f"{' ':22s}" + ' '.join(js.debug_str() for js in self.joint_segments)


class SegmentList(object):
    positions: List[float]


    def __init__(self, joints: List[Joint]):

        self.joints = deepcopy(joints)

        self.directions = [0] * len(self.joints)

        for i, j in enumerate(self.joints):
            j.n = i

        self.segments = deque()
        self.positions = [0] * len(self.joints)

        self._sub_segments = []

    def add_distance_segment(self, joint_distances: List[int], v0: List[float] = None):
        """Add a new segment, with joints expressing joint distance """

        assert len(joint_distances) == len(self.joints)

        if len(self.segments) > 0 and v0 is not None:
            raise SegmentError("Can only set initial velocity on first segment")

        # Update directions that may be zero

        self.positions = [x0 + x1 for x0, x1 in zip(self.positions, joint_distances)]

        if v0 is None:
            v0 = [0] * len(self.joints)

        if len(self.segments):
            prior_seg = self.segments[-1] # Soon to be the second to last.
        else:
            prior_seg = None

        next_seg = Segment(prior_seg, len(self.segments),
                           [JointSegment(j, x=x, v0=v) for j, x, v in zip(self.joints, joint_distances, v0)])

        self.segments.append(next_seg)

        # Move finished segments out to the subsegment queue
        if len(self.segments) > 2:
            s = self.segments.popleft()
            for ss in s.sub_segments:
                self._sub_segments.append(ss)

        return next_seg


    def add_position_segment(self, positions: List[float], v0: List[float] = None):

        distances = [x1 - x0 for x0, x1 in zip(self.positions, positions)]

        return self.add_distance_segment(distances, v0)

    def validate(self):
        for s in self.segments:
            s.validate()

    @property
    def sub_segments(self):
        """Generate all of the subsegments from each of the segments"""

        for ss in self._sub_segments:
            yield ss

        for s in self.segments:
            yield from s.sub_segments

    @property
    def dataframe(self):
        import pandas as pd

        rows = []

        tc = 0
        for e in self.sub_segments:
            seg_header = []
            for i, ss in enumerate(e):
                rows.append([i, ss.x, ss.v_i, ss.v_f, ss.ss, ss.t, ss.n, ss.cn])

        df = pd.DataFrame(rows, columns='axis x v_i v_f ss del_t n cn'.split())

        df['t'] = df.groupby('axis').del_t.cumsum()

        return df

    def __iter__(self):
        return iter(self.segments)

    def __len__(self):
        return len(self.segments)

    def __str__(self):
        return '\n'.join(str(s) for s in self.segments)

    def debug_str(self):
        return '\n'.join(s.debug_str() for s in self.segments)


from collections import namedtuple

#
StepParams = namedtuple('StepParams', 'n tn  cn ca1 xn vn dir tf x')


# For IDX robot, v has a range of +/- 15K
# a has range of about -50K to 50K

class SimSegment(object):
    """A step segment for simulating the step interval algorithm. """

    fp_bits = 11  # Number of bits for the fixed point fraction

    def __init__(self, v0, v1, x=None, t=None, t0=0, timebase=None):
        """Return segment parameters given the initial velocity, final velocity, and distance traveled. """

        # Each segment must have a consistent, single-valued accceleration
        if abs(sign(v0) + sign(v1)) == 0 and (abs(v0) + abs(v1)) > 0:  # both signs are nonzero and opposite
            raise Exception("The velocity trajectory cannot cross zero. Got {}->{}".format(v0, v1))

        assert t != 0.0

        self.dir = sign(v0 + v1)

        self.v0 = abs(v0)
        self.v1 = abs(v1)  # Velocity at last step

        if x is None:
            self.x = int(round(.5 * (v1 + v0) * t, 0))
            self.t = t

        elif t is None:
            self.x = abs(x)
            if self.v1 + self.v0 == 0:
                self.t = 0
                self.a = 0
            else:
                self.t = abs(2. * float(self.x) / float(self.v1 + self.v0))
                self.a = (self.v1 - self.v0) / self.t

        self.x = int(round(self.x, 0));

        if x == 0 or t == 0:
            self.a = 0
            self.tf = 0  # time at last step
            self.tn = 0  # Total transit time for step n
            self.vn = 0  # running velocity
            self.xn = 0  # running position. Done when xn = x
            self.v1 = 0
            self.v0 = 0
        else:
            self.tf = t  # time at last step
            self.tn = t0  # Total transit time for step n
            self.vn = abs(v0) * self.dir  # running velocity
            self.xn = 0  # running position. Done when xn = x

        self.timebase = timebase if timebase is not None else TIMEBASE

        self.constV = False

        self.sim_time = 0;  # Time to completion in simulation iteration/

    def initial_params(self):
        """Set initial parameters, which are a bit different for the first step,
        to account for low torque. """

        # If n is positive, there is a non zero velocity and we are accelerating
        # If it is negative, there is a non zero velocity and we are decelerating

        v0 = float(self.v0)
        v1 = float(self.v1)

        if v0 == 0 and v1 == 0:
            # Going nowhere.
            n = 0
            cn = 0
            self.constV = True

        elif v0 == 0:
            # Starting from a stop, so need special
            # first delay.
            a = abs(v1) / self.t

            n = 0  # n will always be positive, so accelerating
            cn = 0.676 * sqrt(2.0 / a) * self.timebase  # c0 in Equation 15

            assert cn > 0

        elif v0 == v1:
            # Constant velocity.
            n = 0
            cn = self.timebase / abs(v0)
            self.constV = True

        else:
            # Normal case, between two non-zero velocities
            a = abs(v1 - v0) / self.t

            n = (v0 * v0) / (2.0 * a)  # Equation 16
            cn = self.timebase / abs(v0)

            assert n >= 0

            # Need to put the sign back on n; n must be negative for deceleration
            if abs(v1) < abs(v0):
                n = -n

        # Fixed point version of cn
        ca = int(cn * (1 << SimSegment.fp_bits))
        n = round(n)
        self.ca1 = 0

        return n, cn, ca

    @staticmethod
    def next_params(n, cn):

        n += 1

        ca1 = (2.0 * cn) / ((4.0 * n) + 1)

        cn = cn - ca1  # Equation 13

        return n, cn, ca1

    @staticmethod
    def next_params_xp(n, ca):
        """Fixed point version of the next_params function"""

        n += 1

        if n != 0:
            ca1 = (ca << 1) / ((n << 2) + 1);

            ca = ca - ca1,  # Equation 13

        return n, ca, ca1

    def next_delay(self):
        """Call this after each step to compute the delay to the next step"""

        self.n, self.cn, self.ca1 = self.next_params(self.n, self.cn)

        self.xn += 1
        self.tn += abs(self.cn)
        try:
            self.vn = abs(TIMEBASE / self.cn) * self.dir
        except ZeroDivisionError:
            self.vn = 0

        return self.n, self.cn, self.ca1

    row_header = 'n tn cn xn vn dir tf x'.split()

    def __iter__(self):

        self.n, self.cn, self.ca = self.initial_params()

        while True:
            yield StepParams(self.n, round(self.tn / TIMEBASE, 6), round(self.cn, 2), round(self.ca1, 2), self.xn,
                             int(round(self.vn, 0)), self.dir, self.tf, self.x)
            self.next_delay()
            if self.x - self.xn <= 0:
                break

    def iter_discrete(self, period):
        """Iterate discretely, in units of time of the period"""

        n, cn, ca = self.initial_params()

        t = 0
        stepsLeft = self.x
        lastTime = 0

        self.sim_time = 0

        for i in count():

            lastTime = lastTime + (period << SimSegment.fp_bits);

            self.sim_time += period / self.timebase

            if lastTime > ca:
                lastTime -= ca
                if not self.constV:
                    n += 1;
                    ca1 = int(((2 * ca) / ((4 * n) + 1)))
                    ca = abs(ca - ca1)

                stepsLeft -= 1;
                yield 1
            else:
                yield 0

            if stepsLeft == 0:
                return i

    def iter_time(self, period):

        t = 0
        steps_left = self.x

        delay_counter = 0
        v = self.v0
        delay = 0

        delay_inc = period / self.timebase

        for i in count():

            if delay_counter >= delay:
                delay_counter -= delay

                steps_left -= 1
                yield (i, round(t, 6) * self.timebase, steps_left, v, delay_counter, delay)

            v = self.a * t + self.v0
            delay = 1 / v if v else 0

            if steps_left == 0:
                break;

            delay_counter += delay_inc
            t += delay_inc

        self.sim_time = t

    @property
    def dataframe(self):
        import pandas as pd

        def _y():
            for e in self:
                yield self.n, self.tn / TIMEBASE, self.cn, self.xn, self.vn, self.dir, self.tf, self.x

        return pd.DataFrame(list(_y()), columns='n tn cn xn vn dir tf x'.split())

    # def __repr__(self):
    #    return '{:<10.6} t={:<8.4f} x={:<8.0f} n={:<6d} cn={:<8.4f} xn={:<6.0f} vn={:<6.0f}' \
    #        .format(self.tn / TIMEBASE, self.tf, self.x, self.n, self.cn, self.xn, self.vn)
