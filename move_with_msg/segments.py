
from math import sqrt

DEFAULT_ACCELERATION = 50000

class SegmentError(Exception):
    pass

class SegmentList(object):
    
    def __init__(self, n_joints, v_max, a_max, d_max = None):
        
        self.n_joints = n_joints
        
        self.segments = []
        
        self.initial_velocities = [0] * self.n_joints
        
        self.a_max = a_max
        
        self.d_max = d_max if d_max is not None else self.a_max
        
    def set_initial_velocities(self, velocities):
        assert len(velocities) == self.n_joints
        
        self.initial_velocities = velocities
        
        
    def add_segment(self, joints, t=None, v=None):
        """Add a new segment """
        assert len(joints) == self.n_joints
        
        # Total length of the vector
        vector_x = sqrt(sum( x**2 for x in joints))
        
        if t is None:
            t = vector_x/v
        
        segmentjoints = []
        
        for i,x in enumerate(joints):
            
            if len(self.segments) == 0:
                v0 = self.initial_velocities[i]
            else:
                
                # Since we're getting a new segment, we can allow the previous one to 
                # have a non-zero final velocity
                last_seg_joint = self.segments[-1].joints[i]

                last_seg_joint.v1 = None
                last_seg_joint.td = 0
                last_seg_joint.calc_vr()
              
                v0 = last_seg_joint.v1


            js = JointSegment(x=x, t=t, v0=v0, v1=0, a=self.a_max, d=self.d_max)

            segmentjoints.append(js)
        
        if len(self.segments):
            self.segments[-1].normalize_times()
        
        segment = Segment(segmentjoints, t=t, a=self.a_max, d=self.d_max )
        segment.normalize_times()
        
        self.segments.append(segment)
        
    def normalize_times(self):
        
        for s in self.segments:
            s.normalize_times()
        
    def __iter__(self):
        
        for segment in self.segments:
            for ss in segment:
                yield ss
        
    def __str__(self):
        out = ''
        for s in self.segments:
            out += str(s)
        return out

class Segment(object):
    
    def __init__(self, joints, t, a, d):
        
        self.joints = joints
        self.x  = sqrt(sum( j.x**2 for j in joints))
        self.t = t
       
        self.a = a
        self.d = d
        
        self.ta = None
        self.td = None
        
    
    def normalize_times(self):
        
        max_ta = max(j.ta for j in self.joints)
        max_td = max(j.td for j in self.joints)

        if max_ta + max_td > self.t:
            raise SegmentError("Can't normalize: accl {} + decl {} is greater than total time {}"\
            .format(max_ta, max_td, self.t ))
        
        for j in self.joints:
            if j.ta != max_ta and j.td != max_td and j.x > 0 and ( j.ta > 0 or j.td > 0):
                j.ta = max_ta
                j.td = max_td
                j.calc_vr()
        
        self.ta = max_ta
        self.td = max_td
            
    def __iter__(self):
        
        self.normalize_times()
        
        def calc_x(t, v0, v1):
            x = t * (v0+v1)/2.
        
            return ( (round(x,3)), (round(v0,3)), (round(v1,3)))
        
        if self.ta > 0:
            yield (round(self.ta,3)), [ calc_x(self.ta, j.v0, j.vr ) for j in self.joints ]
            
        tr = self.t-self.ta-self.td
            
        if tr > 0:
            yield tr, [ calc_x(tr, j.vr, j.vr) for j in self.joints ]
        
        if self.td > 0:
            yield (round(self.td,3)), [ calc_x(self.td, j.vr, j.v1)  for j in self.joints ]
        
    
    def set_joint(self, i, js):
        
        self.joints[i] = js
        
    def new_seg(self, x=None, v=None, t=None, v0=0, v1=None):
    
        js = JointSegment(x=x,v=v,t=t,v0=v0, v1=v1, a=self.a, d=self.d )
        
    def __str__(self):
        
        out = "== t={} x={}\n".format(self.t, self.x)
        for js in self.joints:
            out += '   '+str(js)+'\n'
            
        return out

class JointSegment(object):
    """ Description of a trapezoidal velocity profile for a segment of motion on 
    a single joint 
    
    A JointSegment has a constant x. A SubSegment has a constant a.
    
    """
    
    def __init__(self, x=None, v=None, t=None, v0=0, v1=None, a = DEFAULT_ACCELERATION, d = None):
        
        
        # Acceleration and deceleration
        self.a = float(a)
        self.d = float(d) if d is not None else float(a)
        
        self.v0 = float(v0) # Velocity at start of segment
        self.v1 = float(v1) if v1 is not None else None # Velocity at end of segment
        
        # These three parameters must be related by x = vt
        # at least two of these must be specified
        self.x = float(x) if x is not None else None # segment distance
        self.v = float(v) if v is not None else None # average velocity
        self.t = float(t) if t is not None else None # Total segment time
        
        specd_vars = sum( 1 for var in (self.x,self.v,self.t) if var is not None)
        
        if specd_vars == 3:
            raise ValueError("Must specify 2 or fewer of x, v or t")
        elif specd_vars == 1 and self.x is None:
            raise ValueError("If only one of x,v,t is specified, it must be x")
        
        if self.x is None:
            self.x = self.v * self.t
        elif self.t is None:
            if self.v is None:
                self.t = self.x / self.v
            else:
                # Case when only x is specifies
                # Compute the minimum t for x. 
                
                v1 = v1 if v1 else 0
                self.t = (v1-self.v0)/(self.a)
                
        elif self.v is None:
            self.v = self.x / self.t
        else:
            assert False, (x,v,t)
        
        self.vrmax = None # Max v achievable for acel from v0 and decel to v1
        self.xmax = None # Max
        self.xmin = None
        self.ts = None # Time at vrmax
        

        
        self.vr = None # Run velocity
        self.ta = None # Acceleration time, from start of seg to end of accelerations
        self.td = None # Decleration time, from start of deceleration to end of segment
        
        self.calc_vr()

    def calc_vr(self):
        
        self.ta, self.td, self.vr, new_v1, self.ts, self.vrmax, self.xmin, calc_x, self.xmax = \
            self._calc_vr(self.x, self.t, self.v0, self.v1, self.a, self.d)
        
        
        if self.x != 0 and (calc_x - self.x) / self.x > .01:
            diff = (calc_x - self.x)
            r_err =  (calc_x - self.x) / self.x
            raise SegmentError("Calc x is {}, expected {}, diff {}, rel err {}".format(calc_x, self.x, diff, r_err))
            
        assert int(self.xmin) <= int(calc_x) <= int(self.xmax), (self.xmin,calc_x,self.xmax)
        
        self.v1 = new_v1
        
    @staticmethod
    def calc_trap_area(t,vr, v1, v0, ta, td, a, d):
        """Find Vr with a binary search. There is probably an analytical solution, 
           but I'm tired of trying to figure it out. The function isn't continuous, so
           even an analytical solution would probably have some if statements in it."""

        if ta is None:
            ta = abs((vr-v0)/a) # Linear extension from vo at slope a to vr
            
        if td is None: 
            td = abs((vr-v1)/d) # Backward linear extension from v1 at slope d to vr

        x = ( .5*ta*(v0+vr) + # Area under accel trapezoid
              .5*td*(v1+vr) + # Area under decel trapezoid
              vr*(t-ta-td)) # Area under constant v section
      
        return x, ta, td, v1
        
        
    def _calc_vr(self, x,t,v0,v1, a, d):
        """Computue the run velocity, acceleration break time, and deceleration break time
        for other trapezoidial profile parameters. """
        import math
        
        ta_in = self.ta
        td_in = self.td
        v1 = self.v1
        

        if self.v1 is None:

            vrmax = a*t+v0 # Accelerate all the way through the segment
            
            xmax = .5 * a * t * t + v0
            xmin = .5*v0**2/a
            ts = float('nan')
            td_in = 0
            v1 = 0
        else:

            ts = (d*t + v1 - v0) / 2*a # Point where acel line from v0 meets decel line from v1
            
            # Calc vrmax from acceleration from v0 + decel from v1, and average
            vrmax = (v0+v1+a*ts+a*(t-ts)) / 2.
            xmax = .5*ts*(v0+vrmax) + .5 * (t-ts)*(v1+vrmax)
            xmin = .5*v0**2/a + .5*v1**2/d

        low = 0
        high = int(math.ceil(vrmax))

        while high - low > .00001:
    
            vr = (low + high) / 2.0

            x_l, _, _  , v1 = self.calc_trap_area(t, low,  v1, v0, ta_in, td_in, a, d)
            x_m, ta, td, v1 = self.calc_trap_area(t, vr,   v1, v0, ta_in, td_in, a, d)
            #x_h, _, _  , v1 = self.calc_trap_area(t, high, v1, v0, ta_in, td_in, a, d)
    
            if round(x_l,5) <= x <= round(x_m,5):
                high = vr
            else:
                low = vr
                
        if td == 0:
            v1 = vr

        return round(ta,4), round(td,4), round(vr,4), v1, ts, vrmax, xmin, x_m, xmax
 
    def _calc_min_t(self, x,v0,v1, a, d):
        """Computue the run velocity, acceleration break time, and deceleration break time
        for other trapezoidial profile parameters. """
        import math
        
        ta_in = self.ta
        td_in = self.td
        v1 = self.v1
        
        if calc_f is None:
            if self.v1 is None:

                vrmax = a*t+v0 # Accelerate all the way through the segment
                
                xmax = .5 * a * t * t + v0
                xmin = .5*v0**2/a
                ts = float('nan')
                td_in = 0
                v1 = 0
            else:

                ts = (d*t + v1 - v0) / 2*a # Point where acel line from v0 meets decel line from v1
                
                # Calc vrmax from acceleration from v0 + decel from v1, and average
                vrmax = (v0+v1+a*ts+a*(t-ts)) / 2.
                xmax = .5*ts*(v0+vrmax) + .5 * (t-ts)*(v1+vrmax)
                xmin = .5*v0**2/a + .5*v1**2/d
    
        low = 0
        high = int(math.ceil(vrmax))

        while high - low > .00001:
    
            vr = (low + high) / 2.0

            x_l, _, _  , v1 = self.calc_trap_area(t, low,  v1, v0, ta_in, td_in, a, d)
            x_m, ta, td, v1 = self.calc_trap_area(t, vr,   v1, v0, ta_in, td_in, a, d)
            #x_h, _, _  , v1 = self.calc_trap_area(t, high, v1, v0, ta_in, td_in, a, d)
    
            if round(x_l,5) <= x <= round(x_m,5):
                high = vr
            else:
                low = vr
                
        if td == 0:
            v1 = vr

        return round(ta,4), round(td,4), round(vr,4), v1, ts, vrmax, xmin, x_m, xmax
 
    def __str__(self):

        
        return "{} v0={:6.2f} ta={:6.2f} vr={:6.2f} v1={:6.2f} td={:6.2f} xmin={:10.2f} x={:10.2f} xmax={:10.2f} vrmax={:6.2f} ts={:6.2f}"\
        .format(self.t, self.v0, self.ta, self.vr, self.v1, self.td, self.xmin, self.x, self.xmax, self.vrmax, self.ts)