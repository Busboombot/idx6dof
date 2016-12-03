

DEFAULT_ACCELERATION = 50000

class JointSegment(object):
    """ Description of a trapezoidal velocity profile for a setment of motion on 
    a single joint """
    
    def __init__(self, x=None, v=None, t=None, v0=0, v1=0, a = DEFAULT_ACCELERATION, d = None):
        
        if sum( 1 for var in (x,v,t) if var is not None) != 2:
            raise ValueError("Must specify 2 of x, v or t")
        
        # These three parameters must be related by x = vt
        # at least two of these must be specified
        self.x = float(x) # segment distance
        self.v = float(v) # average velocity
        self.t = float(t) # Total segment time
        
        if self.x is None:
            self.x = self.v * self.t
        elif self.t is None:
            self.t = self.x / self.v
        elif self.t is None:
            self.v = self.x / self.t
        else:
            assert False
        
        self.vrmax = None # Max v achievable for acel from v0 and decel to v1
        self.xmax = None # Max
        self.xmin = None
        
        # Acceleration and deceleration
        self.a = float(a)
        self.d = float(d) if d is not None else float(a)
        
        self.v0 = float(v0) # Velocity at start of segment
        self.v1 = float(v1) # Velocity at end of segment
        
        self.vr = None # Run velocity
        self.ta = None # Acceleration time, from start of seg to end of accelerations
        self.td = None # Decleration time, from start of deceleration to end of segment
        
    def recalc_acccel_times(self):
        """ Calculate ta, td and vr to maintain constant x"""
        
    def calc_tatd(self):
        
        self.ta = self.v-self.v0 / self.a
        self.td = self.v
        
    def calc_vr(self):
        
        self.ta,  self.td, self.vrself._cal_vr(self.x, self.t, self.v0, self.v1)
        
        
    def _calc_vr(self, x,t,v0,v1,a, d):
        """Computue the run velocity, acceleration break time, and deceleration break time
        for other trapezoidial profile parameters. """
        ts = (d*t + v1 - v0) / 2*a

        # Calc vrmax from acceleration from v0 + decel from v1, and average
        vrmax = (v0+v1+a*ts+a*(t-ts)) / 2.

        xmax = .5*ts*(v0+vrmax) + .5 * (t-ts)*(v1+vrmax)

        xmin = .5*v0**2/a + .5*v1**2/d


        # Find Vr with a binary search. There is probably an analytical solution, 
        # but I'm tired of trying to figure it out. The function isn't continuous, so
        # even an analytical solution would probably have some if statements in it.
        def calc_area(vr):
    
            ta = abs((vr-v0)/a)
            td = abs((vr-v1)/d)
    
            x = ( .5*ta*(v0+vr) +
                  .5*td*(v1+vr) +
                  vr*(t-ta-td))
          
            return x, ta, td
    
        import math
    
        low = 0
        high = int(math.ceil(vrmax))


        while high - low > .0001:
    
            vr = (low + high) / 2.0
    
            x_l, _, _ = calc_area(low)
            x_m, ta, td = calc_area(vr)
            x_h, _, _ = calc_area(high)
    
            if x_l <= x < x_m:
                high = vr
            else:
                low = vr


        return round(ta,4), round(td,4), round(vr,4), ts, vrmax, xmin, x, xmax
        
    @staticmethod   
    def pr(ta, td, vr, ts, vrmax, xmin, x,  xmax):
        print "ta={:6.2f} vr={:6.2f} td={:6.2f} xmin={:10.2f} x={:10.2f} xmax={:10.2f} vr={:6.2f} ts={:6.2f}"\
        .format(ta, td, vr, xmin, x, xmax, vrmax, ts)