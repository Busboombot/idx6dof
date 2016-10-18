"""Motion Planning"""

from math import sqrt

def sign(a): return (a>0) - (a<0)

def seg_velocity_dist(v0, v1, x):
    """Return segment parameters given the initial velocity, final velocity, and distance traveled. """

    v0 = float(v0)
    v1 = float(v1)
    x = float(x)

    if v0 != v1:
        t = abs(2.*x / (v1+v0))
        a = (v1-v0)/t 
    else:
        t = abs(x/v0)
        a = 0

    return abs(x), v0, v1, t, a
    

class JointVector(object):

    def __init__(self,  x1, v1,  x0=None, v0=None, axis=None):
        
        self.axis = axis
        
        
        if v1 is not None:
            self.v1 = float(v1) # Initial velocity
        else:
            self.v1 = None
            
        if x1 is not None:
            self.x1 = float(x1) # Initial position
        else:
            self.x1 = None
        
        if v0 is not None:
            self.v0 = float(v0) # Initial velocity
        else:
            self.v0 = None
            
        if x0 is not None:
            self.x0 = float(x0) # Initial position
        else:
            self.x0 = None
        
    @property
    def distance(self):
        return self.x1 - self.x0
        
    def _t_from_x_a(self, a):
        """Return the time to cover the distance with a given acceleration"""
        
        # The quadratic equation will fail at a=0, 
        # but works for arbitrarily small a
        from math import sqrt
        
        if a < 1/1000000.:
            return abs(self.distance)/self.v0;
        else:
            return (sqrt(self.v0**2 + 2. * a* abs(self.distance))-self.v0)/a

    def amax_split(self, a_max, v_max):
        """Return the minimum time to cover the distance, considering 
        max acceleration and max velocity
        
        Return a tuple of:
            total time
            final velocity
            position at which final velocity is achieved
            remaining distance at final velocity
           
        """
        
        from math import sqrt
        
        # Solve the quadradic equation of motion for t
        
        a_max = float(a_max)
        v_max = float(v_max)
        
        t = self._t_from_x_a(a_max)
        
        v1 = self.v0+a_max*t
        
        if v1 > v_max:
            # Exceeded vmax, so accelerate until v_max, then velocity
            # is constant
            dv = v_max - self.v0
            tp = dv / a_max # t', time when we his v_max
            xp = .5 * a_max * tp**2 + self.v0*tp # x', distance where hiv v_max
            xr = self.distance - xp # remaining distance at const vel
            tr = xr / v_max # remaining time
            
            return (tp+tr, v_max, xp, xr)
            
        else:
            return (t, v1, self.distance, 0.)

    def __repr__(self):
        
        return " {},{}->{},{}".format(self.x0, self.v0, selv.x1, self.v1)
    
class TrajectoryPoint(object):
    
    N_AXES = 6;
    
    def __init__(self, joints=[], t=None, v=None):
        
        self.v = v;
        self.t = t;            
        
        self.joints = [0]*self.N_AXES
        
        
        self.prior = None
            
        for i in range(self.N_AXES):
            try:
                self.joints[i] = joints[i]
            except IndexError:
                pass
                    
        if self.v is None and self.t is None:
            self.v = 0
            self.t = 0
        elif self.v is not None and self.t is not None:
            raise ValueError("Can't specify both t and v")
              
        
    def __getitem__(self, i):
        return self.joints[i]
    
    
    def calc_initial_interval_params(self,v0, a):
        
        v0 = float(v0)
        a = float(a)
        
        if a == 0.:
            
            if v0 != 0:
                cn  = 1000000. / v0
            else:
                cn = 0.
                
            n = 0.
        elif v0 == 0.:
            n = 0.
            cn = 0.676 * sqrt(2.0 / abs(a)) * 1000000.0 * sign(a); # Equation 15
        else:
            n = ((v0 * v0) / (2.0 * abs(a))) # Equation 16
            cn = 1000000. / v0
            
        if sign(a) != sign(v0): # Decelerating
            n = -n
            
        return n, cn
    
    def set_prior(self, prior):
        
        self.prior = prior

        for i in range(self.N_AXES):
            if self.joints[i] is None:
                self.joints[i] = prior.joints[i]
        
        
        if self.v is not None and self.t is None:
            _, _, _, self.t, _  = seg_velocity_dist(prior.v, self.v, self.length)

        elif self.t is not None and self.v is None:
            self.v =  self.length / self.t
                

    @property
    def length(self):
        """ Length of whole vector relative to the prior"""
        from math import sqrt
        
        if not self.prior:
            return 0
        
        return sqrt(sum( (a-b)**2 for a,b in zip(self.joints, self.prior.joints)))
    
    @property
    def lengths(self):
        """ Length of joiunts relative to the prior"""
        from math import sqrt
        
        if not self.prior:
            return 0
        
        return [ a-b for a,b in zip(self.joints, self.prior.joints) ]
    
    def split(self):
        
        pass
        
        
    def info_row(self):
        return [self.t or 0, self.v or 0, self.length]+self.joints
                
    def __repr__(self):
        
        return "{:6.3f} {:6.3f} {:6.3f} {:6d} {:6d} {:6d} {:6d} {:6d} {:6d}".format(*self.info_row())
            
        
class MotionPlanner(object):
    
    def __init__(self, v_max, a_max, d_max):
    
        self.a_max = a_max # Max acceleration
        self.d_max = d_max # Max deceleration
        self.v_max = v_max
        
        self.points = []
        self.velocities = []
        
        self.velocities.append([0]*TrajectoryPoint.N_AXES)
        self.add_point(TrajectoryPoint([0,0,0,0,0,0], t = 0))
        
    
        
    def add_point(self, point):
        """Add a trajectory point"""
        
        from copy import copy
        
        if len(self.points):
            point.set_prior(self.points[-1])
            
            self.velocities.append( [ (tj-pj)/point.t for tj, pj in 
                                    zip(point.joints, self.points[-1].joints) ] )
            
        self.points.append(point)
        
                                 
    @property  
    def split_velocities(self):

        yield  self.velocities[0]
        
        for i in range(1, len(self.velocities)):
            tv = self.velocities[i]
            pv = self.velocities[i-1]
            
            
            # Check for changes in acceleration, which will require a 
            # split. 
            
            splits = []
            
            for j, (tjv, pjv) in enumerate(zip(tv, pv)):
                if tjv != 0 and pjv !=0 and sign(tjv) != sign(pjv):
                    
                    print '!!!', j, tjv, pjv
                    
                    # Calculate where the change in acceleration happens, 
                    # where the velocity is zero. 
                    
                    # Decelerate from v0 to 0
                    t1 = pjv / self.d_max # Decelerate as fast as possible
                    x1 = .5 *self.d_max*(t1**2) + pjv*t1
                    
                    assert x1 < abs(self.points[i].lengths[j]), (x1, self.points[i].lengths[j])
                    assert t1 < self.points[i].t
                    
                    t2 = self.points[i].t - t1
                    x2 = self.points[i][j] - x1
                    
                    splits.append( (j, t1, x1, t2, x2) )
                    
            
            if not splits:
                yield tv
            else:
                print splits
                yield tv
                
    @property
    def accelerations(self):
        
        yield [0]*TrajectoryPoint.N_AXES
        
        for i in range(1, len(self.velocities)):
            tv = self.velocities[i]
            pv = self.velocities[i-1]
        
            yield [ (tjv-pjv)/self.points[i].t for tjv, pjv in zip(tv, pv)]
        
        

    
    
    

    