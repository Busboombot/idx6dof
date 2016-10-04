"""Motion Planning"""

class JointVector(object):

    def __init__(self,  x1, v1, axis = None, x0=None, v0=None):
        
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
        

        
class TrajectoryVector(object):
    
    N_AXES = 6;
    
    def __init__(self,  *joints, t=None, v=None, prior = None):
        
    
        self.v = v;
        self.t = t;
        
        if len(joints) == 1:
            self.joints = joints[0]
        else:
            self.joints = joints
            
    
        self.max_distance = None
        self.longest_axis = None
    
    
                
        if prior:
            self.update_from_prior(p)
                            
    def update_from_prior(self, p):
        
        for ax in self.axes:
            if ax:
                ax.x0 = p[i].x1
                ax.v0 = p[i].v1
            else:
                pass
        
    def __getitem__(self, i):
        return self.joins[i]
    
    def split(self):
        
        pass
        
    def __repr__(self):
        
        out = ''
        for i in 
        
    

class MotionPlanner(object):
    
    
    def __init__(self, v_max, a_max):
    
        self.a_max = a_max
        self.v_max = v_max
        
        self.points = []
        self.add_point(*([None]*TrajectoryVector.N_AXES))
    
        
    def add_point(self, dt=None, v=None, *args):
        """Add a trajectory point"""
        
        from copy import copy
        
        for i, a in enumerate(args):
            if isinstance(a, tuple):
                jv = JointVector(*a, axis=i)
            elif isinstance(a, JointVector):
                jv = copy(a)
                jv.axis = i
            else:
                jv = JointVector(x1=None, v1=None);
            
            
        self.points.append(TrajectoryVector());
            
            
        
        
        
    
    