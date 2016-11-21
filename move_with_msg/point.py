
def sign(a): return (a>0) - (a<0)

def sign_change(a, b):
    if sign(a) == 0 or sign(b) == 0:
        return False
    else:
        return sign(a) != sign(b)
        
class TrajectoryPoint(object):
    
    N_AXES = 6;
    
    def __init__(self, t, x=[], v0 = [], v1=[], abs_t0=0, abs_x0=None):
        
        self.t = float(t);    
        self.x = [0.]*self.N_AXES
        self.v0 = [0.]*self.N_AXES
        self.v1 = [0.]*self.N_AXES
        self.a = [0.]*self.N_AXES

        # Absolute time and position at the start of the point. 
        self.abs_t0 = abs_t0
        self.abs_x0 = abs_x0 if abs_x0 is not None else [0.]*self.N_AXES

        for i in range(self.N_AXES):
            try:
                self.x[i] = float(x[i])
            except IndexError:
                pass
                
            try:
                self.v0[i] = float(v0[i])
            except IndexError:
                pass
               
        
          
        self.recalc(v1=v1)
               
    def recalc(self, v1=None, t=None):
        
        t = self.t if t is None else t
        v1 = self.v1 if v1 is None else v1
             
        if t <= 0:
            return
                   
        if v1:
            
            for i in range(self.N_AXES):
                try:
                    self.v1[i] = float(v1[i])
                except IndexError:
                    pass
                    
            self.a = [ (v1-v0)/self.t for v0, v1 in zip(self.v0, self.v1)]
            
        else:
                   
            self.a = [ 2.0*(x-v*self.t)/(self.t**2) for x, v in zip(self.x, self.v0)]
            self.v1 = [ a*self.t + v0 for a, v0 in zip(self.a, self.v0)]
                    
        
    def __getitem__(self, i):
        return self.x[i]
    
    
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
        
    @property
    def length(self):
        """ Length of whole vector relative to the prior"""
        from math import sqrt
        
        return sqrt(sum( j**2 for j in self.moves))
    

    @property
    def velocities(self):
        return [ float(m)/self.t for m in self.moves]
    
    
    def split(self, t):
        """Split a point into two at the given time"""

        # Velocities at time t 
        vp = [ a*t + v0 for a, v0 in zip(self.a, self.v0)]
        xp = [ .5*a*(t**2)+v0*t for a, v0 in zip(self.a, self.v0)]

        # Remaining x
        xr = [x0 - xpp for x0, xpp in zip(self.x, xp)]

        abs_x = [ax+xpp for ax, xpp in zip(self.abs_x0, xp)]

        return (
            TrajectoryPoint(t,        x=xp, v0=self.v0, v1= vp, abs_x0=self.abs_x0, abs_t0=self.abs_t0),
            TrajectoryPoint(self.t-t, x=xr, v0=vp, abs_x0=abs_x,abs_t0=self.abs_t0+t)
            )
            
    def yield_splits(self, decel=None):
        """Split at the lowest time at which the velocity of one of the axes that has a velocity vector that 
        changes direction crosses zero
        """

        def next_split(p, decel):
            split_times = set()

            for i in range(self.N_AXES):
                if sign_change(p.v0[i], p.v1[i]):
                
                    decel = (sign(p.a[i])*abs(decel)) if decel else p.a[i]
            
                    t = -p.v0[i] / decel

                    split_times.add( (t,i, decel) )

                    # Need to force all of the joints that will have a split to the decel, even
                    # if the joint isn't getting split this interation. Otherwize, when multiple joints
                    # get split at the same time, they will get different decels, and then they don't 
                    # all hit 0 velocity at the same time like the should.
                    p.a[i] = decel

                
            return sorted(split_times)[0]

        last = self
        while True:
            
            try:
                t,i,split_decel = next_split(last, decel)
                first, last = last.split(t)
                yield first

            except IndexError:
                yield last
                break
    
    def clone(self):
        return TrajectoryPoint(self.t, self.x, self.v, abs_x0=self.abs_x0, abs_t0=abs_t0)
    
    
    def move(self, t, x=[], v1=[]):
        """ Return a new point, with an initial velocity of this points final velocity, 
        and the given displacement"""
        
        return  TrajectoryPoint(t, x=x, v0=self.v1, v1=v1, abs_x0=self.abs_x0+x, abs_t0=self.abs_t0+t )
        
    def run(self, t, v1=[]):
        """Create a new point with a path from this point to the new velocities in time t"""
        
        return self.move(t, x=[ .5 * (v0i+v1i) * t for v0i, v1i in zip(self.v1, v1) ], v1=v1)
        
    
    
    @staticmethod
    def add(a,b):
        return TrajectoryPoint(a.t+b.t, [ av+bv for av, bv in zip(a.moves, b.moves)])
    
    def __add__(self, other):
        
        return self.add(self, other)
        
    def __iadd__(self, other):
        
        self.t += other.t
        
        self.joints = [ t+o for t, o in zip(self.moves, other.moves)]
        
        return self
        
        
    def __str__(self):
        
        from tabulate import tabulate
        
        rows = [
            ['V0'] + self.v0,
            ['Moves'] + self.x,
            ['V1'] + self.v1,
            ['A'] + self.a
        ]
        
        
        return tabulate( rows, headers = "t={:3.3f} J0 J1 J2 J3 J4 J5".format(self.t).split())
        
        
        