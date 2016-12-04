"""Motion Planning"""

from math import sqrt
from point import TrajectoryPoint

def sign(a): return (a>0) - (a<0)

def sign_change(a, b):
    if sign(a) == 0 or sign(b) == 0:
        return False
    else:
        return sign(a) != sign(b)


    
class MotionPlanner(object):
    
    def __init__(self, v_max, a_max, d_max, v0=None, x0=None):
    
        self.a_max = a_max # Max acceleration
        self.d_max = d_max # Max deceleration
        self.v_max = v_max

        self.current = TrajectoryPoint(0, [0]*TrajectoryPoint.N_AXES)
        
    def update(self, point):
        """Add a trajectory point"""
        
        self.current = point


    def to_next(self, point):
        
        t = point.t
        
        points = [TrajectoryPoint(0, [0]*TrajectoryPoint.N_AXES)]
        
        for i in range(TrajectoryPoint.N_AXES):
            x0 = self.current.x[i]
            x1 = point.x[i]
        
            v0 = self.current.v[i]
        

    @property
    def velocities(self):

        velocities = [m.velocities for m in self.moves]

        v1s = None
        for i in range(0, len(velocities)-1):
            v0s = velocities[i]
            v1s = velocities[i+1]
            
            yield  [ v0 if not sign_change(v0, v1) else 0 for v0, v1 in zip(v0s, v1s)]
            
         
        if v1s:           
            yield v1s
        
        

       



        

    
    
    

    