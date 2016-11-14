"""Motion Planning"""

from math import sqrt
from point import TrajectoryPoint

def sign(a): return (a>0) - (a<0)

def sign_change(a, b):
    if sign(a) == 0 or sign(b) == 0:
        return False
    else:
        return sign(a) != sign(b)

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
        
        
    @property
    def accelerations(self):
        
        velocities = [self.v0]+list(self.velocities)

        for i in range(0, len(velocities)-1):
            tv = velocities[i]
            nv = velocities[i+1]
        
            yield [ (njv-tjv)/self.moves[i].t for tjv, njv in zip(tv, nv)]

    def split_velocities(self):

        orig_velocities = list(self.velocities)
        orig_points = list(self.points)
        accelerations = list(self.accelerations)

        def find_split(i, j, tjv, pjv):
            """Find the point of a split in an axis"""
            # If this velocity is not zero, and the previous velocity wasn't,
            # and the two velocities have opposite signs, we will need a split.
            if tjv != 0 and pjv !=0 and sign(tjv) != sign(pjv):
     
                # Calculate where the change in acceleration happens, 
                # where the velocity is zero. 
            
                # Decelerate from v0 to 0
                t1 = pjv / self.d_max # Decelerate as fast as possible in time t1
                x1 = .5 *self.d_max*(t1**2) + pjv*t1 + orig_points[i-1][j] # Distance covered in decel
            
                #assert x1 < abs(orig_points[i].lengths[j]), (x1, orig_points[i].lengths[j])
                #assert t1 < orig_points[i].t
            
                t2 = orig_points[i].t - t1 # Remaining time after the split
                x2 = orig_points[i][j] - x1 # Remaining distance to cover
            
                return (j, t1, x1, t2, x2)
            

            
        def do_splits():

            self.points = [self.points[0]]
            self.velocities = [self.velocities[0]]

            for i in range(1, len(orig_velocities)):
                
                tv = orig_velocities[i] # This velocity
                pv = orig_velocities[i-1] # Previous velocity
                
                # Check for changes in acceleration, which will require a 
                # split. 
        
                splits = None
        
                for j, (tjv, pjv) in enumerate(zip(tv, pv)):
                    splits = find_split(i, j, tjv, pjv)
                    
                    if splits:
                        print '!!!!', pjv, tjv, splits
                        break
                        
                        
                if splits:
                    
                    p = [0]*TrajectoryPoint.N_AXES
                    (split_j, t1, x1, t2, x2) = splits
        

                    for j in range(TrajectoryPoint.N_AXES):
                        if j == split_j:
                            p[j] = x1
                        else:
                            a = accelerations[i][j]
                            v0 = pv[j]
                            p[j] = .5*a*(t1**2)+v0

                    
                    self.add_point(TrajectoryPoint(p, t=t1))


                    self.add_point(TrajectoryPoint(orig_points[i].joints, t=t2))

                    
                else:
                    self.add_point(orig_points[i])

        do_splits()
       



        

    
    
    

    