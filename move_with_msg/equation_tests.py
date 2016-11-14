
from math import sqrt
from tabulate import tabulate
from proto import Proto
from stepsegments import SimSegment


def pv(x, v0, v1, t, a):
    
    print '{:6d}->{:6d} in {:6d} steps, {:6.2f} s , a={:6.2f}'.format(v0, v1, x, t, a)
   
def vtx(x, v0, v1, t, a):
    return v0, t, x
    

def compare(v0, v1, x):
    (x, v0, v1, t, a) = Proto.seg_velocity_dist(v0, v1, x)
   
    ss = SimSegment(v0, v1, x).runOut()

    def almost_equal(v1, v2):
        return abs((float(v1)-float(v2)) / v1) < .1

    if (ss.xn!= x or ss.v0 != v0 or not almost_equal(int(ss.vn+v0),int(v1+v0)) or 
        ss.a != a or not almost_equal(ss.tn, t*1000000.) ):
        print 'E:  Expected: {:6d}->{:6d} in {:6d} steps, {:6.2f} s , a={:6.2f}'.format(v0, v1, x, t, a)
        print 'E:  Actual  :',ss
    else:
        print 'OK: Actual  :',ss

    
    compare(600,600,2400)
    compare(-600,-600,2400)  
    
    compare(0,600,2400)
    compare(600,0,2400)
    compare(0,-600,2400)
    compare(-600,0,2400)
    
    compare(599,601,2400)
    compare(601,599,2400)
    compare(-599,-601,2400)
    compare(-601,-599,2400)
    

    

