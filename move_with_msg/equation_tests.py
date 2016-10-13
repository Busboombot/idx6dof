
from math import sqrt
from tabulate import tabulate
from proto import Proto

t = 2.
v0 = 0.
v1 = 1000.
steps = 1000


class StepSegment(object):
    
    def __init__(self, v0, v1, x):
        """Return segment parameters given the initial velocity, final velocity, and distance traveled. """

        if v0 == v1:
            t = abs(float(x)/float(v0))
            a = 0
        else:
            t = abs(2*x / (v1-v0))
            a = (v1-v0)/t 
    
        if a == 0:
            cn  = 1000000. / v0
            n = 0
        elif v0 == 0:
            n = 0 
            cn = 0.676 * sqrt(2.0 / a) * 1000000.0; # Equation 15
        else:
            n = ((v0 * v0) / (2.0 * a)) # Equation 16
            cn = abs(1000000. / v0)
            
        self.x = abs(x)
        self.xn = 0 # running position. Done when xn = x
        self.v0 = v0
        self.v1 = v1 # Velocity at last step
        self.tn  =  0 # Total transit time
        self.t1 = t # time at last step
        self.vn = v0 # time at last step
        self.a = a
        self.n = n
        self.cn = cn

    def next_delay(self):

        if self.a != 0:
            self.cn = self.cn - ((2.0 * self.cn) / ((4.0 * self.n) + 1));  # Equation 13 

        self.n += 1
        self.xn += 1
        self.tn += self.cn
        self.vn = 1000000./self.cn
        
        return self.cn
        
        
    def __iter__(self):
        yield self.tn,self.cn, self.xn
        self.n += 1
        while True:
            self.next_delay()
            yield self.tn,self.cn, self.n
            if self.x == self.xn:
                break
                
    def runOut(self):
        for _ in self:
            pass
        return self

    def segment_time():
        """Compute the time to move X distance, with acceleration a
        and initial velocity v0. Also returns final velocity"""
    
        x = self.x
        v0 = self.v0
        a = self.a
    
        if a != 0:
            t =  (-v0 + sqrt( v0**2 + 2 * a * x ) ) / a
        else:
            t = x / v0

        return t,  v0+a*t
        
    def __repr__(self):
        return  '{:6d}->{:6.0f} in {:6d} steps, {:6.2f} s , a={}'\
                 .format(self.v0, self.vn, self.xn, self.tn/1000000., self.a)



def pv(x, v0, v1, t, a):
    
    print '{:6d}->{:6d} in {:6d} steps, {:6.2f} s , a={:6.2f}'.format(v0, v1, x, t, a)
   
def vtx(x, v0, v1, t, a):
    return v0, t, x
    
def sim(v0, t, x):
    
    a = 2*(x-v0*t)/(t*t)
    
    v1 = v0 + a*t

    print '{}->{} in {} steps, {} s , a={:6.2f}'.format(v0, v1, x, t, a)

    tp, v1p = segment_time(a,v0,x)

    print "calc time: {}, {}% ".format(tp, round((t-tp)/t*100,2))
    for tpp,stepspp, _, _  in yield_delays(a, v0, x): pass
    
    print "sim time : {}, {}%".format(tpp, round((t-tpp)/t*100,2))
    print "\n"
 
    
#sim(0,2,1000)
#sim(1000,2,1000)
#sim(500,2,1000)

pv(*Proto.seg_dist_time(0,1200,4))
pv(*Proto.seg_dist_time(0,-1200,4))
pv(*Proto.seg_dist_time(600,1200,4))
pv(*Proto.seg_dist_time(-600,-1200,4))

pv(*Proto.seg_dist_time(-600,1200,4))
pv(*Proto.seg_dist_time(600,-1200,4))
print '---'
pv(*Proto.seg_velocity_time(0,1200,4))
pv(*Proto.seg_velocity_time(0,-1200,4))
pv(*Proto.seg_velocity_time(1200,0,4))
pv(*Proto.seg_velocity_time(-1200,0,4))
print '---'
pv(*Proto.seg_velocity_time(600,600,4))
pv(*Proto.seg_velocity_time(-600,-600,4))
pv(*Proto.seg_velocity_time(600,-600,4))
pv(*Proto.seg_velocity_time(-600,600,4))
print '---'
pv(*Proto.seg_velocity_dist(600,600,2400))
pv(*Proto.seg_velocity_dist(-600,-600,2400))
pv(*Proto.seg_velocity_dist(600,-600,2400))
pv(*Proto.seg_velocity_dist(-600,600,2400))
print '---'
pv(*Proto.seg_velocity_dist(0,600,2400))
print StepSegment(0,600,2400).runOut()
pv(*Proto.seg_velocity_dist(600,0,2400))
print StepSegment(600,0,2400).runOut()



