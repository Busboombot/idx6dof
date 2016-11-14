
from math import sqrt

def sign(a): return (a>0) - (a<0)


class SimSegment(object):
    """A step segment for simulating the step interval algorithm. """
    def __init__(self, v0, v1, x):
        """Return segment parameters given the initial velocity, final velocity, and distance traveled. """

        if sign(v0) and sign(v1) and not ( sign(v0) == sign(v1)):
            raise Exception("The velocity trajectory cannot cross zero.")

        #Each segment must have a consistent, single-valued accceleration
        t = abs(2.* float(x) / float(v1+v0))
        a = (v1-v0)/t 
    
        if a == 0:
            cn  = 1000000. / v0
            n = 0
        elif v0 == 0:
            n = 0 
            cn = 0.676 * sqrt(2.0 / abs(a)) * 1000000.0 * sign(a); # c0 in Equation 15
        else:
            n = ((v0 * v0) / (2.0 * abs(a))) # Equation 16
            cn = 1000000. / v0
            
        # If n is positive, there is a non zero velocity and we are accelerating
        # If it is negative, there is a non zero velocity and we are decelerating 
        if sign(a) != sign(v0): # Decelerating
            n = -n
            
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

        """Call this after each step to compute the delay to the next step"""

        if self.a != 0:
            self.cn = self.cn - ( (2.0 * self.cn) / ((4.0 * self.n) + 1));  # Equation 13 
            
        self.n += 1
        self.xn += 1
        self.tn += abs(self.cn)
        self.vn = 1000000./self.cn
        
        return self.cn
        
        
    def __iter__(self):
        yield self.tn,self.cn, self.n
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
            t =  abs((-v0 + sqrt( v0**2 + 2 * a * x ) ) / a)
        else:
            t = abs(x / v0)

        return t,  v0+a*t
        
    def __repr__(self):
        return  '{:6.0f}->{:6.0f} in {:6.0f} steps, {:6.2f} s , a={:6.2f}'\
                 .format(self.v0, self.vn, self.xn, self.tn/1000000., self.a)

