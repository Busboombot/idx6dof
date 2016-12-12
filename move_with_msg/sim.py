
from math import sqrt

def sign(a): return (a>0) - (a<0)


class SimSegment(object):
    """A step segment for simulating the step interval algorithm. """
    
    @staticmethod
    def initial_params( v0, a):
        
        # Cn is the number of microseconds between  steps
        # n is acceleration step number 
        
        if a == 0:
            n = 0
            cn  = 1000000. / v0
        elif v0 == 0:
            n = 0 
            cn = 0.676 * sqrt(2.0 / abs(a)) * 1000000.0; # c0 in Equation 15
        else:
            n = int((v0 * v0) / (2.0 * a)) # Equation 16
            cn = 1000000. / v0
        
        #if sign(a) != sign(v0): # Decelerating
        #    n = -n
        
        return n, cn

    @staticmethod
    def next_params(n, cn):
        cn = float(cn)
        cn = cn - ( (2.0 * cn) / ((4.0 * n) + 1));  # Equation 13 
        n += 1
        
        return n, cn
    
    def __init__(self, v0, v1, x):
        """Return segment parameters given the initial velocity, final velocity, and distance traveled. """

        #Each segment must have a consistent, single-valued accceleration
        if abs(sign(v0) + sign(v1)) == 0: # both signs are nonzero and opposite
            raise Exception("The velocity trajectory cannot cross zero. Got {}->{}".format(v0, v1))

        # If n is positive, there is a non zero velocity and we are accelerating
        # If it is negative, there is a non zero velocity and we are decelerating 
      
        self.x = int(round(abs(x),0))
        self.xn = 0 # running position. Done when xn = x
        self.v0 = abs(v0)
        self.v1 = abs(v1) # Velocity at last step
        
        t = abs(2.* float(self.x) / float(self.v1+self.v0))
        a = (self.v1-self.v0)/t 
    
        n, cn = self.initial_params(self.v0, a)
        
        
        self.tn  =  0 # Total transit time
        self.t1 = t # time at last step
        self.vn = abs(v0) # running velocity
        self.a = a
        self.n = n
        self.cn = cn
        
        self.dir = sign(v0 + v1 ) 

        
    def next_delay(self):

        """Call this after each step to compute the delay to the next step"""

        if self.a != 0:
            self.n, self.cn = self.next_params(self.n, self.cn)
        
        self.xn += 1
        self.tn += abs(self.cn)
        self.vn = abs(1000000./self.cn) * self.dir
        
        return self.n, self.cn
        
        
    def __iter__(self):
        while True:
            yield self.tn, self.xn, self.vn, self.cn, self.n
            self.next_delay()
            if self.x == self.xn:
                break
                
    def run_out(self):
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
        return  '{:<10.6} n={:<6d} cn={:<8.4f} xn={:<6.0f} vn={:<6.0f}'\
                 .format(self.tn/1000000., self.n, self.cn, self.xn, self.vn)

