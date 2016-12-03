import numpy as np


def calc(x,t,v0,v1,a, d):
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
        
def pr(ta, td, vr, ts, vrmax, xmin, x,  xmax):
    print "ta={:6.2f} vr={:6.2f} td={:6.2f} xmin={:10.2f} x={:10.2f} xmax={:10.2f} vr={:6.2f} ts={:6.2f}"\
    .format(ta, td, vr, xmin, x, xmax, vrmax, ts)


a = 1.0 # Max acceleration
d = 2.0

t = 400.

v0 = 50.
v1 = 100.

x = ( 32. - 5.5) * 50. * 50. 

pr(*calc(x,t,v0,v1,a, d))


a = 1.0 # Max acceleration
d = 1.0

t = 400.

v0 = 150.
v1 = 150.

x = ( 12 ) * 50 * 50 

pr(*calc(x,t,v0,v1,a, d))
