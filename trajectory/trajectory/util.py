

import struct

def sign(a): return (a>0) - (a<0)

def mkmap(r1,r2, d1,d2):
    """Map from one inter range to another"""
    r = r2-r1
    d = d2-d1

    def range(x):
        
        x = round(x,2)
        
        if x < r1:
            x = r1
        elif x > r2:
            x = r2
        
        s = float(x-r1)/float(r)
        v =  d1+(s*d)
        
        return v
    
    return range
    
# Different maps for each max speed
freq_map = [
   mkmap(0, 1, 0, 600 ),
   mkmap(0, 1, 0, 3000 ),
   mkmap(0, 1, 0, 8000 ),
   mkmap(0, 1, 0, 11000 ), 
   mkmap(0, 1, 0, 15000 ) 
]


def s32tou(v):
    """Convert a signed 32 bit in to unsigned """
    return struct.unpack('I', struct.pack('i', v))[0]

def _find_getch():
    try:
        import termios
    except ImportError:
        # Non-POSIX. Return msvcrt's (Windows') getch.
        import msvcrt
        return msvcrt.getch

    # POSIX system. Create and return a getch that manipulates the tty.
    import sys, tty
    def _getch():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    return _getch

getch = _find_getch()

def seg_dist_time(v0, x,t):
    """Return parameters for a segment given the initial velocity, distance traveled, and transit time."""

    v0 = float(v0)
    x = float(x)
    t = float(t)
    
    a = 2.*(x-v0*t)/(t*t)

    v1 = v0 + a*t

    return abs(x), v0, v1, t, a

def seg_velocity_time(v0,v1,t):
    """ Return parameters for a segment given the initial velocity, final velocity, and transit time. """
    
    v0 = float(v0)
    v1 = float(v1)
    x = float(t)
    
    a = (v1-v0)/t

    x = a * (t**2) / 2.

    return abs(x), v0, v1, t, a

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