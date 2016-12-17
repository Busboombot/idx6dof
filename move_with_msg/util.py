


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