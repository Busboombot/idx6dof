
v0 = 0 # Initial velocity
v = 1000 # Final velocity steps per second
a = 1000 # Acell in steps /s^2

ta = (v - v0) / a # Acceleration time
s = (v**2 - v0**2) / 2*a

def next_v(v, a):
    import math
    return sqrt(v+2*a)
    
vi = v0
si = 0
for i in range(s):
    si = i
    