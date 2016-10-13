
from math import sqrt 
from time import time

path = [
    (1000,1000), # final velocity, steps
    (1000,1000),
    (0,1000)
]

path = [ (1000,1000)]

v = 0
v0 = 0

for v1,s in path:
    dv = float(v1 - v)
    t =   (2. * s / (v0+v1)) 
    
    a = float(v1-v0)  / float(t)
    
    v0 = v1
    
    print t, v1, a

def run():
    
    position = 0;
    expected_position = 0;
    steps_to_go = 0;
    velocity = 0;
    d_v = 0;
    interval= 0;
    acceleration = 0;

    v, s  = 0,0
    
   
    
    last_time = t = 0
    
    print 'time,interval,position,velocity,acceleration'
    
    while True:

    
        if steps_to_go == 0:
            try:
                v1,s = path.pop(0)

            except IndexError:
                return # Done!
            
            v0 = velocity
            
            d_v = float(v1 - v0) 
            
            seg_time =   (2. * s / (v0+v1)) 
            
            a = float(d_v)  / float(seg_time) 
            
            steps_to_go = s;

            interval = 0.676 * sqrt(2.0 / a) * 1000000.0; 
            
            n = 0
            
            print "\nv0={}, v1={} v={} t={} a={} I={}\n".format(v0, v1, velocity,  seg_time, acceleration, interval)


        if  (t - last_time) > interval:

            velocity = 1. / interval

            if velocity > 0:
                position += 1
            else:
                position -= 1

            n += 1
            steps_to_go -= 1

            interval = interval - ((2.0 * interval) / ((4.0 * n) + 1)); 

            

            print ','.join( str(x) for x in [t/1000000., interval, position, velocity, a])

            last_time = t
        t += 1


        
run()