#! /usr/local/bin/python
from __future__ import division
import time 
from proto import Proto, Command, Response

code = 10

steps = [1,2,3,4,5,6]
us_per_steps=[10,20,30,40,50,60]

print "Command  size: ", Command.size
print "Response size: ", Response.size

use_axes = 6
nulls = [0]*(6-use_axes)

def setBit(int_type, offset):
    mask = 1 << offset
    return(int_type | mask)


seq = 0;
with Proto('/dev/cu.usbmodemFD1431') as proto:
    while True:
        
        for freq in (100,200,300,200,100,0,
                    -100,-200,-300,-200,-100,0):
            
            if freq > 0:
                directions = 0x0000ff
            else:
                directions = 0
                
            if freq == 0:
                ticks = steps = 0
            else:
                ticks = int(1000000 / abs(freq))
                steps = int(2000000 / ticks) # .2 sec for every message
            
            seq += 1;
            msg = Command(seq, 10,  directions, [ticks]*use_axes+nulls, [steps]*use_axes+nulls)
            proto.write(msg)

        if len(proto.sent) > 5:
            while len(proto.sent) > 2:
                time.sleep(.01)  
        else:
            time.sleep(0.1)      



    