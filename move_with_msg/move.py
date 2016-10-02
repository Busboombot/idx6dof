#! /usr/local/bin/python

import time 
from proto import Proto, Message

code = 10
directions = 57
steps = [1,2,3,4,5,6]
us_per_steps=[10,20,30,40,50,60]

print "Message size: ", Message.size


use_axes = 6
nulls = [0]*(6-use_axes)

def step_test():
    
    d = (
        (25, 500),
      
    )
    seq = 0;
    with Proto('/dev/cu.usbmodemFD1441') as proto:
        while True:
            seq += 1;
            for (ticks, steps) in d:
                msg = Message(seq, 10,  0, [ticks]*use_axes+nulls, [steps]*use_axes+nulls)
                proto.write(msg)

            
            if len(proto.sent) > 1:
                while len(proto.sent) > 1:
                    time.sleep(.01)
                    
                

def speed_test():

    with Proto('/dev/cu.usbmodemFD1441') as proto:

        t1 = time.time()
        N = 10;
        for seq in range(N):

            msg = Message(seq, code,  directions, ticks, steps)
        
            proto.write(msg)

        while(len(proto.sent) > 0):
            pass
            #print (len(proto.buf), len(proto.sent))
    
        print (time.time()-t1) / float(N)


step_test()
    
    

