#! /usr/local/bin/python
from __future__ import division
import time 
from proto import Proto, Command


code = 10

steps = [1,2,3,4,5,6]
us_per_steps=[10,20,30,40,50,60]

print "Message size: ", Command.size

use_axes = 6
nulls = [0]*(6-use_axes)

def mkmap(r1,r2, d1,d2):
    """Map from one inter range to another"""
    r = r2-r1
    d = d2-d1

    def range(x):
        
        if x < r1:
            x = r1
        elif x > r2:
            x = r2
        
        s = float(x-r1)/float(r)
        return d1+(s*d)
    
    return range
    
def setBit(int_type, offset):
    mask = 1 << offset
    return(int_type | mask)
    

# Different maps for each max speed
freq_map = [
    mkmap(0, 1, 80, 700 ),
    mkmap(0, 1, 80, 3000 ),
    mkmap(0, 1, 50, 8000 ),
    mkmap(0, 1, 50, 11000 ), 
    mkmap(0, 1, 1, 15000 ) 
]

class App(object):
    def __init__(self):
        import pygame

        pygame.init()
        pygame.joystick.init()

        self.joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]

        for j in self.joysticks:
            j.init()
        
        for i, j in enumerate(self.joysticks):
            pass
            #print j.get_init(), j.get_name()
            #print [j.get_axis(k) for k in range(j.get_numaxes())]
            #print [j.get_button(z) for z in range(j.get_numbuttons())]
            
    def main(self):
        import pygame
        from pygame.locals import KEYDOWN, K_ESCAPE, QUIT

        seq = 0;
        with Proto('/dev/cu.usbmodemFD1421') as proto:
            while True:
            
                self.g_keys = pygame.event.get()


                for event in self.g_keys:
                    if (event.type == KEYDOWN and event.key == K_ESCAPE):
                        self.quit()
                        return

                    elif (event.type == QUIT):
                        self.quit()
                        return

                if len(proto.sent) <= 1:
                    
                    ticks = [0]*6
                    steps = [0]*6
                    directions = 0x0000000
                    
                    for i, j in enumerate(self.joysticks):
                        
                        buttons =  [z  for z in range(j.get_numbuttons()) if j.get_button(z) ]
                      
                        try:
                            m = freq_map[buttons[0]+1]
                        except:
                            m = freq_map[0]
                        
                        
                        for axis in range(j.get_numaxes()):
                            v = j.get_axis(axis)
                            if v > 0:
                                directions = setBit(directions, axis)
                            freq = int(m(abs(v))) 

                            ticks[axis] = int(1000000 / freq)
                            steps[axis] = int(20000 / ticks[axis]) # .2 sec for every message
                            
                            # Force close to zero to exactly zero. 
                            if ticks[axis] >= 10000:
                                ticks[axis] = 0
                                steps[axis] = 0

                    if sum(steps): # only send messages that have movement. 
                      
                        msg = Command(seq, 10,  directions, ticks, steps)
                        seq += 1
                        seq = seq % (2**16-1)
                        
                        proto.write(msg)
                        

                if len(proto.sent) > 5:
                    while len(proto.sent) > 1:
                        time.sleep(.01)
                else:
                    # It appears that this sleep is needed to prevent the 
                    # serial read thread from starving. If the thread starves, 
                    # proto.sent never hits it's limit in the if clause
                    time.sleep(.01)

    def quit(self):
        pygame.display.quit()
            
app = App();
app.main()
    

    