#! /usr/local/bin/python
from __future__ import division
import time 
from proto import Proto, Message



code = 10

steps = [1,2,3,4,5,6]
us_per_steps=[10,20,30,40,50,60]

print "Message size: ", Message.size


use_axes = 6
nulls = [0]*(6-use_axes)


def joy_move():
    
    def mkmap(r1,r2, d1,d2):
    
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
    
    m = mkmap(0, 1, 0, 800 ) # DOmain is in kHs step frequency
    
    class App(object):
        def __init__(self):
            import pygame

            pygame.init()
            pygame.joystick.init()
    
            self.joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
    
            for j in self.joysticks:
                j.init()
            
        def main(self):
            import pygame
            from pygame.locals import KEYDOWN, K_ESCAPE, QUIT

            seq = 0;
            with Proto('/dev/cu.usbmodemFD1411') as proto:
                while True:
                
                    self.g_keys = pygame.event.get()
 
                    for event in self.g_keys:
                        if (event.type == KEYDOWN and event.key == K_ESCAPE):
                            self.quit()
                            return
 
                        elif (event.type == QUIT):
                            self.quit()
                            return
                
                    for i, j in enumerate(self.joysticks):
                        pass
                        #print j.get_init(), j.get_name()
                        #print [j.get_axis(k) for k in range(j.get_numaxes())]
                        #print [j.get_button(z) for z in range(j.get_numbuttons())]
                    
                    if len(proto.sent) <= 1:
                        
                        ticks = [0]*6
                        steps = [0]*6
                        directions = 0x0000000
                        
                        for i, j in enumerate(self.joysticks):
                            for axis in range(j.get_numaxes()):
                                v = j.get_axis(axis)
                                if v > 0:
                                    directions = setBit(directions, axis)
                                freq = int(m(abs(v))) 
                                ticks[axis] = int(1000000 / freq)
                                steps[axis] = int(20000 / ticks[axis]) # .2 sec for every message
                                
                                if ticks[axis] > 10000:
                                    ticks[axis] = 0
                                    steps[axis] = 0
                                

                        
                        if sum(steps):
                          
                            msg = Message(seq, 10,  directions, ticks, steps)
                            seq += 1
                            seq = seq % (2**16-1)
                            
                            proto.write(msg)
                            
                    
                
                    if len(proto.sent) > 1:
                        while len(proto.sent) > 1:
                            time.sleep(.01)
    
    def quit(self):
        pygame.display.quit()
            
    app = App();
    app.main()
    
def key_move():
    
    from util import getch
    
    seq = 0;
    with Proto('/dev/cu.usbmodemFD1441') as proto:
        while True:
            c = getch()
            try:
                v = int(c)
            except KeyboardInterrupt:
                break
            except:
                if c == '.' or ord(c) == 3:
                    break
                print c
                continue
            
            axis = (v-1)%3
            speed = [50,150,500][(v-1)//3]
        
            print axis, speed
        
    

def step_test():
    
    d = (
        (0x0fffffff, 50, 10000),
        (0, 50, 10000),
        (0x0fffffff, 50, 10000),
        (0, 50, 10000),
      
    )
    seq = 0;
    with Proto('/dev/cu.usbmodemFD1441') as proto:
        while True:
            
            for (directions, ticks, steps) in d:
                seq += 1;
                msg = Message(seq, 10,  directions, [ticks]*use_axes+nulls, [steps]*use_axes+nulls)
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


joy_move()
    
    

