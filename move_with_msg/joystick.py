#! /usr/local/bin/python
from __future__ import division
import time 
from math import copysign
import pygame
from pygame.locals import KEYDOWN, K_ESCAPE, QUIT

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
        v =  d1+(s*d)
        
        if v< 90:
            return 0
        
        return v
    
    return range
    
# Different maps for each max speed
freq_map = [
   mkmap(0, 1, 70, 600 ),
   mkmap(0, 1, 70, 3000 ),
   mkmap(0, 1, 50, 8000 ),
   mkmap(0, 1, 50, 11000 ), 
   mkmap(0, 1, 1, 15000 ) 
] 


class Joystick(object):
    
    def __init__(self, t = None):
        """Read the pygame joystick and yield frequency values for the stepper motors. 
        
        param t: minimum frequency, in seconds,  at which to yield a result, even if there are no changes.
        Defaults to 1 
        """
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
            
        if t:
            self.interval = int(t * 1000)
        else:
            self.interval = 1000
            
    def __iter__(self):


        seq = 0;

        pygame.time.set_timer(pygame.USEREVENT, self.interval)

        last = [None, None, None, None]

        while True:
        
            event = pygame.event.wait()

            if (event.type == KEYDOWN and event.key == K_ESCAPE):
                pygame.quit()
                return

            elif (event.type == QUIT):
                pygame.quit()
                return
                
            elif (event.type == pygame.USEREVENT):
                
                yield last
                
            else:

                for i, j in enumerate(self.joysticks):
                
                    buttons =  [z  for z in range(j.get_numbuttons()) if j.get_button(z) ]
              
                    try:
                        m = freq_map[buttons[0]+1]
                    except:
                        m = freq_map[0]
                
                
                    last =  [ copysign(m(abs(j.get_axis(axis))),j.get_axis(axis)) for axis in range(j.get_numaxes())]
                
                    yield last
                


    def __del__(self):
        
        pygame.display.quit()
  

   
  
if __name__ == "__main__":
    
    # Run the joystick in a loop and write values to a file. 
    
    import sys
    
    with open(sys.argv[1], "wb") as f:
        for values in Joystick(.5):
            f.write(','.join(str(e) for e in values))
            f.write('\n')
            f.seek(0)

    

    