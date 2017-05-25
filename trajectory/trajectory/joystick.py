#! /usr/local/bin/python
from __future__ import division
import time 
from math import copysign
from .util import freq_map



class PygameJoystick(object):
    
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
        import pygame
        from pygame.locals import KEYDOWN, K_ESCAPE, QUIT

        seq = 0;

        pygame.time.set_timer(pygame.USEREVENT, self.interval)

        last = [0, 0, 0, 0, 0, 0, 0]

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
                
                    button =  max([0]+[z+1 for z in range(j.get_numbuttons()) if j.get_button(z) and z in range(4) ] )
                 
                    axis_mode = max( [0]+[z for z in range(j.get_numbuttons()) if j.get_button(z) and z in range(4, 8) ])

                    m = freq_map[button]
                 
                    hats = [ j.get_hat( i ) for i in range(j.get_numhats()) ]


                    last =  [axis_mode]+\
                            [ copysign(m(abs(j.get_axis(axis))),j.get_axis(axis)) 
                            for axis in range(j.get_numaxes())] + \
                            [ copysign(m(abs(h*.5)),h) for h in hats[0] ]
                           
                    yield last
                

    def __del__(self):
        import pygame
        pygame.display.quit()
  

   

    

    