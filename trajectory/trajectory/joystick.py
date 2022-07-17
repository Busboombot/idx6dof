#! /usr/local/bin/python
from __future__ import division

from math import copysign

from .util import freq_map
from time import time
from collections import namedtuple

JoyValues = namedtuple('JoyValues', 'seq now delta button axis_mode axes'.split())

class PygameJoystick(object):

    def __init__(self, t=None):
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
            # print j.get_init(), j.get_name()
            # print [j.get_axis(k) for k in range(j.get_numaxes())]
            # print [j.get_button(z) for z in range(j.get_numbuttons())]

        if t:
            self.interval = int(t*1000)
        else:
            self.interval = 500

        self.last = JoyValues(0,0,0,0,0,[0]*6)

    def __iter__(self):
        import pygame
        from pygame.locals import KEYDOWN, K_ESCAPE, QUIT, \
            JOYAXISMOTION, JOYBALLMOTION, JOYBUTTONDOWN, JOYBUTTONUP, JOYHATMOTION

        joy_events = (JOYAXISMOTION, JOYBALLMOTION, JOYBUTTONDOWN, JOYBUTTONUP, JOYHATMOTION)

        seq = 0;
        lasttime = time()

        m = lambda v: int(v * 1000)  # freq_map[button]

        def p(axis):
            v = j.get_axis(axis)
            v = v if abs(v) > .03 else 0
            return int(copysign(m(abs(v)), v))

        while True:

            event = pygame.event.wait(self.interval)

            if (event.type == KEYDOWN and event.key == K_ESCAPE):
                pygame.quit()
                return

            elif (event.type == QUIT):
                pygame.quit()
                return

            else:

                if event and event.type in joy_events:
                    i = event.joy
                    j = self.joysticks[i]

                    button = [z + 1 for z in range(j.get_numbuttons()) if j.get_button(z) and z in range(4)]

                    axis_mode = [z for z in range(j.get_numbuttons()) if j.get_button(z) and z in range(4, 8)]


                    hats = [j.get_hat(i) for i in range(j.get_numhats())]

                    axes = [p(axis) for axis in range(j.get_numaxes())] + \
                           [copysign(m(abs(h)), h) for h in hats[0]]

                    axes = [0 if abs(a) < 30 else a for a in axes]

                else:
                    button = self.last.button
                    axis_mode = self.last.axis_mode
                    axes = self.last.axes

                now = time()
                delta = now - lasttime

                self.last = JoyValues( seq, now, delta, button, axis_mode, axes )
                lasttime = now
                seq += 1

                yield self.last


    def __del__(self):
        import pygame
        pygame.display.quit()
