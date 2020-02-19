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
            self.interval = int(t * 500)
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

        while True:

            event = pygame.event.wait()

            if (event.type == KEYDOWN and event.key == K_ESCAPE):
                pygame.quit()
                return

            elif (event.type == QUIT):
                pygame.quit()
                return

            elif event.type in joy_events:

                i = event.joy
                j = self.joysticks[i]

                button = max([0] + [z + 1 for z in range(j.get_numbuttons()) if j.get_button(z) and z in range(4)])

                axis_mode = max(
                    [0] + [z for z in range(j.get_numbuttons()) if j.get_button(z) and z in range(4, 8)])

                m = freq_map[button]

                def p(axis):
                    v = j.get_axis(axis)
                    v = v if abs(v) > .03 else 0
                    return copysign(m(abs(v)), v)

                hats = [j.get_hat(i) for i in range(j.get_numhats())]

                axes = [p(axis) for axis in range(j.get_numaxes())] + \
                       [copysign(m(abs(h * .5)), h) for h in hats[0]]

                now = time()
                delta = now - lasttime

                axes = [ 0 if abs(a) < 30 else a for a in axes]

                self.last = JoyValues( seq, now, delta, button, axis_mode, axes )
                lasttime = now
                seq += 1

                yield self.last

    def __del__(self):
        import pygame
        pygame.display.quit()
