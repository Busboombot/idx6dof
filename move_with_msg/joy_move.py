#! /usr/local/bin/python
from __future__ import division
import time 
from proto import Proto, Command
from joystick import Joystick


def setBit(int_type, offset):
    mask = 1 << offset
    return(int_type | mask)


for e in Joystick():
    


    

    