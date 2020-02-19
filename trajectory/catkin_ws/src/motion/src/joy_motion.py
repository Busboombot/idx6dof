#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from stepper.msg import MoveCommand
from time import sleep
from math import copysign, ceil, sqrt




velocity_map = [2,4,8,16]

dt = .2 # Length of time for the move commands

def get_axis_vmax(axis):
     return rospy.get_param('/stepper/axis{}/v_max'.format(axis),  rospy.get_param('/stepper/v_max'))

def callback(data, memo):
    
    try:
        v_mult = velocity_map[data.buttons.index(1)]
    except (ValueError, IndexError):
        v_mult = 1
    
    x = [ v_mult*v_base*v_joy*dt  for v_base, v_max, v_joy in zip(memo['v_base'], memo['v_max'], data.axes)]
    
    x += [0] * (6-memo['n_axes'])
    
    memo['x'] = x
    
    memo['dist'] = sum(e*e for e in memo['x'])
    
    if memo['dist'] != 0:
        memo['run'] = True;
    
def make_memo(pub):
    
    n_axes = rospy.get_param('/stepper/n_axes');
    
    v_max = [get_axis_vmax(axis) for axis in range(n_axes)]

    v_base = [v/velocity_map[-1] for v in v_max]

    memo={
        'pub': pub, 
        'v_max' : v_max,
        'v_base': v_base,
        'n_axes' : n_axes,
        'freq_map': None, 
        'x': [0]*n_axes,
        'dist': 0
       
    }

    return memo

def listener():

    rospy.init_node('motion')

    #rate = rospy.Rate(10) # In messages per second

    pub = rospy.Publisher('/stepper/cmd', MoveCommand, queue_size=10)

    memo = make_memo(pub)

    rospy.Subscriber("joy", Joy, callback, memo)

    #rospy.Timer(rospy.Duration(.05), lambda event: timed_callback(event, memo))

    seq = 0;
    
    try: 
        while not rospy.core.is_shutdown(): 
            rospy.rostime.wallsleep(0.05) 

            if(memo['dist'] > 0):
                m = MoveCommand(movetype=MoveCommand.JOG, t=dt, x=memo['x'])
                m.header.stamp = rospy.Time.now() 
                m.header.seq = seq
                seq+=1
                memo['pub'].publish(m)
                print(m)
               
                
    except KeyboardInterrupt: 
        logdebug("keyboard interrupt, shutting down") 
        rospy.core.signal_shutdown('keyboard interrupt')

if __name__ == '__main__':
    listener()