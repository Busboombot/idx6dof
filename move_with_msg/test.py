from planning import MotionPlanner, TrajectoryPoint
from tabulate import tabulate
from proto import Proto, Command, Response
from stepsegments import SimSegment

import unittest

def compare(v0, v1, x):
    (x, v0, v1, t, a) = Proto.seg_velocity_dist(v0, v1, x)

    ss = SimSegment(v0, v1, x).runOut()

    def almost_equal(v1, v2):
        return abs((float(v1)-float(v2)) / v1) < .1

    if (ss.xn!= x or ss.v0 != v0 or not almost_equal(int(ss.vn+v0),int(v1+v0)) or 
        ss.a != a or not almost_equal(ss.tn, t*1000000.) ):
        print 'E:  Expected: {:6d}->{:6d} in {:6d} steps, {:6.2f} s , a={:6.2f}'.format(v0, v1, x, t, a)
        print 'E:  Actual  :',ss
    else:
        print 'OK: Actual  :',ss

class TestPoints(unittest.TestCase):
    
    
    def test_accel_calcs(self):

        compare(600,600,2400)
        compare(-600,-600,2400)  
    
        compare(0,600,2400)
        compare(600,0,2400)
        compare(0,-600,2400)
        compare(-600,0,2400)
    
        compare(599,601,2400)
        compare(601,599,2400)
        compare(-599,-601,2400)
        compare(-601,-599,2400)
    
    
    def test_accel_cals_2(self):
        
        ss = SimSegment(0, 10, 10)
    
        for e in ss:
            print ss
    
    def test_point(self):
        
        p = TrajectoryPoint(2,[100])
        
        print(p)
        
        p1, p2 = p.split(1)
        
        print(p1)
        print(p2)
        
    def test_split(self):
        
        p = TrajectoryPoint(1,[100,100,100], [-100,-105,-110])

        for s in p.yield_splits(500):
            print s
    
    
    def test_run_move(self):
        
        p = TrajectoryPoint(0,[0,0])
        
        print(p)
        
        p = p.move(1,[100,100])

        print(p)
        
        p = p.run(1,[500,500])

        print(p)
    
    
    def test_gen_params(self):
        
        p = TrajectoryPoint(0,[0])
        
        x = 1000
        
        for i in range(10):
 
            p = p.move(5,[x])
            x = -x
            
            for s in p.yield_splits(500):
                print '----', s.abs_t0, s.abs_x0
                print s

       
    
    def test_joy(self):
        
        from joystick import Joystick
        from time import sleep, time
        from joystick import Joystick
        
        last = time()
        p = TrajectoryPoint(0,[0,0])
        for e in Joystick(.25):
            
            if last + .20 < time():
            
                dt = time() - last
                last = time()
                
                p = p.run(dt, v1=e)
                print p
            
    def test_messages(self):
        import time
        
        print (Command.size)
        print (Response.size)
        
        null_axes = [0]*6
        
        with Proto('/dev/cu.usbmodemFD1431') as proto:
            for i in range(10):
        
                msg = Command(10, i, 3*1, null_axes, null_axes, null_axes)
                proto.write(msg)
                print(msg)

                if len(proto.sent) > 5:
                    while len(proto.sent) > 2:
                        time.sleep(.01)  
                else:
                    time.sleep(0.1)
        
    
    def x_test_trajectory(self):
        mp = MotionPlanner(100000,500000, 1000000)
        mp.add_point(TrajectoryPoint(.1, [1000,500]))
        mp.add_point(TrajectoryPoint(.1, [2000,-500]))
        mp.add_point(TrajectoryPoint(.1, [4000,500]))
        mp.add_point(TrajectoryPoint(.1, [0,-500]))

        print "Positions"
        print tabulate(  [ p.info_row() for p in mp.moves] , headers = "T V L J0 J1 J2 J3 J4 J5".split())

        print "\nVelocities"
        print tabulate( mp.velocities, headers = "J0 J1 J2 J3 J4 J5".split())

        print "\nAccelerations"
        print tabulate( mp.accelerations, headers = "J0 J1 J2 J3 J4 J5".split())
    
if __name__ == '__main__':
    unittest.main()