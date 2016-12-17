from planning import MotionPlanner, TrajectoryPoint
from tabulate import tabulate
from proto import Proto, Command, Response
from sim import SimSegment

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
            
    def test_simple_messages(self):
        import time
        
        print ("Command size", Command.size)
        print ("Response Size", Response.size)
        
        null_axes = [0,1,2,3,4,5]
        
        with Proto('/dev/cu.usbmodemFD1431') as proto:
            
            msg = Command(1, 10, 3*1, null_axes, null_axes, null_axes)
            proto.write(msg)
            print(msg)
            
            msg = Command(2, 10, 3*1, null_axes, null_axes, null_axes)
            proto.write(msg)
            print(msg)
        
            while len(proto.sent) > 0:
                time.sleep(.01)


            
    def test_messages(self):
        import time
        
        print ("Command size", Command.size)
        print ("Response Size", Response.size)
        
        null_axes = [0,1,2,3,4,5]
        
        with Proto('/dev/cu.usbmodemFD1431') as proto:
            for i in range(10):
        
                msg = Command(i, 10, 3*1, null_axes, null_axes, null_axes)
                proto.write(msg)
                print(msg)

                if len(proto.sent) > 5:
                    while len(proto.sent) > 2:
                        time.sleep(.01)  
                else:
                    time.sleep(0.1)

    
        
    def test_linear_segments(self):
        from segments import SegmentList
        
        sl = SegmentList(6, 15000, 1)
        sl.add_segment([20000,30000,40000,0,0,0], t=400)
        sl.add_segment([20000,30000,40000,0,0,0], t=400)
        sl.add_segment([20000,30000,40000,0,0,0], t=400)
        sl.add_segment([20000,30000,40000,0,0,0], t=400)
        
        print sl
        
        for s in sl.iter_subsegments:
            print s
        
        t_sum = 0
        x_sum = [0] * 6
        for t, joints in sl.iter_subsegments:
             t_sum += t
             x = [e[0] for e in joints]
             v = [e[2] for e in joints]
             x_sum = [ xs+xi for xs, xi in zip(x_sum, x)]

        self.assertEquals(1600, round(t_sum, -1))
        self.assertEquals(80000, round(x_sum[0], -1))
        self.assertEquals(120000, round(x_sum[1], -1))
        
    def test_sub_segments(self):
        from segments import SegmentList, SegmentIterator
        
        sl = SegmentList(1, 15000, 1)
        sl.add_segment([20000], t=400)
        sl.add_segment([-20000], t=400)
        
        for s in sl.iter_subsegments():
            print s
            
        print '======'
        
        si = SegmentIterator(sl)
        print si.positions, si.velocities
        for ss in si:
            print ss
            print si.positions, si.velocities
        
        
    def test_complex_segments(self):
        from segments import SegmentList
        
        n_axes = 1
        
        sl = SegmentList(n_axes, 15000, 1)
        sl.add_segment([20000], t=400)

        print sl
        
        for s in sl.iter_subsegments:
            print s
            
        print "==========="

        sl = SegmentList(n_axes, 15000, 1)
        sl.add_segment([20000], t=400)
        sl.add_segment([-20000], t=400)

        print sl
        
        for s in sl.iter_subsegments:
            print s
            
        print "==========="

        sl = SegmentList(n_axes, 15000, 1)
        sl.add_segment([-20000], t=400)
        sl.add_segment([20000], t=400)

        print sl
        
        for s in sl.iter_subsegments:
            print s
        
    def test_sim_steps(self):
        from sim import SimSegment
        from math import sqrt
        from operator import attrgetter
        
        def prss(ss):
            print ss.n,ss.tn,ss.xn,ss.vn,ss.cn
            
        def roundss(ss):
            return (ss.n,round(ss.tn,-4),ss.xn,round(ss.vn,-1),round(ss.cn,-4))
          
        #                   n      t       x     v1      cn
        self.assertEquals((500, 980000.0, 500, 1000.0,  0.0),     roundss(SimSegment(0    ,1000 , 500).run_out()))
        self.assertEquals((500, 980000.0, 500, -1000.0, 0.0),     roundss(SimSegment(0    ,-1000, 500).run_out()))
        self.assertEquals((0,   990000.0, 500, 30.0,    30000.0), roundss(SimSegment(1000 ,0    , 500).run_out()))
        self.assertEquals((0,   990000.0, 500, -30.0,   30000.0), roundss(SimSegment(-1000,0    , 500).run_out()))
        
        ss = SimSegment(0,67,6667)
        for x in ss:
            prss(ss)
            
        
    def test_sim(self):
        from sim import SimSegment
        from segments import SegmentList
    
        sl = SegmentList(6, 40000, 15000)
        sl.add_segment([26500,0,0,0,0,0], t=2.66)
        
        for t, joints in sl:
            x, v0, v1 = joints[0]
            a = float(v1-v0)/float(t)
            print "{} x={} v0={} v1={} a={}".format(t, x, v0, v1, a)
        
        print "-----" 
    
        s = SimSegment(10000,0,5000)
        for _ in s:
            print s
    
    
            
    def test_plot(self):
        import pandas as pd 
        import matplotlib.pyplot as plt
        
        l = [ (t/1.e6,x,v) for t,x,v, cn, n in SimSegment(0,1000,500) ]
        
        df = pd.DataFrame(l, columns='t x v'.split())
        
        print df.head()
        
        df.plot(x='t',y='x')

        plt.show()
        
    def test_plot2(self):
        """Plot v vs t for a set of segments, both from the endpoints of the segments
        and a simulation of the step algorithm. """
        from segments import SegmentList
        import pandas as pd 
        import numpy as np
        import matplotlib.pyplot as plt
        
        sl = SegmentList(6, 15000, 1)
        sl.add_segment([20000,30000,40000,0,0,0], t=400)
        sl.add_segment([20000,30000,40000,0,0,0], t=400)
        sl.add_segment([20000,30000,40000,0,0,0], t=400)
        sl.add_segment([20000,30000,40000,0,0,0], t=400)
        
        t_sum = ti_sum = 0
        x_sum = xi_sum = 0
        l = [(0,0,0)]
        sims = []
        plot_joint = 0
        
        for t, joints in sl:
            t_sum += t
            
            joint = joints[plot_joint]
            
            x = joint[0]
            v0 = joint[1]
            v1 = joint[2]
            
            x_sum += x
            l.append( (t_sum, x_sum,  v1) )
            
            sims += [ ((ti_sum+ti)/1.e6,xi_sum+xi,vi) for ti,xi,vi,cn,n in SimSegment(v0,v1,int(x)) ]
            ti_sum += ti
            xi_sum += xi

        df = pd.DataFrame(l, columns='t x v'.split())
        df2 = pd.DataFrame(sims, columns='t xs vs'.split())
        
        
        fig, ax = plt.subplots(1, 1)
        df.plot(ax=ax,x='t',y='x')
        df2.plot(ax=ax,x='t',y='xs')
        
        plt.show()


    
if __name__ == '__main__':
    unittest.main()