from __future__ import print_function
#from tabulate import tabulate
from trajectory import Proto, Command, Response, SimSegment,  SegmentList, SegmentIterator, SegmentBuffer, \
                       Joystick, TimeoutException

import csv        
import unittest


from time import sleep, time

usb_port = '/dev/cu.usbmodemFD1431'

#usb_port = '/dev/ttyACM0'
        
class TestPoints(unittest.TestCase):
    
   
    def test_joy_move(self):

        def get_joy():
            while True:
                with open('/tmp/joystick') as f:
                    return [float(e) for e in f.readline().split(',')]

          
        with Proto(usb_port) as proto:
            
            last_time = time()
            last_velocities = [0]*6
            seq = 0
            
            while True:
            
                e = get_joy()

                dt = time()-last_time

                if dt >= .20 and len(proto)<=2:
                    last_time = time()
                    
                    velocities = e+[0,0]
                      
                    x = [ .5*(v0+v1)*dt  for v0, v1 in zip(last_velocities, velocities) ]
          
                    msg = Command(seq, 10, dt*1e6, last_velocities, velocities, x)
                       
                    proto.write(msg)

                    seq += 1
                    
                    last_velocities = velocities
                elif dt < .20:
                    sleep(.20-dt)

    def test_rec_joy_moves_cmd(self):

        with open('joy_moves.csv') as f:
            moves = list(csv.reader(f))

        n_axes = 6
        
        sl = SegmentList(n_axes, 800, 15000)
        for m in moves:
                sl.add_velocity_segment([float(e) for e in [m[1],0,m[2],0,0,0]],
                 t=float(m[0]))

        last_velocities = [0]*n_axes
        
        with Proto(usb_port) as proto:
            
            for seq, m in enumerate(moves):
                
                dt = float(m[0])
                velocities = [float(v)for v in [m[1],m[1],m[2],0,0,0]]
              
                x = [ .5*(v0+v1)*dt  for v0, v1 in zip(last_velocities, velocities) ]
  
                msg = Command(seq, 10, dt*1e6, last_velocities, velocities, x)
           
                proto.write(msg)
                print(msg)
                
                last_velocities = velocities
                
                while proto.wait()>1:
                    pass

    def test_rec_joy_moves_cmd(self):

        with open('joy_moves.csv') as f:
            moves = list(csv.reader(f))

        n_axes = 6
        
        sl = SegmentList(n_axes, 800, 15000)
        for m in moves:
                sl.add_velocity_segment([float(e) for e in [m[1],0,m[2],0,0,0]],
                 t=float(m[0]))

        last_velocities = [0]*n_axes
        
        with Proto(usb_port) as proto:
            
            for seq, m in enumerate(moves):
                
                dt = float(m[0])
                velocities = [float(v)for v in [m[1],m[1],m[2],0,0,0]]
              
                x = [ .5*(v0+v1)*dt  for v0, v1 in zip(last_velocities, velocities) ]
  
                msg = Command(seq, 10, dt*1e6, last_velocities, velocities, x)
           
                proto.write(msg)
                print(msg)
                
                last_velocities = velocities
                
                while proto.wait()>1:
                    pass

    def test_csv_moves(self):
        from os.path import join, dirname, abspath
        from operator import itemgetter
        d = dirname(dirname(abspath(__file__)))

        with open(join(d,'test','recorded_moves.csv')) as f:
            moves = list(csv.DictReader(f))
            
        ig = itemgetter(*('axis0,axis1,axis2,axis3,axis4,axis5'.split(',')))
        n_axes = 6
        
        sl = SegmentList(n_axes, 750, 15000)
        
        for m in moves:

            sl.add_segment([float(e) for e in ig(m)],v=float(m['v']))

        for i, s in enumerate(SegmentIterator(sl)):
            print(i,s)
            print(len(sl))

        return 

        with Proto(usb_port) as proto:
            
            for i, s in enumerate(SegmentIterator(sl)):

                msg = Command(i, 10, int(s.t_seg*1000000), 
                            [ int(sj.v0) for sj in s.joints ],
                            [ int(sj.v1) for sj in s.joints ],
                            [ int(sj.x) for sj in s.joints ])
                       
                proto.write(msg)
                print(msg)

                while proto.wait()>4:
                    pass

    def test_move_commands(self):

        n_axes = 6
        
        sl = SegmentList(n_axes, 600, 60000)

        m=600
    
        #for i in range(4):
        #    sl.add_velocity_segment([m,0,0,0,0,0], t=0.1)
        #    sl.add_velocity_segment([0,m,0,0,0,0], t=0.1)
        #    sl.add_velocity_segment([0,0,m,0,0,0], t=0.1)
        #    sl.add_velocity_segment([m,m,0,0,0,0], t=0.1)
      
        sl.add_velocity_segment([m,0,0,0,0,0], t=0.1)
      
        print(sl)

        for i, s in enumerate(SegmentIterator(sl)):
            print(i,s)


        return 

        with Proto(usb_port, timeout=.2) as proto:
            for i, s in enumerate(SegmentIterator(sl)):

                msg = Command(i, 10, int(s.t_seg*1000000), 
                            [ int(sj.v0) for sj in s.joints ],
                            [ int(sj.v1) for sj in s.joints ],
                            [ int(sj.x) for sj in s.joints ])

                print('RQST', msg)
                proto.write(msg)

                while True:

                    try:
                        proto.wait_done(i - 5)
                        break
                    except TimeoutException:
                        print("WAITING", i, i-5)




    def test_direct_messages(self):

        print ("Command size", Command.size)
        print ("Response Size", Response.size)
        
        n_axes = 6
        
        v0s = [0]*n_axes
        v1s = [0]*n_axes
        xs = [0]*n_axes
        
        v1s[0] = 400
        xs[0] = 400
        
        #v1s[1] = 400
        #xs[1] = 400
        
        #v1s[2] = 400
        #xs[2] = 400
        
        
        commands = []
        
        for i in range(3):

            commands.append(Command(i*2, 10, 2000000, v0s, v1s, xs))
            commands.append(Command(i*2+1, 10, 2000000, v1s, v0s, xs))
        
        
        with Proto(usb_port) as proto:
            for msg in commands:

                proto.write(msg)
                print(msg)

                if len(proto.sent) > 5:
                    while len(proto.sent) > 2:
                        sleep(.01)  
                else:
                    sleep(0.1)

    def test_messages(self):

        print ("Command size", Command.size)
        print ("Response Size", Response.size)
        
        null_axes = [0,1,2,3,4,5]
        
        with Proto(usb_port) as proto:
            for i in range(10):
        
                msg = Command(i, 10, 3*1, null_axes, null_axes, null_axes)
                proto.write(msg)
                print(msg)

                if len(proto.sent) > 5:
                    while len(proto.sent) > 2:
                        sleep(.01)  
                else:
                    sleep(0.1)
   
    def test_linear_segments(self):

        sl = SegmentList(6, 15000, 1)
        sl.add_segment([20000,30000,40000,0,0,0], t=400)
        sl.add_segment([20000,30000,40000,0,0,0], t=400)
        sl.add_segment([20000,30000,40000,0,0,0], t=400)
        sl.add_segment([20000,30000,40000,0,0,0], t=400)
        
        print(sl)
        
        t_sum = 0
        x_sum = [0] * 6
            
        for ss in sl.iter_subsegments():
            t_sum += ss.t_seg
            x = [e.x for e in ss.joints]
            v = [e.v1 for e in ss.joints]
            x_sum = [ xs+xi for xs, xi in zip(x_sum, x)]

        self.assertEquals(1600, round(t_sum, -1))
        self.assertEquals(80000, round(x_sum[0], -1))
        self.assertEquals(120000, round(x_sum[1], -1))
        
        sl = SegmentList(1, 15000, 1)
        sl.add_segment([0], t=400)
        sl.add_segment([0], t=400)
        
            
        for ss in sl.iter_subsegments():
            print(ss)
            
        sl = SegmentList(1, 15000, 1)
        sl.add_velocity_segment([0], t=400)
        sl.add_velocity_segment([0], t=400)
           
        for ss in sl.iter_subsegments():
            print(ss)
            
        
    def test_sub_segments(self):

        sl = SegmentList(1, 15000, 50000)
        sl.add_segment([20000], t=.05)
        
        ss=list(SegmentIterator(sl))

        # Check that velocity of run sub segment is limited
        self.assertEquals(15000, ss[1].joints[0].v0)
        self.assertEquals(15000, ss[1].joints[0].v1)
    
        self.assertEquals(20000, round(ss[2].x[0],-2))
        self.assertEquals(0.0, ss[2].joints[0].v1)
        # Final time is longer than the commanded time
        self.assertEquals(1.63, round(ss[2].t,2))
        
    def test_segments_queue(self):

        sl = SegmentList(1, 15000, 50000)

        for i in range(1,5):
            sl.add_segment([i*10000], t=1)
        
        for i, ss in enumerate(SegmentIterator(sl)):
            print(ss.joints[0].v1, i, ss)
            
            if 4 < i < 8:
                sl.add_segment([10000], t=1)
            
        
        self.assertEquals(130000, round(ss.x[0],0))
        
    def test_complex_segments(self):

        n_axes = 1
        
        sl = SegmentList(n_axes, 15000, 1)
        sl.add_segment([20000], t=400)

        print(sl)
        
        for s in sl.iter_subsegments:
            print(s)
            
        print("===========")

        sl = SegmentList(n_axes, 15000, 1)
        sl.add_segment([20000], t=400)
        sl.add_segment([-20000], t=400)

        print(sl)
        
        for s in sl.iter_subsegments:
            print(s)
            
        print("===========")

        sl = SegmentList(n_axes, 15000, 1)
        sl.add_segment([-20000], t=400)
        sl.add_segment([20000], t=400)

        print(sl)
        
        for s in sl.iter_subsegments:
            print(s)
        
    def test_sim_steps(self):

        
        def prss(ss):
            print(ss.n,ss.tn,ss.xn,ss.vn,ss.cn)
            
        def roundss(ss):
            return (ss.n,round(ss.tn,-4),ss.xn,round(ss.vn,-1),round(ss.cn,-4))
          
        #                   n      t       x     v1      cn
        self.assertEquals((500, 980000.0, 500, 1000.0,  0.0),     roundss(SimSegment(0    ,1000 , 500).run_out()))
        self.assertEquals((500, 980000.0, 500, -1000.0, 0.0),     roundss(SimSegment(0    ,-1000, 500).run_out()))
        self.assertEquals((0,   990000.0, 500, 30.0,    30000.0), roundss(SimSegment(1000 ,0    , 500).run_out()))
        self.assertEquals((0,   990000.0, 500, -30.0,   30000.0), roundss(SimSegment(-1000,0    , 500).run_out()))
        
        ss = SimSegment(0,67,6667)
        #for x in ss:
        #    prss(ss)
        
    def test_sim_params(self):

    
        ss = SimSegment(200,300, t=.5)
        print(ss)
        print(ss.run_out())
        print('---')
        ss = SimSegment(300,400, t=.5)
        print(ss)
        print(ss.run_out())
            
    def test_sim(self):

    
        sl = SegmentList(6, 40000, 15000)
        sl.add_segment([26500,0,0,0,0,0], t=2.66)
        
        for t, joints in sl:
            x, v0, v1 = joints[0]
            a = float(v1-v0)/float(t)
            print ("{} x={} v0={} v1={} a={}".format(t, x, v0, v1, a))
        
        print("-----")
    
        s = SimSegment(10000,0,5000)
        for _ in s:
            print(s)
    
    
    def test_segment_buffer(self):

        sl = SegmentBuffer(1, 15000, 50000)
        
        def mkcn(ss):

            t = ss.t_seg

            return [ (x_,)+ SimSegment.initial_params(v_, a_) for x_, v_, a_ in 
                     [ (ssj.x, ssj.v0, (round(ssj.v1,0)-round(ssj.v0,0))/t)  for ssj in ss.joints ]
                   ]
        
        for i in range(5):
            s =  sl.add_velocity_segment([10000], t=1)
            if s:
                for ss in s:
                    print(len(sl), ss, mkcn(ss))

        for ss in sl.pop():
            print (len(sl),ss, mkcn(ss))
    
    def test_joy_buffer(self):
        

        
        sl = SegmentBuffer(4, 15000, 50000)
        
        def mkcn(ss):
            t = ss.t_seg
            
            return [ (x_,)+ SimSegment.initial_params(v_, a_) for x_, v_, a_ in 
                     [ (ssj.x, ssj.v0, (round(ssj.v1,0)-round(ssj.v0,0))/t)  for ssj in ss.joints ]
                   ]
        
        last = time()
        for e in Joystick(.25):
            
            if last + .20 < time():
                dt = time() - last
                last = time()

                s =  sl.add_velocity_segment(e, t=dt)
                if s:
                    for ss in s:
                        if ss.t_seg > 0:
                            print(mkcn(ss))
    
                  
    def test_plot(self):
        import pandas as pd 
        import matplotlib.pyplot as plt
        
        l = [ (t/1.e6,x,v) for t,x,v, cn, n in SimSegment(0,1000,500) ]
        
        df = pd.DataFrame(l, columns='t x v'.split())
        
        print(df.head())
        
        df.plot(x='t',y='x')

        plt.show()
        
    def test_plot2(self):
        """Plot v vs t for a set of segments, both from the endpoints of the segments
        and a simulation of the step algorithm. """
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