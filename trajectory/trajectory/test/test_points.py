# from tabulate import tabulate
import csv
import unittest
from time import sleep, time

from trajectory import Proto, Command, SimSegment, SegmentList, Joystick

usb_prog_port = '/dev/cu.usbmodem1464101'
usb_native_port = '/dev/cu.usbmodem64213801'
usb_baud = 115200
# usb_port = '/dev/ttyACM0'
n_axes = 6


class TestPoints(unittest.TestCase):

    def test_joy_move(self):

        def get_joy():
            while True:
                with open('/tmp/joystick') as f:
                    return [float(e) for e in f.readline().split(',')]

        with Proto(usb_native_port, baud=usb_baud) as proto:

            last_time = time()
            last_velocities = [0] * 6
            seq = 0

            while True:

                e = get_joy()

                dt = time() - last_time

                if dt >= .20 and len(proto) <= 2:
                    last_time = time()

                    velocities = e + [0, 0]

                    x = [.5 * (v0 + v1) * dt for v0, v1 in zip(last_velocities, velocities)]

                    msg = Command(seq, 10, dt * 1e6, last_velocities, velocities, x)

                    proto.write(msg)

                    seq += 1

                    last_velocities = velocities
                elif dt < .20:
                    sleep(.20 - dt)

    def test_rec_joy_moves_cmd(self):

        with open('joy_moves.csv') as f:
            moves = list(csv.reader(f))

        n_axes = 6

        sl = SegmentList(n_axes, 800, 15000)
        for m in moves:
            sl.add_velocity_segment([float(e) for e in [m[1], 0, m[2], 0, 0, 0]],
                                    t=float(m[0]))

        last_velocities = [0] * n_axes

        with Proto(usb_port) as proto:

            for seq, m in enumerate(moves):

                dt = float(m[0])
                velocities = [float(v) for v in [m[1], m[1], m[2], 0, 0, 0]]

                x = [.5 * (v0 + v1) * dt for v0, v1 in zip(last_velocities, velocities)]

                msg = Command(seq, 10, dt * 1e6, last_velocities, velocities, x)

                proto.write(msg)
                print(msg)

                last_velocities = velocities

                while proto.wait() > 1:
                    pass

    def test_rec_joy_moves_cmd_2(self):

        with open('joy_moves.csv') as f:
            moves = list(csv.reader(f))

        n_axes = 6

        sl = SegmentList(n_axes, 800, 15000)
        for m in moves:
            sl.add_velocity_segment([float(e) for e in [m[1], 0, m[2], 0, 0, 0]],
                                    t=float(m[0]))

        last_velocities = [0] * n_axes

        with Proto(usb_port) as proto:

            for seq, m in enumerate(moves):

                dt = float(m[0])
                velocities = [float(v) for v in [m[1], m[1], m[2], 0, 0, 0]]

                x = [.5 * (v0 + v1) * dt for v0, v1 in zip(last_velocities, velocities)]

                msg = Command(seq, 10, dt * 1e6, last_velocities, velocities, x)

                proto.write(msg)
                print(msg)

                last_velocities = velocities

                while proto.wait() > 1:
                    pass


    def test_multiple_velocities(self):
        """Execute moves froma file"""

        import saleae, time
        s = saleae.Saleae()
        s.set_capture_seconds(1)

        start_time = time.time()

        def cur_time():
            return round((time.time() - start_time), 4)

        def callback(proto, resp):
            print(cur_time(), resp)

        with Proto(usb_native_port, baud=usb_baud, callback=callback) as proto:
            proto.reset()

            dt = 3 # Seconds

            for i,v in enumerate([1000,2000,3000,4000,5000,6000] ):

                velocities = [v*1]*n_axes

                x = [ v * dt for v in  velocities]

                msg = proto.send_apos(dt, velocities, velocities, x)

                s.capture_start()
                print(msg)
                proto.wait()


                s.export_data2('/tmp/saleae.csv')

                break


    def test_lots_of_resets(self):

        def callback(proto, resp):
            print(resp);

        with Proto(usb_native_port, callback=callback, baud=usb_baud) as proto:
            for i in range(1000):
                proto.reset()

    def test_random_messages(self):
        from random import randint

        def randarray32():
            return [ randint(-(2**31), 2**31) for i in range(6)]

        def randarrayu16():
            return [ randint(0,2*16) for i in range(6)]

        def randintu16():
            return randint(0,2**16)

        def randintu32():
            return randint(0,2**32)


        def callback(proto, resp):
            print(resp);

        with Proto(usb_native_port, callback=callback, baud=usb_baud) as proto:

            for i in range(10):
                for j in range(10):
                    msg = Command(randintu16(), randintu16(), randintu32(),
                                   randarrayu16(), randarrayu16(), randarray32())
                    print(msg)
                    proto.write(msg)

                sleep(2);
                proto.reset()




    def run_moves_from_commands(self, moves_file, axes=1, microsteps=1, steps_per_rev=200):
        """Execute moves froma file"""
        from time import time

        with open(moves_file) as f:
            moves = list(csv.reader(f))

        last_velocities = [0] * n_axes

        start_time = time();

        def cur_time():
            return round((time() - start_time), 4)

        def callback(proto, resp):
            print(cur_time(), resp)

        def pulse_per_rpm(rpm):
            """Convert RPM to pullses per sec"""
            return rpm * microsteps * steps_per_rev / 60

        with Proto(usb_native_port, callback=callback, baud=usb_baud) as proto:
            proto.reset()

            for seq, m in enumerate(moves):

                dt = float(m[0])

                v = [float(pulse_per_rpm(int(m[1])))]

                velocities = v*axes + [0]*(n_axes-axes)

                x = [.5 * (v0 + v1) * dt for v0, v1 in zip(last_velocities, velocities)]

                msg = proto.send_apos(dt, last_velocities, velocities, x)

                print(cur_time(), msg)

                last_velocities = velocities

            proto.wait_backlog(1)

    def test_speed(self):
        import saleae, time
        #s = saleae.Saleae()
        #s.capture_start()
        #time.sleep(.5)

        def callback(proto, resp):
            print(resp);

        last_velocities = [0] * n_axes
        with Proto(usb_native_port, callback=callback, baud=usb_baud) as proto:

            dt = 1
            for speed in [10000,-15000,20000,-25000,30000,-35000,40000, -45000, 50000]*1000:

                velocities = [speed]*n_axes

                x = [.5 * (v0 + v1) * dt for v0, v1 in zip(velocities, velocities)]

                msg = proto.send_apos(dt, velocities, velocities, x)
                print(msg)

                proto.wait_backlog(1)

    def test_speed_cycle(self):
        import saleae, time
        s = saleae.Saleae()
        s.capture_start()
        time.sleep(.5)

        def callback(proto, resp):
            print(resp)

        last_velocities = [0] * n_axes
        with Proto(usb_native_port, callback=callback, callforward=callback, baud=usb_baud) as proto:
            dt = 2
            for speed in [8000, 0, -8000, 0,6000,0,-6000,0] * 50:

                velocities = [speed] * n_axes

                x = [.5 * (v0 + v1) * dt for v0, v1 in zip(last_velocities, velocities)]

                proto.send_apos(dt,  last_velocities, velocities, x)

                last_velocities = velocities

                proto.wait_backlog(2)


    def test_runload(self):

        import saleae, time
        s = saleae.Saleae()
        s.capture_start()

        time.sleep(.5)

        def callback(proto, resp):
            print(resp);

        def callforward(proto, req):
            print(req)


        def pulse_per_rpm(rpm):
            """Convert RPM to pullses per sec"""
            microsteps = 8
            steps_per_rev = 200
            return rpm * microsteps * steps_per_rev / 60

        with Proto(usb_native_port, callback=callback, callforward=callforward, baud=usb_baud) as proto:
            proto.reset()

            dt = .2
            last_velocities =[80] * n_axes
            last_velocities = [80] + ([0]* (n_axes-1))

            pps = pulse_per_rpm(300)

            for speed in [1000]:

                velocities = [speed]*n_axes

                velocities = [speed] + ([0]* (n_axes-1))

                x = [.5 * (v0 + v1) * dt for v0, v1 in zip(last_velocities, velocities)]

                proto.send_apos(dt, last_velocities, velocities, x)

                last_velocities = velocities

            proto.wait_empty()

    def test_realtime(self):
        """Test sending messages in 'real-time', wiating until there are
        one or two messages in the queue before sending another message. """

        def callback(proto, resp):
            print(resp);

        def pulse_per_rpm(rpm):
            """Convert RPM to pullses per sec"""
            microsteps = 8
            steps_per_rev = 200
            return rpm * microsteps * steps_per_rev / 60

        with Proto(usb_native_port, callback=callback, callforward=callback, baud=usb_baud) as proto:
            proto.reset()

            dt = 1
            last_velocities =[80] * n_axes

            pps = pulse_per_rpm(300)

            for speed in [pps,pps,pps,pps,pps, 80]*10:

                velocities = [speed]*n_axes

                #velocities = [speed] + ([0]* (n_axes-1))

                x = [.5 * (v0 + v1) * dt for v0, v1 in zip(last_velocities, velocities)]

                proto.send_apos(dt, last_velocities, velocities, x)

                last_velocities = velocities

            proto.wait_empty()

    def test_short_moves(self):

        self.run_moves_from_commands('short_moves.csv', microsteps=10)

    def test_long_moves(self):

        self.test_lots_of_resets()

        for i in range(10000):
            print(f"======= {i} ======")
            self.run_moves_from_commands('long_moves.csv', microsteps=5, axes=1)

    def test_long_slow_moves(self):

        for i in range(10000):
            print(f"======= {i} ======")
            self.run_moves_from_commands('slow_long_moves.csv', microsteps=5, axes=1)


    def test_csv_moves(self):
        from os.path import join, dirname, abspath
        from operator import itemgetter
        d = dirname(dirname(abspath(__file__)))

        with open(join(d, 'test', 'recorded_moves.csv')) as f:
            moves = list(csv.DictReader(f))

        ig = itemgetter(*('axis0,axis1,axis2,axis3,axis4,axis5'.split(',')))
        n_axes = 6

        sl = SegmentList(n_axes, 750, 15000)

        for m in moves:
            sl.add_distance_segment([float(e) for e in ig(m)], v=float(m['v']))

        for i, s in enumerate(SegmentIterator(sl)):
            print(i, s)
            print(len(sl))

        return

        with Proto(usb_port) as proto:

            for i, s in enumerate(SegmentIterator(sl)):

                msg = Command(i, 10, int(s.t_seg * 1000000),
                              [int(sj.v0) for sj in s.joints],
                              [int(sj.v1) for sj in s.joints],
                              [int(sj.x) for sj in s.joints])

                proto.write(msg)
                print(msg)

                while proto.wait() > 4:
                    pass

    def test_velocity_command(self):

        sl = SegmentList(n_axes, 5000, 10000)

        m = 2000

        # for i in range(4):
        #    sl.add_velocity_segment([m,0,0,0,0,0], t=0.1)
        #    sl.add_velocity_segment([0,m,0,0,0,0], t=0.1)
        #    sl.add_velocity_segment([0,0,m,0,0,0], t=0.1)
        #    sl.add_velocity_segment([m,m,0,0,0,0], t=0.1)

        sl.add_velocity_segment([m, 0, 0, 0, 0, 0], t=1)
        sl.add_velocity_segment([m, 0, 0, 0, 0, 0], t=1)
        sl.add_velocity_segment([m, 0, 0, 0, 0, 0], t=1)

        print(sl)

        # for i, s in enumerate(SegmentIterator(sl)):
        #    print(i,s)

        start_time = time();

        def cur_time():
            return round((time() - start_time), 4)

        def callback(proto, resp):
            print(cur_time(), resp)

        with Proto(usb_native_port, timeout=10, callback=callback, baud=usb_baud) as proto:
            proto.reset()

            for i, s in enumerate(SegmentIterator(sl)):
                msg = proto.send_segment(s)

                print(msg)

            proto.wait_empty()

    ## **** HEY! ****
    ## Most of the other tests are broken ( July 2017 ) so, use this test as
    ## a model for how to write the control loop
    ##

    def dm_commands(self):

        sl = SegmentList(6, 15000, 60000)
        sl.add_velocity_segment([3000, 3000, 0, 0, 0, 0], t=.5)
        sl.add_velocity_segment([-3000, -3000, 0, 0, 0, 0], t=.5)

        si = SegmentIterator(sl)

        commands = []

        for i, s in enumerate(si):
            commands.append(Command(i, 10, int(s.t_seg * 1000000),
                                    [int(sj.v0) for sj in s.joints],
                                    [int(sj.v1) for sj in s.joints],
                                    [int(sj.x) for sj in s.joints]))

        for c in commands:
            yield (c)

    def test_direct_messages(self):

        commands = list(self.dm_commands())

        seq = 0
        with Proto(usb_port) as proto:

            while (commands):

                if proto.backlog < 5:

                    msg = commands[seq % len(commands)];
                    msg.seq = seq
                    seq += 1

                    proto.write(msg)
                    print(msg)
                else:
                    proto.read_next()
                    sleep(.05)

            proto.wait_backlog(0)

    def test_sim_direct_messages(self):

        commands = self.dm_messages()

        def prss(ss):
            print("{:6d} {:10.2f} {:6d} {:10.2f} {:10.2f}".format(ss.n, ss.tn, ss.xn, ss.vn, ss.cn))

        for cmd in commands:
            ss = SimSegment(cmd.v0[0], cmd.v1[0], x=cmd.steps[0])
            ss.run_out()
            prss(ss)

    def test_encoder(self):

        n_axes = 6

        nulls = [0] * (n_axes - 2)

        i = 0

        commands = [
            Command(0, 10, 2000000, [0] * 2 + nulls, [12000] * 2 + nulls, [2 ** 15] * 2 + nulls),
            Command(0, 10, 2000000, [12000] * 2 + nulls, [0] * 2 + nulls, [2 ** 15] * 2 + nulls),
            Command(0, 10, 2000000, [0] * 2 + nulls, [-12000] * 2 + nulls, [-2 ** 15] * 2 + nulls),
            Command(0, 10, 2000000, [-12000] * 2 + nulls, [0] * 2 + nulls, [-2 ** 15] * 2 + nulls),
        ]

        with Proto(usb_port) as proto:

            while (commands):

                if proto.backlog < 3:

                    msg = commands[i % 4]
                    msg.seq = i
                    i += 1

                    proto.write(msg)
                    print(msg)
                else:
                    proto.read_next()
                    sleep(.5)

            proto.wait_backlog(0)

    def test_linear_segments(self):

        sl = SegmentList(6, 15000, 1)
        sl.add_distance_segment([20000, 30000, 40000, 0, 0, 0], t=400)
        sl.add_distance_segment([20000, 30000, 40000, 0, 0, 0], t=400)
        sl.add_distance_segment([20000, 30000, 40000, 0, 0, 0], t=400)
        sl.add_distance_segment([20000, 30000, 40000, 0, 0, 0], t=400)

        print(sl)

        t_sum = 0
        x_sum = [0] * 6

        for ss in sl.iter_subsegments():
            t_sum += ss.t_seg
            x = [e.x for e in ss.joints]
            v = [e.v1 for e in ss.joints]
            x_sum = [xs + xi for xs, xi in zip(x_sum, x)]

        self.assertEquals(1600, round(t_sum, -1))
        self.assertEquals(80000, round(x_sum[0], -1))
        self.assertEquals(120000, round(x_sum[1], -1))

        sl = SegmentList(1, 15000, 1)
        sl.add_distance_segment([0], t=400)
        sl.add_distance_segment([0], t=400)

        for ss in sl.iter_subsegments():
            print(ss)

        sl = SegmentList(1, 15000, 1)
        sl.add_velocity_segment([0], t=400)
        sl.add_velocity_segment([0], t=400)

        for ss in sl.iter_subsegments():
            print(ss)

    def test_sub_segments(self):

        sl = SegmentList(1, 15000, 50000)
        sl.add_distance_segment([20000], t=.05)

        ss = list(SegmentIterator(sl))

        # Check that velocity of run sub segment is limited
        self.assertEquals(15000, ss[1].joints[0].v0)
        self.assertEquals(15000, ss[1].joints[0].v1)

        self.assertEquals(20000, round(ss[2].x[0], -2))
        self.assertEquals(0.0, ss[2].joints[0].v1)
        # Final time is longer than the commanded time
        self.assertEquals(1.63, round(ss[2].t, 2))

    def test_segments_queue(self):

        sl = SegmentList(1, 15000, 50000)

        for i in range(1, 5):
            sl.add_distance_segment([i * 10000], t=1)

        for i, ss in enumerate(SegmentIterator(sl)):
            print(ss.joints[0].v1, i, ss)

            if 4 < i < 8:
                sl.add_distance_segment([10000], t=1)

        self.assertEquals(130000, round(ss.x[0], 0))

    def test_complex_segments(self):

        n_axes = 1

        sl = SegmentList(n_axes, 15000, 1)
        sl.add_distance_segment([20000], t=400)

        print(sl)

        for s in sl.iter_subsegments():
            print(s)

        print("===========")

        sl = SegmentList(n_axes, 15000, 1)
        sl.add_distance_segment([20000], t=400)
        sl.add_distance_segment([-20000], t=400)

        print(sl)

        for s in sl.iter_subsegments():
            print(s)

        print("===========")

        sl = SegmentList(n_axes, 15000, 1)
        sl.add_distance_segment([-20000], t=400)
        sl.add_distance_segment([20000], t=400)

        print(sl)

        for s in sl.iter_subsegments():
            print(s)

    def test_sim_steps(self):

        def prss(ss):
            print(ss.n, ss.tn, ss.xn, ss.vn, ss.cn)

        def roundss(ss):
            return (ss.n, round(ss.tn, -4), ss.xn, round(ss.vn, -1), round(ss.cn, -4))

        #                   n      t       x     v1      cn
        self.assertEquals((500, 980000.0, 500, 1000.0, 0.0), roundss(SimSegment(0, 1000, 500).run_out()))
        self.assertEquals((500, 980000.0, 500, -1000.0, 0.0), roundss(SimSegment(0, -1000, 500).run_out()))
        self.assertEquals((0, 990000.0, 500, 30.0, 30000.0), roundss(SimSegment(1000, 0, 500).run_out()))
        self.assertEquals((0, 990000.0, 500, -30.0, 30000.0), roundss(SimSegment(-1000, 0, 500).run_out()))

        ss = SimSegment(0, 67, 6667)
        # for x in ss:
        #    prss(ss)

    def test_sim_params(self):

        ss = SimSegment(200, 300, t=.5)
        print(ss)
        print(ss.run_out())
        print('---')
        ss = SimSegment(300, 400, t=.5)
        print(ss)
        print(ss.run_out())

    def test_sim(self):

        sl = SegmentList(6, 40000, 15000)
        sl.add_distance_segment([26500, 0, 0, 0, 0, 0], t=2.66)

        for s in sl.iter_subsegments():
            x, v0, v1 = s.joints[0]
            a = float(v1 - v0) / float(s.t_seg)
            print("{} x={} v0={} v1={} a={}".format(s.t_seg, x, v0, v1, a))

        print("-----")

        s = SimSegment(10000, 0, 5000)
        for _ in s:
            print(s)

    def test_segment_buffer(self):

        sl = SegmentBuffer(1, 15000, 50000)

        def mkcn(ss):

            t = ss.t_seg

            return [(x_,) + SimSegment.initial_params(v_, a_) for x_, v_, a_ in
                    [(ssj.x, ssj.v0, (round(ssj.v1, 0) - round(ssj.v0, 0)) / t) for ssj in ss.joints]
                    ]

        for i in range(5):
            s = sl.add_velocity_segment([10000], t=1)
            if s:
                for ss in s:
                    print(len(sl), ss, mkcn(ss))

        for ss in sl.pop():
            print(len(sl), ss, mkcn(ss))

    def test_joy_buffer(self):

        sl = SegmentBuffer(4, 15000, 50000)

        def mkcn(ss):
            t = ss.t_seg

            return [(x_,) + SimSegment.initial_params(v_, a_) for x_, v_, a_ in
                    [(ssj.x, ssj.v0, (round(ssj.v1, 0) - round(ssj.v0, 0)) / t) for ssj in ss.joints]
                    ]

        last = time()
        for e in Joystick(.25):

            if last + .20 < time():
                dt = time() - last
                last = time()

                s = sl.add_velocity_segment(e, t=dt)
                if s:
                    for ss in s:
                        if ss.t_seg > 0:
                            print(mkcn(ss))

    def test_plot(self):
        import pandas as pd
        import matplotlib.pyplot as plt

        l = [(t / 1.e6, x, v) for t, x, v, cn, n in SimSegment(0, 1000, 500)]

        df = pd.DataFrame(l, columns='t x v'.split())

        print(df.head())

        df.plot(x='t', y='x')

        plt.show()

    def test_plot2(self):
        """Plot v vs t for a set of segments, both from the endpoints of the segments
        and a simulation of the step algorithm. """
        import pandas as pd
        import matplotlib.pyplot as plt

        sl = SegmentList(2, 5000, 10000)
        # sl.add_distance_segment([0, 0], t=1)
        sl.add_distance_segment([20000, 0], t=.01)
        sl.add_distance_segment([-20000, 0], t=.01)
        sl.add_distance_segment([20000, 0], t=.01)
        sl.add_distance_segment([-20000, 0], t=.01)

        t_sum = ti_sum = 0
        x_sum = xi_sum = 0
        l = [(0, 0, 0)]
        sims = []
        plot_joint = 0

        for ss in sl.iter_subsegments():
            t_sum += ss.t_seg

            joint = ss.joints[plot_joint]
            x = joint.x
            v0 = joint.v0
            v1 = joint.v1
            a = (v1 - v0) / ss.t_seg

            x_sum += x

            l.append((t_sum, x_sum, v1, a))

            def sign(x):
                return x / abs(x) if x else x

            direc = sign(x)  # sign(v1-v0) if v1 != v0 else sign(v1)

            for ti, xi, vi, cn, n, _ in SimSegment(v0, v1, int(x)):
                ts = ti_sum + (ti / 1.e6)
                xs = xi_sum + (xi * direc)
                # print (xi_sum, xs, xi)
                sims.append((ts, xs, vi * direc))

            ti_sum = ts
            xi_sum = xs

        df = pd.DataFrame(l, columns='t x v a'.split())
        df2 = pd.DataFrame(sims, columns='t xs vs'.split())

        fig, ax = plt.subplots(3, 1, sharex=True)
        df.plot(ax=ax[0], x='t', y='x')
        df2.plot(ax=ax[0], x='t', y='xs', linestyle=':')
        df.plot(ax=ax[1], x='t', y='v')
        df2.plot(ax=ax[1], x='t', y='vs', linestyle=':')
        df.plot(ax=ax[2], x='t', y='a')

        plt.show()


if __name__ == '__main__':
    unittest.main()
