import unittest

from trajectory.segments import Joint, SegmentList, Segment, JointSegment


def make_sl(moves, a_max=300_000):
    sl = SegmentList([Joint(10000, a_max), Joint(10000, a_max)])

    for move in moves:
        sl.add_distance_segment(move)

    return sl

class TestBasic(unittest.TestCase):

    def test_basic(self):

        jl = [Joint(5000, 1_000_000), Joint(1000, 1_000_000)]

        sl = SegmentList(jl)

        sl.add_distance_segment([10000, 10000])
        #sl.add_distance_segment([1000, 1000])
        # sl.add_distance_segment([500, 500])
        # sl.add_distance_segment([1000, 1000])

        diff = 0

        for i, seg in enumerate(sl):
            seg.update_t_acd()

        print(sl)

        for i, seg in enumerate(sl):
            diff += seg.update_boundary_velocities()

        print(sl)

        for i, seg in enumerate(sl):
            diff += seg.update_running_velocities()

        print(sl)

    def test_inconsistent_signs(self):
        """Test a case that triggers sub-section sign assertion"""



        def _do(a):
            sl = SegmentList([Joint(5000, a, a), Joint(5000, a, a)])
            sl.add_distance_segment([18000, 20000])
            sl.add_distance_segment([-100, 5000])
            sl.add_distance_segment([100, 10000])
            sl.add_distance_segment([-2500, 1000])
            sl.add_distance_segment([100, 1000])  ## ERROR! This velocity should not be negative
            sl.add_distance_segment([1000, 1000])  ## ERROR! This velocity should not be negative
            sl.update()
            df = sl.dataframe

        _do(300_000) # Succeedes
        _do(30_000) # Fails

    def test_dist_signs(self):
        sl = SegmentList([Joint(5000, 30_000, 30_000), Joint(5000, 30_000, 30_000)])
        sl.add_distance_segment([1000, 500])

        sg = list(sl.segments)[0]

        print(sg)
        print(sg.update_t_acd())
        print(sg)


    def test_inital_velocity(self):

        jl = [Joint(5000, 1_000_000), Joint(1000, 1_000_000)]

        sl = SegmentList(jl)

        sl.add_distance_segment([10000, 10000], v0=[500,500])

        # sl.add_distance_segment([500, 500])
        # sl.add_distance_segment([1000, 1000])

        sl.update()

        print(sl)

    def test_pop(self):

        jl = [Joint(5000, 1_000_000), Joint(1000, 1_000_000)]

        sl = SegmentList(jl)

        sl.add_distance_segment([10000, 10000])
        sl.add_distance_segment([500, 500])
        sl.add_distance_segment([1000, 1000])
        sl.add_distance_segment([-1000, -1000])
        sl.update()

        for ss in sl.sub_segments:
            print(ss)

        print(sl)
        print(len(list(sl.sub_segments)))
        print('---')

        extras = [ [500, 500], [1000, 1000], [500, 500], [1000, 1000] ]

        for ss in sl.yield_subsegments():

            print(ss)

            if extras:
                sl.add_distance_segment(extras.pop(0))
                sl.update()

        print('---')
        print(sl)


    def test_seg_length_sorting(self):
        """Test sorting joint segments by time required to execute them. """

        x = 10_000

        jl = [Joint(5000, 100_000, 200_000), Joint(2000, 100_000, 200_000), Joint(1000, 100_000, 200_000)]
        sl = SegmentList(jl)
        sl.add_distance_segment([10000, 10000, 10000])

        seg = list(sl.segments)[-1]
        seg.update_t_acd()

        self.assertEqual([2, 1, 0], [js.joint.n for js in seg.joint_segments_sorted])

        sl.add_distance_segment([10000, 1000, 100])

        seg = list(sl.segments)[-1]
        seg.update_t_acd()
        self.assertEqual([0, 1, 2], [js.joint.n for js in seg.joint_segments_sorted])

    def test_two_segment_updates_diff_velocity(self):
        x = 10_000

        jl = [Joint(5000, 100_000, 200_000), Joint(1000, 100_000, 200_000)]
        sl = SegmentList(jl)
        sl.add_distance_segment([10000, 10000])
        seg = list(sl.segments)[-1]

        for i in range(10):
            seg.update_t_acd()
            self.assertTrue(seg.validate())
            diff = seg.update_running_velocities()
            diff += seg.update_boundary_velocities()
            self.assertTrue(seg.validate())

            if diff < 5:
                break

        self.assertEqual(
            '|0.0100 10.1458 0.0050| [    5/0      9993@985        2/0    ]  [    5/0      9993@1000       2/0    ]',
            str(seg))


    def test_update(self):


        sl = SegmentList([Joint(10000, 300_000), Joint(10000, 300_0000)])
        sl.add_distance_segment([20000, 50])
        sl.add_distance_segment([10, 20000])
        sl.add_distance_segment([20000, 50])

        print(sl)


    def test_list_cpp(self):


        sl = SegmentList([Joint(10000, 300_000), Joint(10000, 300_0000)])
        sl.add_distance_segment([10000, 100])
        sl.add_distance_segment([10000, 100])

        print(sl)

    def test_large_rand_sequence(self):

        from random import randint

        def rand_segment_2(sl):  # Allows negative moves
            m = (randint(-joints[0].v_max, joints[0].v_max),
                 randint(-joints[1].v_max, joints[1].v_max),
                 randint(-joints[2].v_max, joints[2].v_max))
            sl.add_distance_segment(m)

        joints = [Joint(15000, 300_000, 300_000), Joint(10000, 300_000, 300_000), Joint(7500, 300_000, 300_000)]
        sl = SegmentList(joints)

        for i in range(500):
            rand_segment_2(sl)

if __name__ == '__main__':
    unittest.main()
