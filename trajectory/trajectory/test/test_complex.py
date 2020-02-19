import unittest

from trajectory.segments import Joint, SegmentList, Segment, JointSegment
import pandas as pd

class TestComplex(unittest.TestCase):

    def test_basic(self):
        import numpy as np

        sl = SegmentList([Joint(10000, 300_000, 300_000), Joint(10000, 300_000, 300_000)])
        sl.add_distance_segment([5.0,-200.0])
        sl.add_distance_segment([5.0,-200.0])
        sl.add_distance_segment([-5.0,-200.0])
        sl.add_distance_segment([1000, 1000])
        sl.add_distance_segment([1000, -1000])
        sl.add_distance_segment([0, -1000])
        sl.add_distance_segment([-1000, -1000])
        print(sl.debug_str())

        #sl.new_update()


    def test_long_ad(self):
        """Error: Acceleration periods are longer than segment"""
        sl = SegmentList([Joint(10000, 300_000, 300_000), Joint(10000, 300_000, 300_000)])
        sl.add_distance_segment([5.0, -100.0])
        sl.add_distance_segment([5.0, -100.0])
        sl.add_distance_segment([-5.0, -100.0])

        sl.update()
        sl.validate()

    def test_long_ad_2(self):
        """Error: Acceleration periods are longer than segment"""
        sl = SegmentList([Joint(10000, 300_000, 300_000), Joint(10000, 300_000, 300_000), Joint(10000, 300_000, 300_000)])
        sl.add_distance_segment([5000,10000,20000])
        sl.add_distance_segment([5000,10000,20000])
        sl.add_distance_segment([5000,10000,20000])

        sl.update()
        sl.validate()

        for e in sl.sub_segments:
            print(e[0].x, e[0].v_i)


if __name__ == '__main__':
    unittest.main()
