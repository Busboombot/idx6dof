import unittest

from trajectory.segments import Joint, SegmentList, Segment, JointSegment
from trajectory.segments import SimSegment
from trajectory import *
from trajectory.plot import *

class TestBasic(unittest.TestCase):

    def test_basic(self):


        sl = SegmentList([Joint(5000, 50_000, 50_000), Joint(5000, 50_000, 50_000)])
        sl.add_distance_segment([1000, 1000])
        sl.add_distance_segment([500, 1000])
        sl.add_distance_segment([500, 1000])
        sl.add_distance_segment([1000, 1000])

        sl.update()

        for e in sl.sub_segments:
            print(e)

        plot_segment_list(sl.dataframe)

if __name__ == '__main__':
    unittest.main()
