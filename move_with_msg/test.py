
from planning import JointVector, MotionPlanner, TrajectoryPoint
from tabulate import tabulate

mp = MotionPlanner(100000,500000, 1000000)
mp.add_point(TrajectoryPoint([1000], .1))
mp.add_point(TrajectoryPoint([2000], .1))
mp.add_point(TrajectoryPoint([4000], .1))
mp.add_point(TrajectoryPoint([0], .1))


print "Positions"
print tabulate(  [ p.info_row() for p in mp.points] , headers = "T V L J0 J1 J2 J3 J4 J5".split())

print "\nVelocities"
print tabulate( mp.velocities, headers = "J0 J1 J2 J3 J4 J5".split())

print "\nsplit_velocities"
p
print tabulate( mp.split_velocities, headers = "J0 J1 J2 J3 J4 J5".split())

