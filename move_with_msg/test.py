
from planning import JointVector, MotionPlanner

ap = JointVector(axis=0, x0=0, v0=10, x1=100, v1=30)

print ap.distance

p = ap.amax_split(4,30)
print  p
assert p[0] == 5
assert p[1] == 30
assert p[2] == 100
assert p[3] == 0

p = ap.amax_split(4,20)
print  p
assert p[0] == 5.625
assert p[1] == 20
assert p[2] == 37.5
assert p[3] == 62.5


null_ax = JointVector(x0=0,v0=0,x1=0,v1=0)

mp = MotionPlanner(30,5);


