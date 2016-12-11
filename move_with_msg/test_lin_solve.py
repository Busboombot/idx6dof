import numpy as np

a = 1
d = 1
v0 = 150
v1 = 150

a = np.array([[0,-a,0],[0,0,-d],[1,-1,-1]])
b = np.array([v0,v1,0])

print np.linalg.solve(a, b)
