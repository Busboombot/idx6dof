from math import pi, sin
import pandas as pd
import numpy as np
from tabulate import tabulate

rows = []
rng = 32
for i in range(rng):
    step =  float(i) / float(rng)
    acc = 0
    row = []
    for j in range(rng):
        acc += step
        if acc >= 1.0:
            row.append(1)
            acc -= 1.0
        else:
            row.append(0)
            
        
            
    rows.append(row)
     
print "uint8_t step_patterns[32][32] = { "
for i, row in enumerate(rows):
    print "{%s}, // %d" % (','.join(str(e) for e in row),i)
print "};"
        