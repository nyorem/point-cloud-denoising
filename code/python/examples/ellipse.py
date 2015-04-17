#! /usr/bin/env python

import numpy as np
import pylab as plt
import math
import genpoints
import unionballs

# Transform a list of couples to a list
def to_list_points (lst):
    tmp = map(lambda (x, y): [x, y], lst)
    return [item for sublist in tmp for item in sublist]

# Transform a list to a list of couples
def to_couples(lst, f):
    ret = []
    i = 0
    while i < len(lst) - 1:
        x, y = lst[i], lst[i+1]
        ret.append(f((x, y)))
        i = i + 2
    return ret

# Absolute difference between 2 lists
def err(l1, l2):
    return [abs(x - y) for (x, y) in zip(l1, l2)]

# Curvature on an ellipse
def ellipse_curvature (a, b, t):
    return (a * b) / math.pow(a * a * math.sin(t) * math.sin(t) + b * b * math.cos(t) * math.cos(t), 3/2)

# L2 norm
def normL2((x, y)):
    return math.sqrt(x * x + y * y)

# Ellipse parameters
a = 1.5
b = 1
N = 200

# Algorithm parameters
radius = 30
weighted = False

# Expected curvatures
points = genpoints.on_ellipse(a, b, N);
pointsnp = np.array(to_list_points(points))
ts = [ (2 * k * math.pi) / N for k in range(N) ]
curvatures = map(lambda t: ellipse_curvature(a, b, t), ts)

# Computed curvatures
gradvol = to_couples(unionballs.gradient_volume(pointsnp, radius, weighted).tolist(), normL2)
gradper = to_couples(unionballs.gradient_perimeter_boundary(pointsnp, radius, weighted).tolist(), normL2)
k = np.mean(curvatures) / np.mean(gradper)
gradper = [k * g for g in gradper]

# Plot expected / computed curvatures
plt.figure()
plt.plot(ts, curvatures, 'b')
plt.plot(ts, gradvol, 'r')
plt.savefig("curvatures_vol.png")
plt.close()

plt.figure()
plt.plot(ts, curvatures, 'b')
plt.plot(ts, gradper, 'r')
plt.savefig("curvatures_per.png")
plt.close()

# Plot differences
plt.figure()
plt.plot(ts, err(gradper, curvatures))
plt.savefig("errper.png")
plt.close()

plt.figure()
plt.plot(ts, err(gradvol, curvatures))
plt.savefig("errvol.png")
plt.close()

