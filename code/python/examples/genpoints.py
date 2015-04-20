import numpy as np
import math

# Sum two tuples element wise
def sum_tuple (x, y):
    return tuple(sum(z) for z in zip(x, y))

# Generate N points on a circle centered at the origin.
def on_circle (r, N):
    return [ (r * math.cos(2 * k * math.pi / N),
              r * math.sin(2 * k * math.pi / N)) for k in range(N) ]

# Generate N points on an ellipse with major / minor axes (a, b) centered
# at the origin.
# Gaussian noise can be added.
def on_ellipse (a, b, N, sigma = 0):
    points = map(lambda (x, y): (a * x, b * y), on_circle(1, N))
    if sigma == 0:
        return points
    else:
        noise = map(tuple, np.random.normal(0, sigma, (N, 2)).tolist())
        return [sum_tuple(x, y) for (x, y) in zip(points, noise) ]

