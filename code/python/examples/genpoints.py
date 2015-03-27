import numpy as np
import math

# Generate N points on a circle centered at the origin.
def on_circle (r, N):
    return [ (r * math.cos(2 * k * math.pi / N),
              r * math.sin (2 * k * math.pi / N)) for k in range(N) ]

# Generate N points on an ellipse with major / minor axes (a, b) centered
# at the origin.
# Gaussian noise can be added.
def on_ellipse (a, b, N, sigma = 0):
    points = map(lambda (x, y): (a * x, b * y), on_circle(1, N))
    if sigma == 0:
        return points
    else:
        return points + list(np.random.normal(0, sigma, (N, 2)))

