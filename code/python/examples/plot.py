#! /usr/bin/env python

import unionballs
import numpy as np
import pylab as plt
import os
import genpoints

def to_list_points (lst):
    tmp = map(lambda (x, y): [x, y], lst)
    return [item for sublist in tmp for item in sublist]

# points = np.array([0.0, 0.0, 0.5, 0.0, 0.0, 0.5, 0.2, 0.2])
# points = np.array(to_list_points(genpoints.on_circle(10, 100)))
points = np.array(to_list_points(list(genpoints.on_ellipse(10, 20, 100))))
radius = 1
iter = 10
timestep = 0.1

def find_min_max (points):
    i = 0
    x = []
    y = []
    while i < len(points):
        x.append(points[i])
        y.append(points[i+1])
        i += 2
    return min(x), max(x), min(y), max(y)

xmin, xmax, ymin, ymax = find_min_max(points)
var = 1

def step_gradient_descent_perimeter (points, radius, timestep):
    grad = unionballs.gradient_perimeter_boundary(points, radius)
    return points - timestep * grad

def step_gradient_descent_volume (points, radius, timestep):
    grad = unionballs.gradient_volume(points, radius)
    return points - timestep * grad

def plot_points (points, filename):
    plt.figure()
    i = 0
    while i < len(points):
        plt.plot(points[i], points[i + 1], 'r+')
        i += 2
    plt.axis([xmin - var, xmax + var, ymin - var, ymax + var])
    plt.savefig(filename)
    plt.close()

def gradient_descent_perimeter (points, radius, timestep):
    os.system("rm -f perimeter*.png")
    ps = points
    for i in range(0, iter):
        filename = "perimeter" + `i` + ".png"
        plot_points(ps, filename)
        ps = step_gradient_descent_perimeter(ps, radius, timestep)
    os.system("convert -delay 50 -loop 0 `ls perimeter*.png | sort -k1.10n` perimeter.gif")

def gradient_descent_volume (points, radius, timestep):
    os.system("rm -f volume*.png")
    ps = points
    for i in range(0, iter):
        filename = "volume" + `i` + ".png"
        plot_points(ps, filename)
        ps = step_gradient_descent_volume(ps, radius, timestep)
    os.system("convert -delay 50 -loop 0 `ls volume*.png | sort -k1.7n` volume.gif")

gradient_descent_perimeter(points, radius, timestep)
gradient_descent_volume(points, radius, timestep)

