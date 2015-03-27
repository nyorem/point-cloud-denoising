#! /usr/bin/env python

import unionballs
import numpy as np
import pylab as plt
import os
import genpoints

# Transform a list of couples to a list
def to_list_points (lst):
    tmp = map(lambda (x, y): [x, y], lst)
    return [item for sublist in tmp for item in sublist]

# points = np.array([0.0, 0.0, 0.5, 0.0, 0.0, 0.5, 0.2, 0.2])
# points = np.array(to_list_points(genpoints.on_circle(10, 100)))
points = np.array(to_list_points(list(genpoints.on_ellipse(10, 20, 100))))

# Parameters
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

def step_gradient_descent_perimeter (points, radius,timestep, weighted = False):
    grad = unionballs.gradient_perimeter_boundary(points, radius, weighted)
    return points - timestep * grad

def step_gradient_descent_volume (points, radius, timestep, weighted = False):
    grad = unionballs.gradient_volume(points, radius, weighted)
    return points - timestep * grad

def plot_points (points, filename, iter):
    plt.figure()
    i = 0
    while i < len(points):
        plt.plot(points[i], points[i + 1], 'r+')
        i += 2
    plt.axis([xmin - var, xmax + var, ymin - var, ymax + var])
    str = "i = " + `iter`
    print(str)
    plt.text((xmax - xmin)/ 2 - var, -(ymax - ymin) / 2 + var, str)
    # plt.text(0, 0, str)
    plt.savefig(filename)
    plt.close()

def gradient_descent_perimeter (points, radius, timestep, fname, weighted = False):
    print(fname)
    os.system("rm -f " + fname + "*.png")
    ps = points
    for i in range(0, iter):
        filename = fname + `i` + ".png"
        plot_points(ps, filename, i)
        ps = step_gradient_descent_perimeter(ps, radius, timestep, weighted)
    l = len(fname) + 1
    os.system("convert -delay 50 -loop 0 `ls " + fname + "*.png | sort -k1." + `l` + "n` " + fname + ".gif")

def gradient_descent_volume (points, radius, timestep, fname, weighted = False):
    print(fname)
    os.system("rm -f " + fname + "*.png")
    ps = points
    for i in range(0, iter):
        filename = fname + `i` + ".png"
        plot_points(ps, filename, i)
        ps = step_gradient_descent_volume(ps, radius, timestep, weighted)
    l = len(fname) + 1
    os.system("convert -delay 50 -loop 0 `ls " + fname + "*.png | sort -k1." + `l` + "n` " + fname + ".gif")

gradient_descent_perimeter(points, radius, timestep, "perimeter")
gradient_descent_perimeter(points, radius, timestep, "perimeter-weighted", True)
gradient_descent_volume(points, radius, timestep, "volume")
gradient_descent_volume(points, radius, timestep, "volume-weighted", True)

