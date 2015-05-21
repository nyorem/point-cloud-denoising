#! /usr/bin/env python

# Do the gradient descent

import unionballs
import numpy as np
import pylab as plt
import os
import genpoints

# Transform a list of couples to a list
def to_list_points (lst):
    tmp = map(lambda (x, y): [x, y], lst)
    return [item for sublist in tmp for item in sublist]

def value(v):
    return v.sum() / 2

# List of points to consider
points = np.array(to_list_points(genpoints.on_ellipse(3.5, 1, 100, 0)))

# Parameters
radius = 0.3
iter = 50
timestep_vol = 0.1
timestep_per = 0.05
timestep_weighted_vol = 0.05
timestep_weighted_per = 0.0001

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
    str = "i = " + `iter + 1`
    print(str)
    plt.text((xmax - xmin)/ 2 - var, -(ymax - ymin) / 2 + var, str)
    plt.savefig(filename)
    plt.close()

def plot_values (vals, filename):
    plt.figure()
    i = 0
    while i < len(vals):
        plt.plot(i, vals[i], 'r+')
        i += 1
    plt.savefig(filename)
    plt.close()

def gradient_descent_perimeter (points, radius, timestep, fname, weighted = False):
    print(fname)
    os.system("rm -f " + fname + "*.png")
    ps = points
    vals = []
    for i in range(0, iter):
        filename = fname + `i` + ".png"
        plot_points(ps, filename, i)
        ps = step_gradient_descent_perimeter(ps, radius, timestep, weighted)
        vals.append(value(unionballs.volume(ps, radius)))
    l = len(fname) + 1
    os.system("convert -delay 50 -loop 0 `ls " + fname + "*.png | sort -k1." + `l` + "n` " + fname + ".gif")
    plot_values(vals, fname + "_values.png")

def gradient_descent_volume (points, radius, timestep, fname, weighted = False):
    print(fname)
    os.system("rm -f " + fname + "*.png")
    ps = points
    vals = []
    for i in range(0, iter):
        filename = fname + `i` + ".png"
        plot_points(ps, filename, i)
        vals.append(value(unionballs.volume(ps, radius)))
        ps = step_gradient_descent_volume(ps, radius, timestep, weighted)
    l = len(fname) + 1
    os.system("convert -delay 50 -loop 0 `ls " + fname + "*.png | sort -k1." + `l` + "n` " + fname + ".gif")
    plot_values(vals, fname + "_values.png")

gradient_descent_volume(points, radius, timestep_vol, "volume")
# gradient_descent_perimeter(points, radius, timestep_per, "perimeter")
# gradient_descent_volume(points, radius, timestep_weighted_vol, "volume-weighted", True)
# gradient_descent_perimeter(points, radius, timestep_weighted_per, "perimeter-weighted", True)

