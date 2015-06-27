#! /usr/bin/env python

import numpy as np
import pylab as plt

def plot_values (filename_in, filename_out):
    vals = np.loadtxt(filename_in)
    plt.figure()
    i = 0
    while i < len(vals):
        plt.plot(i, vals[i], 'r+')
        i += 1
    plt.xlabel("iteration")
    plt.xlabel("energy")
    plt.savefig(filename_out)
    plt.close()

plot_values("area-dense.txt", "area-dense.png")
plot_values("area-sparse.txt", "area-sparse.png")
plot_values("perimeter-dense.txt", "perimeter-dense.png")
plot_values("perimeter-sparse.txt", "perimeter-sparse.png")

