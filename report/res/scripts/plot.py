#! /usr/bin/env python

import numpy as np
import pylab as plt
import os
import re
import glob

# List of centers file
files = glob.glob("centers*.txt")

# Find min/max x and y
x = []
y = []

for f in files:
    pos = np.loadtxt(f)
    for p in pos:
        x.append(p[0])
        y.append(-p[1])
xmin, xmax = min(x), max(x)
ymin, ymax = min(y), max(y)

# Plot each list of positions
for f in files:
    pos = np.loadtxt(f)
    plt.figure()
    for p in pos:
        plt.plot(p[0], -p[1], 'r+')
    filename = os.path.splitext(f)[0] + ".png"
    plt.axis([xmin - 10, xmax + 10, ymin - 10, ymax + 10])
    i = re.findall(r'\d+', f)[0]
    iter = "i = " + `i`
    plt.text(xmax - 100, ymin + 10, iter)
    plt.savefig(filename)
    plt.close()

# Convert to a gif
# Sort numerically before doing the conversion
fname = "centers"
outputgif = "ellipse"
l = len(fname) + 1
os.system("convert -delay 50 -loop 0 `ls " + fname + "*.png | sort -k1." + `l` + "n` " + outputgif + ".gif")

