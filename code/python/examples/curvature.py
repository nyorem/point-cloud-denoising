#!/ usr/bin/env python

import math
import numpy as np
import pylab as plt
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

# L2 norm
def normL2((x, y)):
    return math.sqrt(x * x + y * y)

# Absolute difference between 2 lists
def err(l1, l2):
    return [abs(x - y) for (x, y) in zip(l1, l2)]

def curvature(xp, xpp, yp, ypp, t):
    return (xp(t) * ypp(t) - yp(t) * xpp(t)) / math.pow(xp(t) * xp(t) + yp(t) * yp(t), 3/2);

def curvature_polar(rho, rhop, rhopp, t):
    return (rho(t) * rho(t) + 2 * rhop(t) * rhop(t) - rho(t) * rhopp(t)) / math.pow(rho(t) * rho(t) + rhop(t) * rhop(t), 3/2);

# Parameters
N = 1000
ts = [ (2 * k * math.pi) / N for k in range(N + 1) ]
radius = 0.5
weighted = False

def compute_polar(name, rho, rhop, rhopp):
    points_x = [rho(t) * math.cos(t) for t in ts]
    points_y = [rho(t) * math.sin(t) for t in ts]
    points = np.array(to_list_points(zip(points_x, points_y)))

    # Curvatures
    curvatures_expected = [curvature_polar(rho, rhop, rhopp, t) for t in ts ]
    curvatures_computed_vol = to_couples(unionballs.gradient_volume(points, radius, weighted).tolist(), normL2)
    curvatures_computed_per = to_couples(unionballs.gradient_perimeter_boundary(points, radius, weighted).tolist(), normL2)
    k = np.mean(curvatures_expected) / np.mean(curvatures_computed_per)
    curvatures_computed_per = [k * g for g in curvatures_computed_per]

    # Plots
    ## Curve
    plt.figure()
    plt.plot(points_x, points_y)
    plt.savefig(name + ".png")
    plt.close()

    ## Curvatures
    ### Volume
    plt.figure()
    plt.plot(ts, curvatures_expected, 'b')
    plt.plot(ts, curvatures_computed_vol, 'r')
    plt.savefig("curvatures_" + name + "_vol.png")
    plt.close()

    ### Perimeter
    plt.figure()
    plt.plot(ts, curvatures_expected, 'b')
    plt.plot(ts, curvatures_computed_per, 'r')
    plt.savefig("curvatures_" + name + "_per.png")
    plt.close()

    ## Errors
    ### Volume
    plt.figure()
    plt.plot(ts, err(curvatures_computed_vol, curvatures_expected))
    plt.savefig("err_" + name + "_vol.png")
    plt.close()

    ### Perimeter
    plt.figure()
    plt.plot(ts, err(curvatures_computed_per, curvatures_expected))
    plt.savefig("err_" + name + "_per.png")
    plt.close()

def compute(name, x, y, xp, yp, xpp, ypp):
    points_x = [x(t) for t in ts]
    points_y = [y(t) for t in ts]
    points = np.array(to_list_points(zip(points_x, points_y)))

    # Curvatures
    curvatures_expected = [curvature(xp, yp, xpp, ypp, t) for t in ts ]

    curvatures_computed_vol = to_couples(unionballs.gradient_volume(points, radius, weighted).tolist(), normL2)
    k = np.mean(curvatures_expected) / np.mean(curvatures_computed_vol)
    curvatures_computed_vol = [k * g for g in curvatures_computed_vol]

    curvatures_computed_per = to_couples(unionballs.gradient_perimeter_boundary(points, radius, weighted).tolist(), normL2)
    k = np.mean(curvatures_expected) / np.mean(curvatures_computed_per)
    curvatures_computed_per = [k * g for g in curvatures_computed_per]

    # Plots
    ## Curve
    plt.figure()
    plt.plot(points_x, points_y)
    plt.savefig(name + ".png")
    plt.close()

    ## Curvatures
    ### Volume
    plt.figure()
    plt.plot(ts, curvatures_expected, 'b')
    plt.plot(ts, curvatures_computed_vol, 'r')
    plt.savefig("curvatures_" + name + "_vol.png")
    plt.close()

    ### Perimeter
    plt.figure()
    plt.plot(ts, curvatures_expected, 'b')
    plt.plot(ts, curvatures_computed_per, 'r')
    plt.savefig("curvatures_" + name + "_per.png")
    plt.close()

    ## Errors
    ### Volume
    plt.figure()
    plt.plot(ts, err(curvatures_computed_vol, curvatures_expected))
    plt.savefig("err_" + name + "_vol.png")
    plt.close()

    ### Perimeter
    plt.figure()
    plt.plot(ts, err(curvatures_computed_per, curvatures_expected))
    plt.savefig("err_" + name + "_per.png")
    plt.close()

# Eight
xeight = lambda t: 0.5 * math.sin(t) * (math.cos(t) + 1)
yeight = lambda t: 0.5 * math.sin(t) * (math.cos(t) - 1)
xpeight = lambda t: 0.5 * (math.cos(2 * t) + math.cos(t))
ypeight = lambda t: 0.5 * (math.cos(2 * t) - math.cos(t))
xppeight = lambda t: -0.5 * (2 * math.sin(2 * t) + math.sin(t))
yppeight = lambda t: -0.5 * (2 * math.sin(2 * t) - math.sin(t))

compute("eight", xeight, yeight, xpeight, ypeight, xppeight, yppeight)

# Flower
rhoflower = lambda t: 0.5 * (1 + 0.5 * math.sin(6 * t + math.pi / 2))
rhopflower = lambda t: 1.5 * math.cos(6 * t + math.pi / 2)
rhoppflower = lambda t: -9 * math.sin(6 * t + math.pi / 2)

compute_polar("flower", rhoflower, rhopflower, rhoppflower)

