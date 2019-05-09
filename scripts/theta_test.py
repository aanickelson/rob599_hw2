from random import uniform
from math import pi
for i in range(20):
    theta = pi * uniform(-1.9, 1.9)
    print("OG theta", theta)

    if theta > pi:
        theta = theta - 2 * pi
    elif theta < -pi:
        theta = 2 * pi + theta

    print("new theta", theta)
    print("##############")