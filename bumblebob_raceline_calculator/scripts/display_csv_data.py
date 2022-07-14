import csv
from collections import defaultdict

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm
from matplotlib.colors import LinearSegmentedColormap, ListedColormap

columns = defaultdict(list)
with open("../doc/raceline_optimizer.csv") as f:
    reader = csv.reader(f)
    reader.next()
    for row in reader:
        for (i, v) in enumerate(row):
            columns[i].append(v)


step = [float(i) for i in columns[0]]
speed = [float(i) for i in columns[1]]
curv = [float(i) for i in columns[2]]
acceleration = [float(i) for i in columns[3]]
init_vel = [float(i) for i in columns[4]]
init_curv = [float(i) for i in columns[5]]
blueX = [float(i) for i in columns[6]]
blueY = [-float(i) for i in columns[7]]
yellX = [float(i) for i in columns[8]]
yellY = [-float(i) for i in columns[9]]
raceX = [float(i) for i in columns[10]]
raceY = [-float(i) for i in columns[11]]
refX = [float(i) for i in columns[12]]
refY = [-float(i) for i in columns[13]]


fig = plt.figure()
fig.suptitle("Velocity Profile")
plt.subplot(311)
plt.scatter(step, speed, s=0.5, c="blue")
plt.scatter(step, init_vel, s=0.5, c="red")
plt.ylabel("velocity in m/s")
plt.xlabel("step")

fig.suptitle("Curvature Profile")
plt.subplot(312)
plt.scatter(step, curv, s=0.5, c="blue")
plt.scatter(step, init_curv, s=0.5, c="red")
plt.ylabel("curvature in 1/m")
plt.xlabel("step")
plt.ylim(0, 0.3)

#
# fig.suptitle("Accel")
# plt.subplot(133)
# plt.scatter(step, acceleration, s=0.5, c="blue")

fig.suptitle("Raceline")
plt.subplot(313)
plt.plot(raceY, raceX, 'b')
plt.plot(refY, refX, 'r', alpha=0.5)
plt.scatter(blueY, blueX, s=2, c="blue")
plt.scatter(yellY, yellX, s=2, c="black")

plt.ylabel("x")
plt.xlabel("y")

plt.show()
