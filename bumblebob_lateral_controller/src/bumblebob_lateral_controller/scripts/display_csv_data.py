import csv
from collections import defaultdict
import argparse

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm
from matplotlib.colors import LinearSegmentedColormap, ListedColormap

x = []
y = []
car_heading = []
road_heading = []
feedback_angle = []
feedforward_angle = []
steering_cmd = []
radius = []
lateral_error = []
heading_error = []
angular_velocity = []

raceline_x = []
raceline_y = []

data_path = ""

"""-------Parse Arguments-------"""
parser = argparse.ArgumentParser(description="Add the path to the data file")
parser.add_argument(
    "data_path",
    help="""Enter the path to your data file generated by the controller node
    or an empty string to use the example files""")
parser.add_argument(
    "reference_path",
    help="""Enter the path to your reference raceline file
    or an empty string to use the example files""")
args = parser.parse_args()

if args.data_path is "":
    reference_path = "race.csv"
else:
    reference_path = args.reference_path

if args.data_path is "":
    data_path = "kinematic_controller.csv"
else:
    data_path = args.data_path


columns = defaultdict(list)
with open(data_path) as f:
    reader = csv.reader(f)
    reader.next()
    for row in reader:
        for (i, v) in enumerate(row):
            columns[i].append(v)

columns_race = defaultdict(list)
with open(reference_path) as f:
    reader = csv.reader(f)
    reader.next()
    for row in reader:
        for (i, v) in enumerate(row):
            columns_race[i].append(v)

# switch from string to float
x = [float(i) for i in columns[0]]
y = [float(i) for i in columns[1]]
car_heading = [float(i) for i in columns[2]]
road_heading = [float(i) for i in columns[3]]
feedback_angle = [float(i) for i in columns[4]]
feedforward_angle = [float(i) for i in columns[5]]
steering_cmd = [float(i) for i in columns[6]]
radius = [float(i) for i in columns[7]]
lateral_error = [float(i) for i in columns[8]]
heading_error = [float(i) for i in columns[9]]
angular_velocity = [float(i) for i in columns[10]]


raceline_x = [float(i) for i in columns_race[0]]
raceline_y = [float(i) for i in columns_race[1]]

timestep = np.linspace(0, len(columns[0]), len(columns[0]))

print(len(timestep))
print(len(heading_error))
# calculate means
abs_lateral_error = [abs(i) for i in lateral_error]
summe = 0
for i in abs_lateral_error:
    if i < 0.3:
        summe += i

print("Average lateral error: ", summe / len(abs_lateral_error))


# color map
viridis = cm.get_cmap('viridis', len(columns[0]))
norm = plt.Normalize(0, len(columns[0]))


fig = plt.figure()
fig.suptitle("k_lateral = 1.0, k_head = 1.2, k_angular = 0.3")
plt.subplot(221)
plt.scatter(x, y, c=timestep, cmap=viridis, norm=norm, s=2)
# plt.scatter(raceline_x, raceline_y, s=0.5, c="grey")
plt.ylabel("y")
plt.xlabel("x")
plt.title("Track and driven course")


plt.subplot(222)
plt.scatter(timestep / 20, lateral_error, c=timestep,
            cmap=viridis, norm=norm, s=2)
plt.ylabel("lateral-error[m]")
plt.xlabel("timestep")
plt.title("Lateral-error")

plt.subplot(223)
plt.scatter(timestep / 20, heading_error, c=timestep,
            cmap=viridis, norm=norm, s=2)

plt.ylabel("heading-error [rad]")
plt.xlabel("timestep")
plt.title("Heading-error")

plt.subplot(224)
plt.scatter(timestep/20, angular_velocity, c=timestep,
            cmap=viridis, norm=norm, s=2)
plt.legend
plt.ylabel("angular velocity [rad/s]")
plt.xlabel("timestep")
plt.title("Angular velocity")
plt.show()
