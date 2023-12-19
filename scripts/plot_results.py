import sys
import json
import numpy as np
from matplotlib import pyplot as plt

filename = sys.argv[1]

with open(filename, 'r') as f:
    data = json.load(f)

data = data['trajectory']
path = [elem['state'] for elem in data]
path = np.array(path)

ctrl = [elem['control'] for elem in data]
ctrl = np.array(ctrl)

obst_filename = sys.argv[2]
obst_list = []
with open(obst_filename, 'r') as file:
    for line in file:
        obst_list.append(list(map(float, line.split())))
        
plt.figure()
plt.plot(path[:,0], path[:,1])
for obst in obst_list:
    circle = plt.Circle(obst[:2], obst[2], color='k')
    plt.gca().add_patch(circle)

plt.title("Path")
plt.xlabel("X, m")
plt.ylabel("Y, m")
plt.savefig("path.png")

plt.figure()
plt.plot(ctrl)
plt.savefig("ctrl.png")