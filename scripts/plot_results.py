import sys, os
import json
import numpy as np
from matplotlib import pyplot as plt

def main():
    traj_filename = sys.argv[1]
    obst_filename = sys.argv[2]
    drawResults(traj_filename, obst_filename, "")

def drawResults(traj_file, obst_file, dirname=".", idx=0):
    with open(traj_file, 'r') as file:
        data = json.load(file)

    data = data['trajectory']
    path = [elem['state'] for elem in data]
    path = np.array(path)

    ctrl = [elem['control'] for elem in data]
    ctrl = np.array(ctrl)

    obst_list = []
    with open(obst_file, 'r') as file:
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
    plt.xlim([0, 10])  # FIXME
    plt.ylim([0, 10])
    plt.savefig(os.path.join(dirname, f"path_{format(idx, '04d')}.png"))
    plt.close()

    plt.figure()
    plt.plot(ctrl)
    plt.savefig(os.path.join(dirname, f"ctrl_{format(idx, '04d')}.png"))
    plt.close()

if __name__ == "__main__":
    main()