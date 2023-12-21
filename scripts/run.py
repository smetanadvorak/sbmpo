import numpy as np
import subprocess as sp
import os, shutil
import json

from plot_results import drawResults

configs_dir = "./configs"
results_dir = "./results"
shutil.rmtree(configs_dir, ignore_errors=True)
shutil.rmtree(results_dir, ignore_errors=True)

exec_path = "../bin/dev/sbmpo"
orig_config = "../resources/kinematic/config.json"

os.makedirs(configs_dir, exist_ok=True)
os.makedirs(results_dir, exist_ok=True)

# open config json
with open(orig_config, 'r') as config_file:
    config = json.load(config_file)

n_iter = 100
dT = 0.5
n_control_steps_consume = 1
time_consume = dT * n_control_steps_consume

obst = np.array([
    [1.0, 2.0, 0.25],
    [4.0, 4.0, 0.25],
    [6.0, 4.0, 0.25],
    [8.0, 10.0, 0.25]
    ])
obst_vel = np.array([[0.1, 0.0, 0.0]])

start = np.array([0.0, 0.0, 0.0])
current = np.copy(start)
goal = np.array([10.0, 10.0, 0.0])

for iter_idx in range(n_iter):
    obstacle_filename = os.path.join(configs_dir, f"obstacles_{iter_idx}.txt")
    config_filename = os.path.join(configs_dir, f"config_{iter_idx}.json")
    result_filename = os.path.join(results_dir, f"results_{iter_idx}.json")
    np.savetxt(obstacle_filename, obst)
    
    # setup config for planner
    config["start"] = list(current)
    config["goal"] = list(goal)
    config["sampling time"] = dT
    config["model"]["obstacle file"] = obstacle_filename
    with open(config_filename, 'w') as config_file:
        json.dump(config, config_file, indent=4)

    # run the planner
    command = f"{exec_path} {config_filename} --output={result_filename}"
    print(command)
    print(config_filename)
    print(result_filename)
    sp.run([exec_path, config_filename, f"--output={result_filename}", "-q"])

    # draw results
    drawResults(result_filename, obstacle_filename, results_dir, iter_idx)

    # move the robot
    with open(result_filename, 'r') as f:
        data = json.load(f)
        data = data['trajectory']
    path = [elem['state'] for elem in data]
     
    print(len(path))
    if (len(path) - 1 < n_control_steps_consume):
        break
    current = np.array(path[n_control_steps_consume])

    # move obstacle
    obst += obst_vel * dT

os.system("convert -delay 10 -loop 0 results/path_* animation.gif")