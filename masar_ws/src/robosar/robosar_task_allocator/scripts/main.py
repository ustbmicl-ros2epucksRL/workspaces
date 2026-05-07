"""
Main script to be executed. Used for testing without ROS.
"""

import random
import os
import rospkg
import numpy as np
import matplotlib.pyplot as plt
from task_allocator.TA import *
import task_allocator.utils as utils
from task_allocator.Environment import Environment
from task_allocator.Simulation import Simulation

# Random graph
def distance(c1, c2):
    diff = (c1[0] - c2[0], c1[1] - c2[1])
    return np.sqrt(diff[0] * diff[0] + diff[1] * diff[1])

def create_random_graph(n, env_size):
    nodes = []
    adj = np.zeros((n, n))
    for i in range(n):
        nodes.append([random.uniform(0,1)*env_size, random.uniform(0,1)*env_size])
    nodes = np.array(nodes)
    for i in range(n):
        for j in range(i,n):
            adj[i, j] = distance(nodes[i,:], nodes[j,:])
            adj[j, i] = adj[i,j]

    return nodes, adj

if __name__ == '__main__':
    # Create graph
    # n = 40
    make_graph = False
    downsample = 5
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('robosar_task_allocator')
    map_path = rospack.get_path('robosar_navigation')
    nodes = np.load(os.path.join(package_path, "src/saved_graphs/scott_SVD_points.npy"))
    filename = os.path.join(map_path, 'maps/scott_hall_SVD.pgm')
    if make_graph:
        print('creating graph')
        adj = utils.create_graph_from_file(filename, nodes, len(nodes), downsample, False)
        print('done')

    utils.plot_pgm(filename)

    # Create environment
    adj = np.load(os.path.join(package_path, 'src/saved_graphs/scott_SVD_graph.npy'))
    n = len(nodes)
    env = Environment(nodes[:n,:], adj)

    # Create robots
    id_list = [0,1,2]
    for id in id_list:
        env.add_robot("robot_"+str(id), id)

    # Plotting
    colors = ['r', 'b', 'm', 'c', 'y', 'g', 'k']
    plt.plot(nodes[:n,0], nodes[:n,1], 'ko', zorder=100)
    for idx, r in enumerate(env.robots.values()):
        plt.plot([r.pos_prev[0], r.pos[0]], [r.pos_prev[1], r.pos[1]], colors[idx] + 'o')

    solver = TA_mTSP()
    solver.init(env, 5)

    sim = Simulation(env, solver, 1, 300, colors)
    robot_paths = sim.simulate()

    for i, path in enumerate(robot_paths):
        print("Robot {} path: {}".format(i, path))

    plt.show()
