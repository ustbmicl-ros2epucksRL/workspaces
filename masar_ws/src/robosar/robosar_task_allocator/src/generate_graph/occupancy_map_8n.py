from tracemalloc import start
from .gridmap import OccupancyGridMap
import matplotlib.pyplot as plt
from .a_star import a_star
from .utils import plot_path
import os
from PIL import Image, ImageOps
import numpy as np

def define_point(node,gmap):
    node = gmap.get_index_from_coordinates(node[0], node[1])
    if gmap.is_occupied_idx(node):
        node = list(node)
        node[0] = node[0] + 1
        node[1] = node[1] + 1
        node = tuple(node)
        define_point(node,gmap)
    else:
        return node

def createGraph(n, nodes,gmap):
    adj = np.zeros((n, n))-1
    nodes = np.array(nodes)
    for i in range(n):
        for j in range(i+1,n):
            start_n = (nodes[i,0], nodes[i,1])
            stop_n = (nodes[j,0], nodes[j,1])
            path,path_px,cost = a_star(start_n, stop_n, gmap, movement='8N')

            if path:
                adj[i, j] = cost
                adj[j, i] = cost

            # gmap.plot()
            # if path:
            #     # plot resulting path in pixels over the map
            #     plot_path(path_px)
            # #     print("hi")
            #     # plot start and goal points over the map (in pixels)
            #     start_node_px = gmap.get_index_from_coordinates(nodes[i][0], nodes[i][1])
            #     goal_node_px = gmap.get_index_from_coordinates(nodes[j][0], nodes[j][1])
            #     plt.plot(start_node_px[0], start_node_px[1], 'ro')
            #     plt.plot(goal_node_px[0], goal_node_px[1], 'go')

            gmap.visited = np.zeros(gmap.dim_cells, dtype=np.float32)

    # plt.show()
    return adj

def set_of_nodes(gmap,num_nodes):
    start_node = np.zeros([num_nodes,2])
    start_node[0,:] = [65.0,40.0]
    for i in range(1,3):
        temp=tuple(start_node[i-1,:])
        start_node[i] = list(define_point((temp[0],temp[1]+5),gmap))
    return tuple(start_node)

if __name__ == '__main__':
    num_nodes = 9
    # load the map
    filename = 'maps/localization_map_vicon.pgm'
    new_file = "{}.png".format(filename)
    with Image.open(filename) as im:
        im = ImageOps.invert(im)
        im.save(new_file)
    gmap = OccupancyGridMap.from_png(new_file, 1)
    nodes = ((65,40),(65,60),(70,60),(80,40),(90,40),(100,40),(90,50),(95,55),(100,60))
    adj = createGraph(num_nodes,nodes,gmap)