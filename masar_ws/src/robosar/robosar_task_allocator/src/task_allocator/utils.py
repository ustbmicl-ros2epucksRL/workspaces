"""
Utils
- plotting map
- pixels to meters conversion
- creating graph
"""

import cv2
import matplotlib.pyplot as plt
import numpy as np
import skimage.measure

# from generate_graph import occupancy_map_8n
# from generate_graph.gridmap import OccupancyGridMap
from generate_graph.gridmap import OccupancyGridMap
from generate_graph import occupancy_map_8n


def plot_pgm(filename):
    im = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
    im = cv2.flip(im, 0)
    plt.imshow(
        im, cmap="gray", vmin=0, vmax=255, origin="lower", interpolation="none", alpha=1
    )
    plt.draw()


def plot_pgm_data(data):
    data[data == -1] = 20
    plt.imshow(
        100 - data,
        cmap="gray",
        vmin=0,
        vmax=100,
        origin="lower",
        interpolation="none",
        alpha=1,
    )
    plt.draw()


def pixels_to_m(pixels, scale, origin):
    return [(pixels[0]) * scale + origin[0], (pixels[1]) * scale + origin[1]]


def m_to_pixels(m, scale, origin):
    return [
        int(np.floor((m[0] - origin[0]) / scale)),
        int(np.floor((m[1] - origin[1]) / scale)),
    ]


def create_graph_from_file(filename, nodes, n, downsample=1, save=False):
    im = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
    dim = ((im.shape[1] + 1) // downsample, (im.shape[0] + 1) // downsample)
    im = cv2.resize(im, dim, interpolation=cv2.INTER_AREA)
    new_file = "{}_downsample.png".format(filename)
    cv2.imwrite(new_file, im)
    gmap = OccupancyGridMap.from_png(new_file, 1)
    nodes_flip = (np.flip(nodes, axis=1) / downsample).tolist()
    adj = occupancy_map_8n.createGraph(n, nodes_flip, gmap) * downsample
    if save:
        np.save("saved_graphs/custom_{}_graph.npy".format(n), adj[:n, :n])
    return adj


def create_graph_from_data(data, nodes, n, downsample=1, save=False):
    resized_image = skimage.measure.block_reduce(data, (downsample, downsample), np.max)
    gmap = OccupancyGridMap.from_data(resized_image)
    nodes_flip = (np.flip(nodes, axis=1) / downsample).tolist()
    adj = occupancy_map_8n.createGraph(n, nodes_flip, gmap) * downsample
    if save:
        np.save("saved_graphs/custom_{}_graph.npy".format(n), adj[:n, :n])
    return adj
