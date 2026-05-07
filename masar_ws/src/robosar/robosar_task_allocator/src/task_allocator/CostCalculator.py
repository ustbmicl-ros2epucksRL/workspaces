#!/usr/bin/env python3

import numpy as np
import task_allocator.utils as utils
from generate_graph.a_star import a_star
from generate_graph.gridmap import OccupancyGridMap

from robosar_task_generator.functions import check_edge_collision


class CostCalculator:
    def __init__(self, utility_range, downsample) -> None:
        self.utility_range = utility_range
        self.gmap = None  # type: OccupancyGridMap
        self.map_msg = None
        self.scale = None
        self.origin = None
        self.downsample = downsample

    def update_map_data(self, gmap, map_msg):
        self.gmap = gmap
        self.map_msg = map_msg
        self.scale = map_msg.info.resolution
        self.origin = [map_msg.info.origin.position.x, map_msg.info.origin.position.y]

    def utility_discount_fn(self, goal_pos, node_pos):
        p = 0.0
        dist = np.linalg.norm([goal_pos[0] - node_pos[0], goal_pos[1] - node_pos[1]])
        # within utility range and not separated by obstacle
        if dist < self.utility_range and check_edge_collision(goal_pos, node_pos, self.map_msg) != 0:
            p = 1.0 - (dist/self.utility_range)**2
        return p

    def a_star_cost(self, start, goal, max_it=5000):
        start = utils.m_to_pixels(start, self.scale, self.origin)
        goal = utils.m_to_pixels(goal, self.scale, self.origin)
        start_flip = [start[1] / self.downsample, start[0] / self.downsample]
        goal_flip = [goal[1] / self.downsample, goal[0] / self.downsample]
        path, _, cost = a_star(start_flip, goal_flip, self.gmap, movement="8N", max_it=max_it)
        self.gmap.visited = np.zeros(self.gmap.dim_cells, dtype=np.float32)
        if len(path) == 0:
            return cost * self.scale * self.downsample, False
        return cost * self.scale * self.downsample, True

    def obstacle_cost(self, node, r):
        pix_node = utils.m_to_pixels(node, self.scale, self.origin)
        pix_range = int(r / self.scale)
        x_min = int(max(pix_node[0] - pix_range, 0))
        x_max = int(min(pix_node[0] + pix_range, self.gmap.data.shape[1]))
        y_min = int(max(pix_node[1] - pix_range, 0))
        y_max = int(min(pix_node[1] + pix_range, self.gmap.data.shape[0]))
        min_pix_to_occ = 2.0 * pix_range
        for i in range(x_min, x_max):
            for j in range(y_min, y_max):
                if self.gmap.data[j][i] == 100:
                    min_pix_to_occ = min(
                        min_pix_to_occ,
                        np.linalg.norm([i - pix_node[0], j - pix_node[1]]),
                    )
        pc = 0.0
        min_range_to_occ = min_pix_to_occ * self.scale
        if min_range_to_occ < r:
            pc = 1.0 - (min_range_to_occ / r)
        return pc

    def proximity_bonus(self, node, prev, r):
        if prev is None:
            return 0.0
        dist = np.linalg.norm([node[0] - prev[0], node[1] - prev[1]])
        p = 0.0
        if dist < r:
            p = 1.0 - (dist / r)
        return p
