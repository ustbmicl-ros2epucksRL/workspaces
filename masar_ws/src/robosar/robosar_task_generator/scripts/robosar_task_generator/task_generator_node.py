import rospy
from robosar_messages.srv import *

import numpy as np

from robosar_task_generator.task_generator import TaskGenerator

def handle_taskgen_getwaypts(req):
    # Extract map from req
    map_width = req.map.info.width
    map_height = req.map.info.height
    map_data = req.map.data
    req_map = np.zeros((map_width, map_height))
    for x in range(0, map_width):
        for y in range(0, map_height):
            idx = y*map_width + x
            req_map[x,y] = map_data[idx]
    req_map = req_map / 100 # Normalize
    # plt.imshow(req_map, cmap="Greys")
    # plt.show()

    # Get other args
    threshold = req.threshold
    iterations = req.iterations

    # Create class, get waypoints
    taskgen = TaskGenerator(req_map, threshold = threshold)
    waypoints = taskgen.generate_tasks(iterations)
    # taskgen.visualize_circles(waypoints[:,0:2], waypoints[:,2])
    # plt.show()

    # Return
    num_pts = waypoints.shape[0]
    dims = 2
    waypts = waypoints[:,0:2].flatten()
    return taskgen_getwayptsResponse(num_pts, dims, waypts)

def task_generator_server():
    rospy.init_node('task_generator_server')
    s = rospy.Service('taskgen_getwaypts', taskgen_getwaypts, handle_taskgen_getwaypts)
    print("Ready to generate waypts")
    rospy.spin()

if __name__ == "__main__":
    # from matplotlib import pyplot as plt
    task_generator_server()