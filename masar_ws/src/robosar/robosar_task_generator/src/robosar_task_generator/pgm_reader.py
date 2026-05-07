import numpy as np

def read_pgm():
    """Return a raster of integers from a PGM as a list of lists."""
    # Open file
    pgmf = open('../maps/localization_map_vicon.pgm', 'rb')
    # Discard first line; P5
    print(pgmf.readline())
    # Discard second line; resolution
    print(pgmf.readline())
    # Discard third line; dimensions
    print(pgmf.readline())
    # Discard fourth line; depth
    print(pgmf.readline())
    # Hardcode values for map for now
    width = 168
    height = 101
    depth = 255
    # Read in map from file
    map = np.zeros((width,height))
    raster = []
    for y in range(height):
        row = []
        for y in range(width):
            row.append(ord(pgmf.read(1)))
        raster.append(row)
    # Convert (row,col) to (x,y)
    map = np.flip(np.asarray(raster).T,axis=1)/depth
    # Make freespace = 0, make occupied = 1
    map = 1-map
    return map

if __name__ == "__main__":
    from matplotlib import pyplot as plt
    import task_generator
    import time
    import skimage.filters

    # Seed RNG
    np.random.seed(0)

    # # Test reading map
    test_map = read_pgm()
    # plt.imshow(test_map, cmap='Greys')
    # plt.show()
    
    # # Run task_generator
    taskgen = task_generator.TaskGenerator(test_map, threshold = 0.18, num_samples=100)
    start = time.time()
    waypoints = taskgen.generate_tasks(20)
    end = time.time()
    elapsed_time = end-start
    print("Elapsed time: ", elapsed_time, "s")
    # Hardcode to remove bad waypoints
    to_remove_x = waypoints[:,0] < 55
    to_remove_y = waypoints[:,1] < 26
    to_remove = np.logical_or(to_remove_x, to_remove_y)
    to_keep = np.logical_not(to_remove)
    waypoints = waypoints[to_keep]

    taskgen.visualize_circles(waypoints[:,0:2], waypoints[:,2])
    plt.xlim([0, test_map.shape[0]])
    plt.ylim([0, test_map.shape[1]])
    plt.show()
    # Export waypoints
    np.save("../outputs/localization_map_vicon",waypoints[:,0:2])
    # Fewer waypoints
    otsu_threshold = skimage.filters.threshold_otsu(waypoints[:,2])
    otsu_mask = waypoints[:,2] >= otsu_threshold
    waypoints_lean = waypoints[otsu_mask]
    taskgen.visualize_circles(waypoints_lean[:,0:2], waypoints_lean[:,2])
    plt.xlim([0, test_map.shape[0]])
    plt.ylim([0, test_map.shape[1]])
    plt.show()
    # Export waypoints_lean
    np.save("../outputs/localization_map_vicon_lean",waypoints_lean[:,0:2])


