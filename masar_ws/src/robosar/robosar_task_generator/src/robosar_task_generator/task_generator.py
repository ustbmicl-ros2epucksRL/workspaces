import numpy as np
from matplotlib import pyplot as plt
import skimage.filters
from multiprocessing import Pool

from robosar_task_generator import max_circle

class TaskGenerator:
    """
    Generates tasks for a given map
    """
    def __init__(self, map, threshold = 0.5, resolution=8, num_samples = 100):
        """
        map is 2D matrix of occupancy grid map in (x,y), not (row,col)
        num_tasks is number of tasks to allocate in map
        threshold determines whether space is occupied or not
        resolution is number of points used for collision detection
        """
        self._map = map
        self._num_samples = num_samples
        self._threshold = threshold
        self._maxcircle = max_circle.MaxCircle(map, threshold, resolution)
    
    def generate_random_points(self, num_pts):
        """
        Generates a number of random points in the map without collisions
        """
        x_coord = np.zeros((num_pts,)).astype(int)
        y_coord = np.zeros((num_pts,)).astype(int)
        collision_mask = np.ones((num_pts,)).astype(int)
        is_collision = True
        while(is_collision):
            x_coord[collision_mask] = np.random.randint(0, self._map.shape[0], np.count_nonzero(collision_mask))
            y_coord[collision_mask] = np.random.randint(0, self._map.shape[1], np.count_nonzero(collision_mask))
            map_vals = self._map[x_coord,y_coord]
            collision_mask = np.logical_or(map_vals > self._threshold, map_vals < 0)
            is_collision = np.any(collision_mask)
        pts = np.hstack((np.expand_dims(x_coord,1), np.expand_dims(y_coord,1)))
        return pts

    def generate_exhaustive_points(self):
        """
        Generates points on every valid point on map
        """
        num_pts = 0
        x = np.arange(0, self._map.shape[0])
        y = np.arange(0, self._map.shape[1])
        x_coord, y_coord = np.meshgrid(x,y)
        x_coord = x_coord.reshape((-1,))
        y_coord = y_coord.reshape((-1,))
        map_vals = self._map[x_coord,y_coord]
        collision_mask = np.logical_or(map_vals > self._threshold, map_vals<0)
        valid_mask = np.logical_not(collision_mask)
        pts = np.hstack((np.expand_dims(x_coord[valid_mask],1), np.expand_dims(y_coord[valid_mask],1)))
        return pts

    def max_circle_pts(self, pts):
        """
        Generates array of maximum radius for each given point
        """
        max_radius_array = np.zeros((pts.shape[0],))
        # Prepare arguments
        arg_list = []
        for i in range(0, pts.shape[0]):
            arg_list.append(pts[i])
        with Pool() as p:
            rst_list = p.map(self._maxcircle.max_radius, arg_list)
        max_radius_array = np.array(rst_list)
        return max_radius_array
    
    def remove_overlaps(self, centers, radii):
        """
        Find and remove centers that are included in other circles
        Circle with larger radius have priority
        Returns centers and radii without overlap
        """
        to_delete = np.zeros((centers.shape[0],)).astype(bool)
        for i in range(0,centers.shape[0]):
            cur_center = centers[i]
            cur_radius = radii[i]
            other_centers = np.delete(np.copy(centers), i, axis=0)
            other_radii = np.delete(np.copy(radii), i)
            other_idx = np.delete(np.arange(0,centers.shape[0]), i)
            # Can eliminiate candidates that are further than radius in x or y directions
            candidates_mask_x = np.logical_and(other_centers[:,0] > (cur_center[0]-cur_radius), other_centers[:,0] < (cur_center[0]+cur_radius))
            candidates_mask_y = np.logical_and(other_centers[:,1] > (cur_center[1]-cur_radius), other_centers[:,1] < (cur_center[1]+cur_radius))
            candidates_mask = np.logical_and(candidates_mask_x, candidates_mask_y)
            candidates = other_centers[candidates_mask]
            candidates_radii = other_radii[candidates_mask]
            candidates_idx = other_idx[candidates_mask]
            # Check if candidates are within cur_circle
            candidate_dists = np.linalg.norm(cur_center-candidates,axis=1)
            confirmed_mask = candidate_dists < cur_radius
            confirmed = candidates[confirmed_mask]
            confirmed_radii = candidates_radii[confirmed_mask]
            confirmed_idx = candidates_idx[confirmed_mask]
            # Flag smaller circles for deletion
            smaller_mask = confirmed_radii < cur_radius
            to_delete_mask = confirmed_idx[smaller_mask]
            to_delete[to_delete_mask] = True
        to_keep = np.logical_not(to_delete)
        new_centers = centers[to_keep]
        new_radii = radii[to_keep]
        return new_centers, new_radii
    
    def select_waypoints(self, centers, radii):
        """
        Select waypoints. Uses Otsu thresholding to keep interesting waypoints
        Other options are also valid (eg. radii above mean radii, all radii above some threshold, etc.)
        Returns M x 3 matrix of waypoints and their associated radii
        """
        waypoints = np.hstack((centers, np.expand_dims(radii,1))).astype(int)
        otsu_threshold = skimage.filters.threshold_otsu(waypoints[:,2])
        otsu_mask = waypoints[:,2] >= otsu_threshold
        return waypoints[otsu_mask]
    
    def generate_tasks(self, num_iter):
        """
        Generate waypoints for a given map
        num_iter determines number of iterations
        Returns waypoints of final iteration
        """
        # Generate good waypoints
        for i in range(0, num_iter):
            # Generate random points
            centers = self.generate_random_points(self._num_samples)
            # Compute maximum radius for each point
            radii = self.max_circle_pts(centers)
            # Remove overlaps, current and past
            if(i == 0):
                # First time; just remove overlaps
                centers, radii = self.remove_overlaps(centers, radii)
                waypoints = np.hstack((centers, np.expand_dims(radii,1)))
            else:
                # Append and remove overlaps
                cur_waypoints = np.hstack((centers, np.expand_dims(radii,1)))
                waypoints = np.vstack((waypoints, cur_waypoints))
                centers, radii = self.remove_overlaps(waypoints[:,0:2], waypoints[:,2])
                waypoints = np.hstack((centers, np.expand_dims(radii, 1)))
            # Select waypoints
            waypoints = self.select_waypoints(waypoints[:,0:2],waypoints[:,2])
            print("Finished ", i, "...")
        # Remove redundant waypoints
        to_del = np.zeros(waypoints.shape[0]).astype(bool)
        for i in range(0, waypoints.shape[0]-1):
            cur_center = waypoints[i,0:2]
            other_centers = waypoints[i+1:,0:2]
            other_dists = np.linalg.norm(cur_center-other_centers,axis=1)
            close_mask = np.concatenate((np.zeros((i+1,)), other_dists < waypoints[i,2]/3)).astype(bool)
            to_del[close_mask] = True
        waypoints = waypoints[np.logical_not(to_del)]
        return waypoints
    
    def generate_tasks_exhaustive(self):
        """
        Generate waypoints for a given map
        Exhaustive, checking every point on map
        Returns waypoints of final iteration
        """
        # Generate exhaustive points
        centers = self.generate_exhaustive_points()
        # Compute maximum radius for each point
        radii = self.max_circle_pts(centers)
        # Remove overlaps
        centers, radii = self.remove_overlaps(centers, radii)
        waypoints = np.hstack((centers, np.expand_dims(radii,1)))
        # Select waypoints
        waypoints = self.select_waypoints(waypoints[:,0:2],waypoints[:,2])
        # Remove redundant waypoints
        to_del = np.zeros(waypoints.shape[0]).astype(bool)
        for i in range(0, waypoints.shape[0]-1):
            cur_center = waypoints[i,0:2]
            other_centers = waypoints[i+1:,0:2]
            other_dists = np.linalg.norm(cur_center-other_centers,axis=1)
            close_mask = np.concatenate((np.zeros((i+1,)), other_dists < waypoints[i,2]/3)).astype(bool)
            to_del[close_mask] = True
        waypoints = waypoints[np.logical_not(to_del)]
        return waypoints
    
    def visualize_pts(self, pts):
        """
        Visualize map and given points
        """
        # Plot map
        xx,yy = np.meshgrid(np.arange(self._map.shape[0]), np.arange(self._map.shape[1]))
        mask = self._map[xx,yy] > self._threshold
        plt.scatter(xx[mask],yy[mask])
        # Plot pts
        plt.scatter(pts[:,0], pts[:,1])
    
    def visualize_circles(self, centers, radii):
        # Plot map
        xx,yy = np.meshgrid(np.arange(self._map.shape[0]), np.arange(self._map.shape[1]))
        mask = self._map[xx,yy] > self._threshold
        plt.scatter(xx[mask],yy[mask], c='k', s=1)
        # Plot circles
        for i in range(0, centers.shape[0]):
            center = centers[i,:]
            radius = radii[i]
            circle = self._maxcircle.compute_circle(center, radius)
            x_plt = np.append(circle[:,0],center[0])
            y_plt = np.append(circle[:,1],center[1])
            plt.scatter(x_plt, y_plt, s = 1)
    


if __name__ == "__main__":
    import time

    # # Generate map; map uses (x,y) not (row,col)
    test_map = np.zeros((300, 400))
    # Boundaries
    test_map[[0,-1],:] = 1
    test_map[:,[0,-1]] = 1
    # Horizontal walls
    test_map[0:125, [100,200,300]] = 1
    test_map[175:, [100,200,300]] = 1
    # Vertical walls
    test_map[[125,175], 75:125] = 1
    test_map[[125,175], 175:225] = 1
    test_map[[125,175], 275:325] = 1
    # Plot map
    # plt.imshow(test_map.T, cmap="Greys")
    # plt.title("Test map")
    # plt.show()

    # # Constructor
    taskgen = TaskGenerator(test_map)
    
    # # # Test generate_exhaustive_points
    # test_pts = taskgen.generate_exhaustive_points()
    # taskgen.visualize_pts(test_pts)
    # plt.title("generate_random_points test")
    # plt.show()

    # # # Test generate_random_points
    # test_pts = taskgen.generate_random_points(1000)
    # # taskgen.visualize_pts(test_pts)
    # # plt.title("generate_random_points test")
    # # plt.show()

    # # # Test max_circle_pts
    # test_max_r_array = taskgen.max_circle_pts(test_pts)
    # # taskgen.visualize_circles(test_pts, test_max_r_array)
    # # plt.title("max_cricle_pts test")
    # # plt.show()

    # # # Test remove_overlaps
    # test_new_centers, test_new_radii = taskgen.remove_overlaps(test_pts, test_max_r_array)
    # # taskgen.visualize_circles(test_new_centers, test_new_radii)
    # # plt.title("remove_overlaps test")
    # # plt.show()

    # # # Test select_waypoints
    # test_waypoints = taskgen.select_waypoints(test_new_centers, test_new_radii)
    # # taskgen.visualize_pts(test_waypoints)
    # # plt.show()

    # # Test generate_tasks
    start = time.time()
    waypoints = taskgen.generate_tasks(20)
    end = time.time()
    elapsed_time = end-start
    print("Elapsed time: ", elapsed_time, "s")
    taskgen.visualize_circles(waypoints[:,0:2], waypoints[:,2])
    plt.show()

    # # # Test generate_tasks_exhaustive
    # start = time.time()
    # waypoints = taskgen.generate_tasks_exhaustive()
    # end = time.time()
    # elapsed_time = end-start
    # print("Elapsed time: ", elapsed_time, "s")
    # taskgen.visualize_circles(waypoints[:,0:2], waypoints[:,2])
    # plt.show()
