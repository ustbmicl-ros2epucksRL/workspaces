import numpy as np
from matplotlib import pyplot as plt

class MaxCircle:
    """
    Finds maximum circle centered about point in given map
    """
    def __init__(self, map, threshold = 0.5, resolution=8):
        """
        Map is 2D matrix of occupancy grid map in (x,y), not (row,col)
        num_tasks is number of tasks to allocate in map
        Threshold determines whether space is occupied or not
        Resolution is number of points used to represent circle at 1 cell length
        """
        self._map = map
        self._threshold = threshold
        self._resolution = resolution
    
    def compute_circle(self, center, radius):
        """
        Compute circle of given radius with given center; all inputs and outputs in cells
        center is 1x2 matrix in cells
        radius is scalar given in cells
        Returns Mx2 matrix (x,y) of points presenting circle, in cells
        """
        circle_center = center.astype(int)
        circle_res = np.ceil(self._resolution * radius) # Make resolution scale with radius
        angle_rad = np.arange(0, circle_res)*(2*np.pi/circle_res)
        # Compute x coordinates of points
        x_coord = circle_center[0] + radius * np.sin(angle_rad)
        y_coord = circle_center[1] + radius * np.cos(angle_rad)
        circle = np.hstack((np.expand_dims(x_coord, 1),np.expand_dims(y_coord, 1))).astype(int)
        return circle
    
    def visualize(self, center, circle):
        """
        Visualize circle center and circumference
        """
        # Plot map
        xx,yy = np.meshgrid(np.arange(self._map.shape[0]), np.arange(self._map.shape[1]))
        mask = self._map[xx,yy] > 0.5
        plt.scatter(xx[mask],yy[mask])
        # Plot circle
        plt.scatter(center[0], center[1])
        plt.scatter(circle[:,0], circle[:,1])
    
    def check_collision(self, center, circle):
        """
        Check circle against map for collisions (above threshold or negative)
        Returns True if collision occurs
        """
        pts = np.vstack((np.expand_dims(center,0),circle))
        if((np.max(pts[:,0]) >= self._map.shape[0]) or (np.max(pts[:,1]) >= self._map.shape[1]) or (np.min(pts[:,0]) < 0) or (np.min(pts[:,1]) < 0)):
            # Out of bounds; consider this a collision
            print("WARN: Circle is out of bounds")
            return True
        map_vals = self._map[pts[:,0],pts[:,1]]
        return np.any(map_vals > self._threshold) or np.any(map_vals < 0.0)
    
    def max_radius(self, center):
        """
        Get maximum allowable radius without collisions
        """
        cur_radius = 0
        quit = False
        while(quit == False):
            cur_radius += 1
            quit = self.check_collision(center, self.compute_circle(center, cur_radius))
        return cur_radius - 1

if __name__ == "__main__":
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
    plt.imshow(test_map.T, cmap="Greys")
    plt.title("Test map")
    plt.show()

    # # Constructor
    maxcircle = MaxCircle(test_map)
    
    # # Test compute_circle
    # test_center = np.array([0,0])
    test_center = np.array([62,50])
    test_circle = maxcircle.compute_circle(test_center, 20)
    maxcircle.visualize(test_center, test_circle)
    plt.title("compute_circle test")
    plt.show()

    # # Test check_collision
    test_collision = maxcircle.check_collision(test_center, test_circle)
    if(test_collision):
        print("Collision has occurred!")
    else:
        print("No collision")
    maxcircle.visualize(test_center, test_circle)
    plt.title("check_collision test")
    plt.show()

    # # Test max_radius
    test_max_radius = maxcircle.max_radius(test_center)
    maxcircle.visualize(test_center, maxcircle.compute_circle(test_center, test_max_radius))
    print("Maximum radius: ", test_max_radius)
    plt.title("max_radius test")
    plt.show()
