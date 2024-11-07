import numpy as np
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from scipy.spatial import cKDTree
import math


class PCLServer():
    def __init__(self, collision_threshold=0.4):
        self.collision_threshold = collision_threshold

    def pcl_cb(self, msg):
        # Convert PointCloud2 message to a NumPy array
        points = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(msg, skip_nans=True)])
        self.tree = cKDTree(points)

    def has_collision(self, point):
        '''
        input: point: a 3-element list [x, y, z], or numpy array
        return: True if the point is in collision, False otherwise
        '''
        if self.tree is None:
            return False
        distance, _ = self.tree.query([point[0], point[1], point[2]], k=1)
        return distance < self.collision_threshold

    def has_collision_strict(self, point):
        '''
        input: point: a 3-element list [x, y, z], or numpy array
        return: True if the point is in collision, False otherwise
        This is a more strict version of has_collision, which uses a larger threshold
        '''
        if self.tree is None:
            return False
        distance, _ = self.tree.query([point[0], point[1], point[2]], k=1)
        return distance < self.collision_threshold * 1.5

    def seg_feasible_check(self, head_pos, tail_pos, step_size=0.1):
        '''
        Check if the straight line from head_pos to tail_pos is feasible.
        '''
        x0, y0, z0 = head_pos
        x1, y1, z1 = tail_pos

        step_num = math.ceil(max(abs(x1 - x0), abs(y1 - y0), abs(z1 - z0)) / step_size) + 1
        x_check_list = np.linspace(x0, x1, step_num)
        y_check_list = np.linspace(y0, y1, step_num)
        z_check_list = np.linspace(z0, z1, step_num)

        for x, y, z in zip(x_check_list, y_check_list, z_check_list):
            if self.has_collision([x, y, z]):
                return False
        return True
