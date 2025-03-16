import numpy as np
import rospy
import octomap
import math


class OctreeServer():
    def __init__(self, bt_filepath, resolution=0.1, collision_threshold=0.2):
        self.octree = None
        self.build_octree_from_file(bt_filepath, resolution)
        self.neighbor_dis = collision_threshold

    def build_octree_from_file(self, bt_filepath, resolution):
        self.octree = octomap.OcTree(resolution)  # resolution is 0.1
        success = self.octree.readBinary(bt_filepath.encode())  # Ref: https://github.com/wkentaro/octomap-python/issues/10
        if success:
            rospy.loginfo("OcTree successfully loaded!")
        else:
            rospy.logerr("Failed to load OctoMap from binary data")

    def is_point_occupied(self, point):
        if self.octree is None:
            rospy.logerr("OcTree not initialized")
            return False

        node = self.octree.search(point)
        if node is not None:  # if the node is not in the tree, it will still return an object, here Python is different from C++
            try:
                return self.octree.isNodeOccupied(node)  # if the node is not in the tree, isNodeOccupied() will raise an exception here
            except Exception as e:
                return False

        return False

    def has_collision(self, point):
        '''
        input: point: a 3-element list [x, y, z], or numpy array
        return: True if the point is in collision, False otherwise
        '''
        offsets = np.array([-self.neighbor_dis, 0, self.neighbor_dis])
        for dx in offsets:
            for dy in offsets:
                for dz in offsets:
                    if self.is_point_occupied([point[0] + dx, point[1] + dy, point[2] + dz]):
                        return True
        return False

    def has_collision_strict(self, point):
        '''
        This is a more strict version of has_collision, which uses a larger threshold
        This is used for obtaining local target point, since one segment determined by two feasible points may not be feasible, we need to make sure the target point is safe enough
        '''
        offsets = np.array([-self.neighbor_dis, 0, self.neighbor_dis]) * 1.5
        for dx in offsets:
            for dy in offsets:
                for dz in offsets:
                    if self.is_point_occupied([point[0] + dx, point[1] + dy, point[2] + dz]):
                        return True
        return False

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
            if self.is_point_occupied([x, y, z]):
                return False
        return True
