import numpy as np
import rospy
import octomap
import math
import struct
import time


class OctreeServer():
    def __init__(self, collision_threshold=0.2):
        self.octree = None
        self.neighbor_dis = collision_threshold

    def octomap_cb(self, msg):
        try:
            file_header = "# Octomap OcTree binary file\n"
            file_header += "id " + msg.id + "\n"
            file_header += "size 10\n"  # This value is just a placeholder.
            file_header += "res " + str(msg.resolution) + "\n"
            file_header += "data\n"

            header_bytes = file_header.encode('utf-8')

            length = len(msg.data)
            pattern = '<%db' % length
            data_bytes = struct.pack(pattern, *msg.data)

            complete_data = header_bytes + data_bytes

            tmp_octree = octomap.OcTree(msg.resolution)
            tmp_octree.readBinary(complete_data)
            actual_size = tmp_octree.size()

            # repalce the placeholder with the actual size
            file_header = file_header.replace("size 10", "size " + str(actual_size))
            header_bytes = file_header.encode('utf-8')
            complete_data = header_bytes + data_bytes

            self.octree = octomap.OcTree(msg.resolution)
            self.octree.readBinary(complete_data)  # Read from memory

        except Exception as e:
            rospy.logerr(f"Error in octomap_cb: {e}")

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

    def has_collision(self, point, scaling=1.0):
        if self.is_point_occupied(point):
            return True

        main_offsets = [
            [self.neighbor_dis * scaling, 0, 0],
            [-self.neighbor_dis * scaling, 0, 0],
            [0, self.neighbor_dis * scaling, 0],
            [0, -self.neighbor_dis * scaling, 0],
            [0, 0, self.neighbor_dis * scaling],
            [0, 0, -self.neighbor_dis * scaling]
        ]

        for offset in main_offsets:
            check_point = [point[0] + offset[0], point[1] + offset[1], point[2] + offset[2]]
            if self.is_point_occupied(check_point):
                return True

        return False

    def has_collision_strict(self, point):
        return self.has_collision(point, scaling=1.5)

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
