import os
import sys
current_path = os.path.abspath(os.path.dirname(__file__))[:-9]  # -9 removes '/ros_node'
sys.path.insert(0, current_path)
from map_server.pcl_server import PCLServer
from sensor_msgs.msg import PointCloud2
import octomap
import rospy
import numpy as np
import time
import struct
from octomap_msgs.msg import Octomap


class CollisionTest():
    def __init__(self, node_name="collision_test"):
        # Node
        rospy.init_node(node_name, anonymous=False)

        # PCL
        self.pcl = PCLServer(0.1)  # safe dis = 0.1
        self.pcl_sub = rospy.Subscriber('/pointcloud/output', PointCloud2, self.pcl.pcl_cb)
        self.octomap_sub = rospy.Subscriber('/octomap_binary', Octomap, self.octomap_cb)

        # Octomap (Octree)
        self.octree = None

        rospy.sleep(3)  # to make sure the PCL is loaded from topic

        # Run collision test
        for _ in range(3):
            self.run_collision_test()

        print("Collision test finished")

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

    def run_collision_test(self):
        print("\nRunning batch collision test")
        points = self.gen_rand_points(100000)

        time_start = time.time()
        self.batch_collision_test_pcl(points)
        time_end = time.time()
        print("[KDTree] Time elapsed: ", time_end - time_start)

        time_start = time.time()
        self.batch_collision_test_octree(points)
        time_end = time.time()
        print("[Octomap] Time elapsed: ", time_end - time_start)

    def is_collision_octree(self, point):
        if self.octree is None:
            print("OctoTree not initialized")
            return False

        # Now search for it
        node = self.octree.search(point)
        if node is not None:  # if the node is not in the tree, it will still return an object, here Python is different from C++
            try:
                return self.octree.isNodeOccupied(node)  # if the node is not in the tree, it will raise an exception here
            except Exception as e:
                return False
        return False

    def batch_collision_test_octree(self, points):
        collision_num = 0
        for point in points:
            if self.is_collision_octree(point):
                collision_num += 1

        print("[Octomap] Number of points in collision: ", collision_num)

    def batch_collision_test_pcl(self, points):
        collision_num = 0
        for point in points:
            if self.pcl.has_collision(point):
                collision_num += 1

        print("[KDTree] Number of points in collision: ", collision_num)

    def gen_rand_points(self, num_points):
        points = []
        for i in range(num_points):
            x = np.random.uniform(0, 30)
            y = np.random.uniform(-5, 5)
            z = np.random.uniform(0, 5)
            points.append([x, y, z])
        return points


if __name__ == "__main__":
    collision_test = CollisionTest()

    rospy.spin()
