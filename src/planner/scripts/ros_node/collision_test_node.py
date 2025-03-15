import os
import sys
current_path = os.path.abspath(os.path.dirname(__file__))[:-9]  # -9 removes '/ros_node'
sys.path.insert(0, current_path)
import time
import numpy as np
import rospy
import octomap
from sensor_msgs.msg import PointCloud2
from map_server.pcl_server import PCLServer



class CollisionTest():
    def __init__(self, node_name="collision_test"):
        # Node
        rospy.init_node(node_name, anonymous=False)

        # PCL
        self.pcl = PCLServer(0.1)  # safe dis = 0.1
        self.pcl_sub = rospy.Subscriber('/pointcloud/output', PointCloud2, self.pcl.pcl_cb)

        # Octomap (Octree)
        self.octree = None
        bt_filename = "bt_files/bricks.bt"
        bt_filepath = current_path + '/' + bt_filename
        self.build_octree_from_file(bt_filepath)

        rospy.sleep(3)  # to make sure the PCL is loaded from topic

        # Run collision test
        for _ in range(3):
            self.run_collision_test()
        
        print("Collision test finished")

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

    def build_octree_from_file(self, bt_filepath):
        self.octree = octomap.OcTree(0.1)  # resolution is 0.1
        success = self.octree.readBinary(bt_filepath.encode())  # Ref: https://github.com/wkentaro/octomap-python/issues/10
        if success:
            rospy.loginfo("OctoMap successfully loaded!")
            # print("Nodes: ", self.octree.getNumLeafNodes())
            # print("Depth: ", self.octree.getTreeDepth())
            # print("Resolution: ", self.octree.getResolution())
            # print("Volume: ", self.octree.volume())
            # print("Memory: ", self.octree.memoryUsage())
            # print("Memory Full Grid: ", self.octree.memoryFullGrid())
            # print("Size: ", self.octree.size())
            # print("Num Nodes: ", self.octree.calcNumNodes())
            # print("Num Leaf Nodes: ", self.octree.getNumLeafNodes())
        else:
            rospy.logerr("Failed to load OctoMap from binary data")

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
