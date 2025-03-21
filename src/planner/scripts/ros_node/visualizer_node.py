from nav_msgs.msg import Path
import rospy
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import numpy as np


class Visualizer():
    def __init__(self, node_name="visualizer"):
        # Node
        rospy.init_node(node_name, anonymous=False)
        self.real_path = Path()
        self.real_path.header.frame_id = "map"

        # randomly generate a color for the drone
        self.drone_color = [int(256 * x) for x in np.random.rand(3)]

        # drone marker
        self.drone_marker = Marker()
        self.drone_marker.header.frame_id = "map"
        self.drone_marker.type = self.drone_marker.MESH_RESOURCE
        self.drone_marker.mesh_resource = "package://planner/models/meshes/iris.stl"
        self.drone_marker.color.r = self.drone_color[0]/256
        self.drone_marker.color.g = self.drone_color[1]/256
        self.drone_marker.color.b = self.drone_color[2]/256
        self.drone_marker.color.a = 1
        self.drone_marker.scale.x = 1.5
        self.drone_marker.scale.y = 1.5
        self.drone_marker.scale.z = 1.5

        # Subscribers
        self.odom_sub = rospy.Subscriber('mavros/local_position/odom', Odometry, self.odom_cb)

        # Publishers
        self.real_path_pub = rospy.Publisher('real_path', Path, queue_size=10)
        self.drone_marker_pub = rospy.Publisher('drone_marker', Marker, queue_size=10)

    def odom_cb(self, data):
        pose_stamped = PoseStamped()
        pose_stamped.pose = data.pose.pose
        self.real_path.poses.append(pose_stamped)
        self.real_path_pub.publish(self.real_path)

        self.drone_marker.pose = data.pose.pose
        self.drone_marker_pub.publish(self.drone_marker)


if __name__ == "__main__":
    visualizer = Visualizer()

    rospy.spin()
