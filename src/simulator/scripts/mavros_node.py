import rospy
from nav_msgs.msg import Odometry
from mavros_msgs.msg import PositionTarget
import numpy as np
import pyquaternion


class FakeMavros():
    def __init__(self, node_name="mavros"):
        # Node
        rospy.init_node(node_name, anonymous=False)

        # Parameters
        self.odom_hz = 30
        init_pos = rospy.get_param("~init_pos", "0.0, 4.0, 2.0")
        self.init_pos = [float(x) for x in init_pos.split(",")]
        
        # Subscribers
        self.local_pos_cmd_sub = rospy.Subscriber('mavros/setpoint_raw/local', PositionTarget, self.cmd_cb)

        # Publishers
        self.odom_pub = rospy.Publisher('mavros/local_position/odom', Odometry, queue_size=10)

        # init odom
        self.odom = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_link"

        self.odom.pose.pose.position.x = self.init_pos[0]
        self.odom.pose.pose.position.y = self.init_pos[1]
        self.odom.pose.pose.position.z = self.init_pos[2]
        self.odom.pose.pose.orientation.w = 1

        self.odom.twist.twist.linear.x = 0
        self.odom.twist.twist.linear.y = 0
        self.odom.twist.twist.linear.z = 0
        self.odom.twist.twist.angular.x = 0
        self.odom.twist.twist.angular.y = 0
        self.odom.twist.twist.angular.z = 0

        self.publish_odom()

    def cmd_cb(self, data):
        '''
        Make the odom the same as the cmd
        '''
        self.odom.header.stamp = rospy.Time.now()
        self.odom.pose.pose.position.x = data.position.x
        self.odom.pose.pose.position.y = data.position.y
        self.odom.pose.pose.position.z = data.position.z

        self.odom.twist.twist.linear.x = data.velocity.x
        self.odom.twist.twist.linear.y = data.velocity.y
        self.odom.twist.twist.linear.z = data.velocity.z

        ax = data.acceleration_or_force.x
        ay = data.acceleration_or_force.y
        az = data.acceleration_or_force.z
        yaw = data.yaw

        g = 9.8
        a_ll = np.array([ax, ay, az + g])
        z_b = a_ll / np.linalg.norm(a_ll)
        x_c = np.array([np.cos(yaw), np.sin(yaw), 0])
        y_b = np.cross(z_b, x_c) / np.linalg.norm(np.cross(z_b, x_c))
        x_b = np.cross(y_b, z_b)

        # the rotation matrix from world frame to body frame
        rot_mat = np.linalg.inv(np.array([x_b, y_b, z_b]))

        # the quaternion from world frame to body frame
        q = pyquaternion.Quaternion(matrix=rot_mat)

        # Set the orientation in the odom message
        self.odom.pose.pose.orientation.w = q[0]
        self.odom.pose.pose.orientation.x = q[1]
        self.odom.pose.pose.orientation.y = q[2]
        self.odom.pose.pose.orientation.z = q[3]

    def publish_odom(self):
        self.odom_pub_timer = rospy.Timer(rospy.Duration(1/self.odom_hz), self.odom_pub_timer_cb)

    def odom_pub_timer_cb(self, event):
        self.odom_pub.publish(self.odom)


if __name__ == "__main__":
    mavros = FakeMavros()

    rospy.spin()
