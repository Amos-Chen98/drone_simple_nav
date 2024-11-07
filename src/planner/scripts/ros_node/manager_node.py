import os
import sys
current_path = os.path.abspath(os.path.dirname(__file__))[:-9] # -9 removes '/ros_node'
sys.path.insert(0, current_path)
import datetime
import rosbag
from geometry_msgs.msg import PoseStamped
from transitions.extensions import GraphMachine
from transitions import Machine
import numpy as np
import rospy
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest
from mavros_msgs.msg import State, PositionTarget
from nav_msgs.msg import Odometry
import actionlib
from planner.msg import *
from pyquaternion import Quaternion


class DroneState():
    def __init__(self):
        self.global_pos = np.zeros(3)
        self.global_vel = np.zeros(3)
        self.local_vel = np.zeros(3)
        self.attitude = Quaternion()


class Manager():
    def __init__(self, node_name="manager"):
        # Node
        rospy.init_node(node_name, anonymous=False)

        # Members
        self.flight_state = State()
        self.offb_req = SetModeRequest()
        self.arm_req = CommandBoolRequest()
        self.pos_cmd = PositionTarget()
        self.drone_state = DroneState()

        # customized parameters
        self.recording_rosbag = rospy.get_param("~recording_rosbag", False)
        self.mission_mode = rospy.get_param("~mission_mode", 'manual')
        self.target_pos_z = rospy.get_param("~target_pos_z", 2.0)

        # Parameters
        self.offb_req.custom_mode = 'OFFBOARD'
        self.arm_req.value = True
        self.pos_cmd.coordinate_frame = 1
        self.pos_cmd.position.z = self.target_pos_z
        self.global_target = None
        self.takeoff_pos_x = 0.0
        self.takeoff_pos_y = 0.0

        # Flags and counters
        self.odom_received = False
        self.has_goal = False
        self.rosbag_is_on = False

        # Action client
        self.plan_client = actionlib.SimpleActionClient('plan', PlanAction)

        # Subscribers
        self.flight_state_sub = rospy.Subscriber('mavros/state', State, self.flight_state_cb)
        self.odom_sub = rospy.Subscriber('mavros/local_position/odom', Odometry, self.odom_cb)
        self.target_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.trigger_plan)

        # Publishers
        self.local_pos_cmd_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        self.next_goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # FSM
        self.fsm = GraphMachine(model=self, states=['HOVER', 'MISSION'], initial='HOVER')
        self.fsm.add_transition(trigger='trigger_plan', source='HOVER', dest='MISSION', after=['print_current_state', 'open_rosbag'])
        self.fsm.add_transition(trigger='trigger_plan', source='MISSION', dest='MISSION', after=['print_current_state', 'open_rosbag'])
        self.fsm.add_transition(trigger='finish_planning_cb', source='MISSION', dest='HOVER', after=['print_current_state', 'close_rosbag'])

    def print_current_state(self):
        rospy.loginfo("Current state: %s", self.state)

    def trigger_plan(self, target):
        print("")
        self.global_target = np.array([target.pose.position.x,
                                       target.pose.position.y,
                                       target.pose.position.z])
        goal_msg = PlanGoal()
        goal_msg.target = target
        if self.has_goal == True:
            self.plan_client.cancel_goal()
        else:
            self.has_goal = True

        self.plan_client.send_goal(goal_msg, done_cb=self.finish_planning_cb)

    def open_rosbag(self):
        if self.recording_rosbag:
            now = datetime.datetime.now()
            timestamp = now.strftime("%m%d%H%M%S")
            self.bag = rosbag.Bag(f'{current_path[:-8]}/rosbag/{timestamp}.bag', 'w')
            self.rosbag_is_on = True
            rospy.loginfo("rosbag opened!")

    def close_rosbag(self):
        if self.recording_rosbag:
            self.rosbag_is_on = False
            self.bag.close()
            rospy.loginfo("rosbag closed!")

    def finish_planning_cb(self, state, result):
        if result.success:
            rospy.loginfo("Reached goal!")
        else:
            rospy.loginfo("Failed to reach goal!")

        if self.mission_mode == "random":
            self.set_random_goal()

    def set_random_goal(self):
        '''
        randomly generate a goal
        '''
        x_bounds = [-2, 28]
        y_bounds = [-8, 8]
        # randomly generate a goal
        x = np.random.uniform(x_bounds[0], x_bounds[1])
        y = np.random.uniform(y_bounds[0], y_bounds[1])

        # if target is in obstale-rich aera, regenerate
        while x > 0 and x < 26 and y > -6 and y < 6:
            x = np.random.uniform(x_bounds[0], x_bounds[1])
            y = np.random.uniform(y_bounds[0], y_bounds[1])

        target = PoseStamped()
        target.header.frame_id = "map"
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = self.target_pos_z

        self.next_goal_pub.publish(target)

    def flight_state_cb(self, data):
        self.flight_state = data

    def get_odom(self):
        while not self.odom_received or not self.flight_state.connected:
            rospy.sleep(0.01)

    def odom_cb(self, data):
        '''
        1. store the drone's global status
        2. publish dynamic tf transform from map frame to camera frame
        (Currently, regard camera frame as drone body frame)
        '''
        self.odom_received = True

        if self.recording_rosbag and self.rosbag_is_on:
            self.bag.write('mavros/local_position/odom', data)

        local_pos = np.array([data.pose.pose.position.x,
                              data.pose.pose.position.y,
                              data.pose.pose.position.z])
        global_pos = local_pos
        local_vel = np.array([data.twist.twist.linear.x,
                              data.twist.twist.linear.y,
                              data.twist.twist.linear.z])
        quat = Quaternion(data.pose.pose.orientation.w,
                          data.pose.pose.orientation.x,
                          data.pose.pose.orientation.y,
                          data.pose.pose.orientation.z)  # from local to global
        global_vel = quat.rotate(local_vel)
        self.drone_state.global_pos = global_pos
        self.drone_state.global_vel = global_vel
        self.drone_state.local_vel = local_vel
        self.drone_state.attitude = quat

    def draw_fsm_graph(self):
        self.get_graph().draw(f'{current_path[:-8]}/fsm.pdf', prog='dot')


if __name__ == "__main__":

    manager = Manager()
    
    rospy.spin()
