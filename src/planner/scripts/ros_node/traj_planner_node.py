import os
import sys
current_path = os.path.abspath(os.path.dirname(__file__))[:-9]  # -9 removes '/ros_node'
sys.path.insert(0, current_path)
from sensor_msgs.msg import PointCloud2
from tf.transformations import euler_from_quaternion
from planner.msg import *
import actionlib
from nav_msgs.msg import Odometry, Path
from map_server.pcl_server import PCLServer
import time
from pyquaternion import Quaternion
from mavros_msgs.srv import SetMode, SetModeRequest
from mavros_msgs.msg import State, PositionTarget
import numpy as np
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from visualizer.visualizer import Visualizer
from traj_planner.geo_planner import GeoPlanner


class DroneState():
    def __init__(self):
        self.global_pos = np.zeros(3)
        self.global_vel = np.zeros(3)
        self.local_vel = np.zeros(3)
        self.attitude = Quaternion()  # ref: http://kieranwynn.github.io/pyquaternion/
        self.yaw = 0.0


class AstarConfig():
    def __init__(self):
        self.resolution = rospy.get_param("~resolution", 0.2)
        self.min_flight_height = rospy.get_param("~min_flight_height", 2.0)
        self.max_flight_height = rospy.get_param("~max_flight_height", 4.0)


class TrajPlanner():
    def __init__(self, node_name="traj_planner"):
        # Node
        rospy.init_node(node_name, anonymous=False)

        # Parameters
        a_star_config = AstarConfig()
        collision_threshold = rospy.get_param("~collision_threshold", 0.4)
        self.replan_mode = rospy.get_param("~replan_mode", 'fix_time')  # available options: global, fix_time, rush
        self.replan_period = rospy.get_param("~replan_period", 0.5)  # the interval between replanning， 0 means replan right after the previous plan
        self.planning_time_ahead = rospy.get_param("~planning_time_ahead", 1.0)  # the time ahead of the current time to plan the trajectory
        self.longitu_step_dis = rospy.get_param("~longitu_step_dis", 5.0)  # the distance forward in each replanning
        self.lateral_step_length = rospy.get_param("~lateral_step_length", 1.0)  # if local target pos in obstacle, take lateral step
        self.target_reach_threshold = rospy.get_param("~target_reach_threshold", 0.2)
        self.cmd_hz = rospy.get_param("~cmd_hz", 60)
        self.yaw_shift_tol = rospy.get_param("~yaw_shift_tol", 0.17453)
        self.move_vel = rospy.get_param("~move_vel", 1.0)
        self.target_pos_z = rospy.get_param("~target_pos_z", 2.0)

        # Planner
        self.planner = GeoPlanner(a_star_config, self.move_vel)
        self.map = PCLServer(collision_threshold)
        self.visualizer = Visualizer()
        self.drone_state = DroneState()
        self.state_cmd = PositionTarget()
        self.state_cmd.coordinate_frame = 1
        self.des_path = Path()
        self.init_marker_arrays()

        # Flags and counters
        self.target_received = False
        self.reached_target = False
        self.near_global_target = False
        self.odom_received = False
        self.des_state_index = 0
        self.future_index = 99999
        self.des_state_length = 99999  # this is used to check if the des_state_index is valid

        # Server
        self.plan_server = actionlib.SimpleActionServer('plan', PlanAction, self.execute_mission, False)
        self.plan_server.start()

        # Subscribers
        self.flight_state_sub = rospy.Subscriber('mavros/state', State, self.flight_state_cb)
        self.pcl_sub = rospy.Subscriber('/pointcloud/output', PointCloud2, self.map.pcl_cb)
        self.odom_sub = rospy.Subscriber('mavros/local_position/odom', Odometry, self.odom_cb)

        # Publishers
        self.local_pos_cmd_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        self.target_vis_pub = rospy.Publisher('global_target', Marker, queue_size=10)
        self.local_target_pub = rospy.Publisher('local_target', Marker, queue_size=10)
        # self.des_wpts_pub = rospy.Publisher('des_wpts', MarkerArray, queue_size=10)
        # self.des_path_pub = rospy.Publisher('des_path', MarkerArray, queue_size=10)

        rospy.loginfo(f"Trajectory planner initialized!")

    def flight_state_cb(self, data):
        self.flight_state = data

    def odom_cb(self, data):
        '''
        1. store the drone's global status
        2. publish dynamic tf transform from map frame to camera frame
        (Currently, regard camera frame as drone body frame)
        '''
        self.odom_received = True
        self.odom = data
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

        # get yaw from quaternion
        euler = euler_from_quaternion([data.pose.pose.orientation.x,
                                       data.pose.pose.orientation.y,
                                       data.pose.pose.orientation.z,
                                       data.pose.pose.orientation.w])

        self.drone_state.yaw = euler[2]

        if self.target_received and np.linalg.norm(global_pos - self.global_target) < self.target_reach_threshold:
            rospy.loginfo("Global target reached!\n")
            self.end_mission(reached_target=True)

    def init_mission(self):
        self.target_received = True
        self.reached_target = False
        self.near_global_target = False
        self.des_state_index = 0

    def end_mission(self, reached_target):
        self.tracking_cmd_timer.shutdown()
        self.target_received = False
        self.reached_target = reached_target
        self.near_global_target = False
        self.des_state_index = 0
        if self.replan_mode == 'fix_time':
            self.replan_timer.shutdown()

    def execute_mission(self, goal):
        target = goal.target

        self.global_target = np.array([target.pose.position.x, target.pose.position.y, self.target_pos_z])  # 3d pos
        rospy.loginfo("Target received: x = %f, y = %f, z = %f", target.pose.position.x, target.pose.position.y, self.target_pos_z)
        self.vis_target()

        self.init_mission()

        if self.replan_mode == 'global':
            rospy.loginfo("Replan mode: global")
            self.global_planning()
        elif self.replan_mode == 'rush':
            rospy.loginfo("Replan mode: rush")
            self.online_planning()
        elif self.replan_mode == 'fix_time':
            rospy.loginfo("Replan mode: fix_time")
            self.periodic_planning()
        else:
            rospy.logerr("Invalid replan_mode!")

        self.report_planning_result()

    def vis_target(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.position.x = self.global_target[0]
        marker.pose.position.y = self.global_target[1]
        marker.pose.position.z = self.target_pos_z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.target_vis_pub.publish(marker)

    def report_planning_result(self):
        while not self.plan_server.is_preempt_requested() and not self.reached_target:
            time.sleep(0.01)

        if self.plan_server.is_preempt_requested():
            rospy.loginfo("Planning preempted!\n")
            self.end_mission(reached_target=False)
            self.plan_server.set_preempted()
        else:  # this means the target is reached
            result = PlanResult()
            result.success = self.reached_target
            self.plan_server.set_succeeded(result)

    def global_planning(self):
        while not self.odom_received:
            time.sleep(0.01)

        self.target_state = np.array([self.global_target, np.zeros(3)])  # [3d pos, 3d vel]

        self.first_plan()
        self.start_tracking()
        # self.visualize_des_wpts()
        # self.visualize_des_path()

    def online_planning(self):
        while not self.odom_received:
            time.sleep(0.01)

        self.try_first_plan()
        self.start_tracking()

        while (
                not self.reached_target
                and not self.near_global_target
                and not self.plan_server.is_preempt_requested()
        ):
            self.try_local_planning()

    def periodic_planning(self):
        while not self.odom_received:
            time.sleep(0.01)

        self.try_first_plan()
        self.start_tracking()

        # after the first plan, replan periodically
        self.replan_timer = rospy.Timer(rospy.Duration(self.replan_period), self.replan_cb)

    def try_first_plan(self):
        seed = 0
        self.set_local_target(seed)
        # self.visualize_local_target()
        while True:
            try:
                self.first_plan()
                break
            except Exception as ex:
                rospy.logwarn("First planning failed: %s", ex)
                seed += 1
                self.set_local_target(seed)
                if seed > 10:
                    rospy.logerr("Entire planning failed!\n")
                    self.end_mission(reached_target=False)
                    self.plan_server.set_aborted()
                    return

        # self.visualize_des_wpts()
        # self.visualize_des_path()

    def replan_cb(self, event):
        if (
            not self.reached_target
            and not self.near_global_target
            and not self.plan_server.is_preempt_requested()
        ):
            self.try_local_planning()

    def try_local_planning(self):
        seed = 0
        self.set_local_target(seed)
        while True:
            try:
                self.replan()
                break
            except Exception as ex:
                rospy.logwarn("Local planning failed: %s", ex)
                seed += 1
                self.set_local_target(seed)
                if seed > 10:
                    rospy.logerr("Entire planning failed!\n")
                    self.end_mission(reached_target=False)
                    self.plan_server.set_aborted()
                    return

        # self.visualize_des_wpts()
        # self.visualize_des_path()

    def set_local_target(self, seed=0):
        current_pos = self.drone_state.global_pos
        current_pos_2d = current_pos[:2]
        global_target_pos = self.global_target
        global_target_pos_2d = global_target_pos[:2]

        # if current pos is close enough to global target, set local target as global target
        if np.linalg.norm(global_target_pos - current_pos) < self.longitu_step_dis:  # 3d distance
            self.target_state[0] = global_target_pos
            self.near_global_target = True
            return

        longitu_dir = (global_target_pos_2d - current_pos_2d)/np.linalg.norm(global_target_pos_2d - current_pos_2d)
        lateral_dir = np.array([[longitu_dir[1], -longitu_dir[0]],
                                [-longitu_dir[1], longitu_dir[0]]])
        lateral_dir_flag = 0
        lateral_move_dis = self.lateral_step_length

        # get local target pos
        if seed > 1e-3:
            local_target_pos_2d = current_pos_2d + self.longitu_step_dis * longitu_dir + np.random.normal(0, 1, 2)  # 0 for mean, 1 for std
        else:
            local_target_pos_2d = current_pos_2d + self.longitu_step_dis * longitu_dir

        # expand local_target_pos_2d to a 3-dimensional array
        local_target_pos = np.append(local_target_pos_2d, self.target_pos_z)

        while self.map.has_collision_strict(local_target_pos):
            local_target_pos_2d += lateral_move_dis * lateral_dir[lateral_dir_flag]
            lateral_dir_flag = 1 - lateral_dir_flag
            lateral_move_dis += self.lateral_step_length
            local_target_pos = np.append(local_target_pos_2d, self.target_pos_z)

        # get local target vel
        goal_dir = (global_target_pos - local_target_pos) / np.linalg.norm(global_target_pos - local_target_pos)
        local_target_vel = self.move_vel * goal_dir

        local_target = np.array([local_target_pos,
                                 local_target_vel])

        self.target_state = local_target

        self.visualize_local_target()

    def first_plan(self):
        time_start = time.time()
        des_state = self.geo_traj_plan(self.map, self.drone_state, self.target_state)
        time_end = time.time()

        rospy.loginfo("Planning time: {}".format(time_end - time_start))

        # First planning! Retrieve planned trajectory
        # Set the des_state_array as des_state
        self.des_state_array = des_state
        self.des_state_length = self.des_state_array.shape[0]

    def get_drone_state_ahead(self):
        '''
        get the drone state after 1s from self.des_state
        '''
        self.future_index = min(int(self.planning_time_ahead * self.cmd_hz) + self.des_state_index,
                                self.des_state_length - 1)
        drone_state_ahead = DroneState()
        drone_state_ahead.global_pos = self.des_state_array[self.future_index, 0, :]
        drone_state_ahead.global_vel = self.des_state_array[self.future_index, 1, :]
        return drone_state_ahead

    def replan(self):
        drone_state_ahead = self.get_drone_state_ahead()

        time_start = time.time()
        des_state = self.geo_traj_plan(self.map, drone_state_ahead, self.target_state)
        time_end = time.time()

        rospy.loginfo("Planning time: {}".format(time_end - time_start))

        # Concatenate the new trajectory to the old one, at index self.future_index
        self.des_state_array = np.concatenate((self.des_state_array[:self.future_index], des_state), axis=0)
        self.des_state_length = self.des_state_array.shape[0]

    def geo_traj_plan(self, map, plan_init_state, target_state):
        des_state = self.planner.geo_traj_plan(map, plan_init_state, target_state)

        return des_state

        # visualize path
        # self.visualize_init_path(path)

    def warm_up(self):
        # Send a few setpoints before switching to OFFBOARD mode
        self.state_cmd.position.x = self.drone_state.global_pos[0]
        self.state_cmd.position.y = self.drone_state.global_pos[1]
        self.state_cmd.position.z = self.drone_state.global_pos[2]
        rate = rospy.Rate(100)
        for _ in range(5):  # set 5 points
            if (rospy.is_shutdown()):
                break
            self.local_pos_cmd_pub.publish(self.state_cmd)
            rate.sleep()

    def enter_offboard(self):
        '''
        if not in OFFBOARD mode, switch to OFFBOARD mode
        '''
        if self.flight_state.mode != "OFFBOARD":
            self.warm_up()
            set_offb_req = SetModeRequest()
            set_offb_req.custom_mode = 'OFFBOARD'
            if (self.set_mode_client.call(set_offb_req).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

    def start_tracking(self):
        '''
        When triggered, start to publish full state cmd
        '''
        # self.enter_offboard()

        self.tracking_cmd_timer = rospy.Timer(rospy.Duration(1/self.cmd_hz), self.tracking_cmd_timer_cb)

    def tracking_cmd_timer_cb(self, event):
        '''
        Publish state cmd, height is fixed to current height
        '''
        self.state_cmd.position.x = self.des_state_array[self.des_state_index][0][0]
        self.state_cmd.position.y = self.des_state_array[self.des_state_index][0][1]
        self.state_cmd.position.z = self.des_state_array[self.des_state_index][0][2]

        self.state_cmd.velocity.x = self.des_state_array[self.des_state_index][1][0]
        self.state_cmd.velocity.y = self.des_state_array[self.des_state_index][1][1]
        self.state_cmd.velocity.z = self.des_state_array[self.des_state_index][1][2]

        self.state_cmd.acceleration_or_force.x = self.des_state_array[self.des_state_index][2][0]
        self.state_cmd.acceleration_or_force.y = self.des_state_array[self.des_state_index][2][1]
        self.state_cmd.acceleration_or_force.z = self.des_state_array[self.des_state_index][2][2]

        self.state_cmd.yaw = np.arctan2(self.des_state_array[self.des_state_index][0][1] - self.des_state_array[self.des_state_index - 1][0][1],
                                        self.des_state_array[self.des_state_index][0][0] - self.des_state_array[self.des_state_index - 1][0][0])

        self.state_cmd.header.stamp = rospy.Time.now()

        self.local_pos_cmd_pub.publish(self.state_cmd)

        if self.des_state_index < self.des_state_length - 1:
            self.des_state_index += 1

    def init_marker_arrays(self):
        # local target
        self.local_target_marker = Marker()
        self.local_target_marker.header.frame_id = "map"
        self.local_target_marker.type = Marker.SPHERE
        self.local_target_marker.scale.x = 0.4
        self.local_target_marker.scale.y = 0.4
        self.local_target_marker.scale.z = 0.4
        self.local_target_marker.color.a = 1
        self.local_target_marker.color.r = 1
        self.local_target_marker.color.g = 1
        self.local_target_marker.color.b = 0
        self.local_target_marker.pose.orientation.w = 1.0

    def visualize_local_target(self):
        self.local_target_marker.header.stamp = rospy.Time.now()
        self.local_target_marker.pose.position.x = self.target_state[0][0]
        self.local_target_marker.pose.position.y = self.target_state[0][1]
        self.local_target_marker.pose.position.z = self.target_state[0][2]

        self.local_target_pub.publish(self.local_target_marker)

    #     # des wpts
    #     self.wpts_markerarray = MarkerArray()
    #     # max_wpts_length = 20
    #     max_wpts_length = 1000
    #     for i in range(max_wpts_length):
    #         marker = Marker()
    #         marker.id = i
    #         marker.header.frame_id = "map"
    #         marker.type = Marker.SPHERE
    #         marker.pose.orientation.w = 1.0
    #         marker.scale.x = 0.4
    #         marker.scale.y = 0.4
    #         marker.scale.z = 0.4

    #         self.wpts_markerarray.markers.append(marker)

    #     # des path
    #     self.path_markerarray = MarkerArray()
    #     max_path_length = 1000
    #     for i in range(max_path_length):
    #         marker = Marker()
    #         marker.id = i
    #         marker.header.frame_id = "map"
    #         marker.type = Marker.LINE_STRIP
    #         marker.pose.orientation.w = 1.0
    #         marker.scale.x = 0.1

    #         self.path_markerarray.markers.append(marker)

    # def visualize_init_path(self, path):
    #     '''
    #     path: [[x1,y1], [x2,y2], ...]
    #     This function is only used for demo. When using, make sure the length of MarkerArray is enough (around line 638)
    #     Plus, self.planner.geo_traj_plan should return the path
    #     '''
    #     path_array = np.array(path).T
    #     print("shape of path_array: ", path_array.shape)
    #     pos_array = np.vstack((path_array, self.target_pos_z * np.ones([1, path_array.shape[1]]))).T
    #     self.wpts_markerarray = self.visualizer.modify_wpts_markerarray(self.wpts_markerarray, pos_array)
    #     self.des_path_pub.publish(self.wpts_markerarray)

    # def visualize_des_wpts(self):
    #     '''
    #     Visualize the desired waypoints as markers
    #     '''
    #     pos_array = self.planner.int_wpts  # shape: (2,n)
    #     pos_array = np.vstack((pos_array, self.target_pos_z * np.ones([1, pos_array.shape[1]]))).T
    #     self.wpts_markerarray = self.visualizer.modify_wpts_markerarray(
    #         self.wpts_markerarray, pos_array)  # the id of self.wpts_markerarray will be the same
    #     self.des_wpts_pub.publish(self.wpts_markerarray)

    # def visualize_des_path(self):
    #     '''
    #     Visualize the desired path, where high-speed pieces and low-speed pieces are colored differently
    #     '''
    #     pos_array = self.planner.get_pos_array(hz=10)
    #     pos_array = np.hstack((pos_array, self.target_pos_z * np.ones([len(pos_array), 1])))
    #     vel_array = np.linalg.norm(self.planner.get_vel_array(), axis=1)  # shape: (n,)
    #     self.path_markerarray = self.visualizer.modify_path_markerarray(self.path_markerarray, pos_array, vel_array)
    #     self.des_path_pub.publish(self.path_markerarray)


if __name__ == "__main__":

    traj_planner = TrajPlanner()

    rospy.spin()
