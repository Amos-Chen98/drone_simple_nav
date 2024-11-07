import os
import sys
current_path = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, current_path)
from traj_planner.traj_utils import TrajUtils
import numpy as np
from astar_planner import AstarPlanner
import math
import copy
import time



class GeoPlanner(TrajUtils):
    def __init__(self, a_star_config):
        super().__init__()
        self.astar_planner = AstarPlanner(a_star_config)
        self.s = 3
        self.D = 3
        self.avg_speed = 2.0  # m/s

    def geo_traj_plan(self, map, plan_init_state, target_state):
        start_pos = plan_init_state.global_pos
        # target_pos = np.array([target_state[0][0], target_state[0][1], plan_init_state.global_pos[2]])
        target_pos = target_state[0]
        start_vel = plan_init_state.global_vel
        # target_vel = np.array([target_state[1][0], target_state[1][1], 0.0])
        target_vel = target_state[1]

        # target_state = np.vstack((target_pos, target_vel))

        time_start = time.time()
        path = self.astar_planner.plan(map, start_pos, target_pos)
        time_end = time.time()
        print("\nA star planning time: ", time_end - time_start)

        if path is None:
            raise Exception("A star planner failed to find a path!")

        path = self.prune_path_nodes(map, path)

        if len(path) == 2:  # Means the path directly connects the start_pos and target_pos
            print("Straight line trajectory!")
            path_length = np.linalg.norm(np.array(path[0]) - np.array(path[1]))
            total_time = path_length / self.avg_speed
            sample_num = int(total_time * 60 + 1)

            des_pos = np.linspace(start_pos, target_pos, sample_num)
            des_vel = np.linspace(start_vel, target_vel, sample_num)
            des_acc = np.zeros((sample_num, 3))

            des_state = np.zeros((sample_num, 3, self.D))  # 3*D: [pos, vel, acc].T * D

            des_state[:, 0, :] = des_pos
            des_state[:, 1, :] = des_vel
            des_state[:, 2, :] = des_acc

        else:
            print("Minimum jerk trajectory!")
            path = path[1:-1]  # remove the first and last element of the path

            path_all = np.concatenate((np.array([start_pos]),
                                       np.array(path),
                                       np.array([target_pos])), axis=0)

            # calculate the distance between each two points in path_all
            distances = np.linalg.norm(np.diff(path_all, axis=0), axis=1)

            ts = distances / self.avg_speed

            head_state = np.array([start_pos, start_vel])
            int_wpts = np.array(path).T

            self.read_planning_condition(head_state, target_state, int_wpts, ts)

            des_state = self.get_full_state_cmd()

        return des_state

    def read_planning_condition(self, head_state, tail_state, int_wpts, ts):
        self.D = head_state.shape[1]
        self.M = ts.shape[0]

        self.head_state = np.zeros((self.s, self.D))
        self.tail_state = np.zeros((self.s, self.D))

        for i in range(min(self.s, head_state.shape[0])):
            self.head_state[i] = head_state[i]
        for i in range(min(self.s, tail_state.shape[0])):
            self.tail_state[i] = tail_state[i]

        self.int_wpts = int_wpts
        self.ts = ts

    def prune_path_nodes(self, map, path):
        '''
        including the head and tail of the path
        '''
        key_index = [0]
        head_index = int(0)
        tail_index = int(1)

        while tail_index < len(path):
            while map.seg_feasible_check(path[head_index], path[tail_index]) or tail_index - head_index == 1:
                tail_index += 1
                if tail_index == len(path):
                    break

            # the path from head_index to [tail_index - 1] is feasible
            # the path from head_index to tail_index is not feasible
            key_index.append(tail_index - 1)
            head_index = copy.deepcopy(tail_index - 1)  # reset the head_index

        path_pruned = []
        for i in key_index:
            path_pruned.append(path[i])

        return path_pruned
