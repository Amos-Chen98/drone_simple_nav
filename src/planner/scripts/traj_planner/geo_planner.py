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
    def __init__(self, a_star_config, move_vel, cmd_hz):
        super().__init__()
        self.astar_planner = AstarPlanner(a_star_config)
        self.s = 3
        self.D = 3
        self.move_vel = move_vel
        self.cmd_hz = cmd_hz

    def geo_traj_plan(self, map, plan_init_state, target_state):
        start_pos = plan_init_state.global_pos
        target_pos = target_state[0]

        time_start = time.time()
        path = self.astar_planner.plan(map, start_pos, target_pos)
        time_end = time.time()
        print("\nA star planning time: ", time_end - time_start)

        if path is None:
            raise Exception("A star planner failed to find a path!")

        path = self.prune_path_nodes(map, path)

        des_state = []

        for i in range(len(path) - 1):
            seg_start = np.array(path[i])
            seg_end = np.array(path[i + 1])
            seg_unit_dir = (seg_end - seg_start) / np.linalg.norm(seg_end - seg_start)
            seg_length = np.linalg.norm(seg_end - seg_start)
            seg_time = seg_length / self.move_vel
            sample_num = int(seg_time * self.cmd_hz) + 1
            seg_vel = self.move_vel * seg_unit_dir
            seg_acc = np.zeros(3)

            des_pos = np.linspace(seg_start, seg_end, sample_num)[1:]
            des_vel = np.tile(seg_vel, (sample_num - 1, 1))
            des_acc = np.tile(seg_acc, (sample_num - 1, 1))

            seg_des_state = np.zeros((sample_num - 1, 3, self.D))  # 3*D: [pos, vel, acc].T * D

            seg_des_state[:, 0, :] = des_pos
            seg_des_state[:, 1, :] = des_vel
            seg_des_state[:, 2, :] = des_acc

            des_state.append(seg_des_state)

        des_state = np.concatenate(des_state, axis=0)
        
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
