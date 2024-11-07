import math
import matplotlib.pyplot as plt


class AstarPlanner():

    def __init__(self, a_star_config):
        self.motion = self.get_motion_model()
        self.root_parent_index = -1  # The parent of the start node. No actual meaning, just to avoid duplicate index
        self.resolution = a_star_config.resolution
        self.min_flight_height = a_star_config.min_flight_height
        self.max_flight_height = a_star_config.max_flight_height

    class Node:
        def __init__(self, index, cost, parent_index):
            self.index = index  # a 3-element tuple (x, y), note that this is not the real position
            self.cost = cost  # the cost in defined in grid, from the start node to the current node, not including the heuristic cost
            self.parent_index = parent_index  # a 3-element tuple (x, y)

    def plan(self, map, start_pos, target_pos):
        '''
        input: start_pos, target_pos: 3-element list [x, y, z]
        return a path: a list of 3-element list, [start_pos, [x1, y1, z1], ... , target_pos]
        '''
        # read map info
        self.map = map  # the map class is defined in src/planner/scripts/map_server/pcl_server.py

        # Check if the straight line from start_pos to target_pos is feasible.
        if self.map.seg_feasible_check(start_pos, target_pos):
            return [start_pos, target_pos]

        # read mission info
        start_node_index = self.calc_index(start_pos)
        start_node = self.Node(start_node_index, 0.0, self.root_parent_index)
        target_node_index = self.calc_index(target_pos)
        self.target_node = self.Node(target_node_index, 0.0, self.root_parent_index)

        # open and close set
        open_set, close_set = dict(), dict()
        open_set[start_node_index] = start_node

        # search
        while True:
            if not open_set:
                print("Open set is empty, no path found")
                return None

            # get the current_node node
            current_node_index = min(open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(open_set[o], self.target_node))
            current_node = open_set[current_node_index]

            # check if the current_node node is the target node
            if current_node.index == self.target_node.index:
                self.target_node.parent_index = current_node.parent_index
                self.target_node.cost = current_node.cost
                break

            # move the current_node node to the close set
            del open_set[current_node_index]
            close_set[current_node_index] = current_node

            # expand the current_node node
            for move_x, move_y, move_z, move_cost in self.motion:
                candidate_node_index = (int(current_node.index[0] + move_x),
                                        int(current_node.index[1] + move_y),
                                        int(current_node.index[2] + move_z))
                candidate_node = self.Node(candidate_node_index, current_node.cost + move_cost, current_node_index)

                # check if the candidate_node is valid
                if candidate_node_index in close_set:
                    continue

                if not self.verify_node(candidate_node):
                    continue

                # update the node in open set
                if candidate_node_index not in open_set or open_set[candidate_node_index].cost > candidate_node.cost:
                    open_set[candidate_node_index] = candidate_node

        # retrieve the path
        path = self.retrieve_final_path(close_set)

        # self.visualize_path(path)
        return path

    def get_motion_model(self):
        # dx, dy, dz, cost
        motion = [
            [1, 0, 0, 1],
            [0, 1, 0, 1],
            [0, 0, 1, 1],
            [-1, 0, 0, 1],
            [0, -1, 0, 1],
            [0, 0, -1, 1],
            [1, 1, 0, math.sqrt(2)],
            [1, -1, 0, math.sqrt(2)],
            [-1, 1, 0, math.sqrt(2)],
            [-1, -1, 0, math.sqrt(2)],
            [1, 0, 1, math.sqrt(2)],
            [1, 0, -1, math.sqrt(2)],
            [-1, 0, 1, math.sqrt(2)],
            [-1, 0, -1, math.sqrt(2)],
            [0, 1, 1, math.sqrt(2)],
            [0, 1, -1, math.sqrt(2)],
            [0, -1, 1, math.sqrt(2)],
            [0, -1, -1, math.sqrt(2)],
            [1, 1, 1, math.sqrt(3)],
            [1, 1, -1, math.sqrt(3)],
            [1, -1, 1, math.sqrt(3)],
            [1, -1, -1, math.sqrt(3)],
            [-1, 1, 1, math.sqrt(3)],
            [-1, 1, -1, math.sqrt(3)],
            [-1, -1, 1, math.sqrt(3)],
            [-1, -1, -1, math.sqrt(3)]
        ]
        return motion

    def calc_real_pos(self, node_index):
        '''
        return a 3-element list [x, y, z]: the real position of the node
        '''
        return [node_index[0]*self.resolution, node_index[1]*self.resolution, node_index[2]*self.resolution]

    def calc_index(self, pos):
        return (int(pos[0]/self.resolution), int(pos[1]/self.resolution), int(pos[2]/self.resolution))

    @staticmethod
    def calc_heuristic(node1, node2):
        dx = node1.index[0] - node2.index[0]
        dy = node1.index[1] - node2.index[1]
        dz = node1.index[2] - node2.index[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def collision_check(self, node):
        # return True if the node is in collision
        node_3d_pos = self.calc_real_pos(node.index)
        return self.map.has_collision(node_3d_pos)

    def verify_node(self, node):
        # Return True if the node is valid
        if node.index[2] < self.min_flight_height / self.resolution:
            return False
        if node.index[2] > self.max_flight_height / self.resolution:
            return False
        if self.collision_check(node):
            return False
        return True

    def retrieve_final_path(self, close_set):
        path = []
        node = self.target_node
        while node.parent_index != self.root_parent_index:
            path.append(self.calc_real_pos(node.index))
            node = close_set[node.parent_index]
        path.append(self.calc_real_pos(node.index))  # append the start node
        return path[::-1]

    def visualize_path(self, path):
        plt.plot([x[0] for x in path], [x[1] for x in path], 'r-')
        plt.axis("equal")
        plt.show()
