
import math
import random
import numpy as np
class Node:
    def __init__(self, joint_angles):
        self.joint_angles = joint_angles
        self.parent = None

class RRT:
    def __init__(self, start_config, goal_config, joint_limits, max_iter=10000, step_size=5, goal_prob=0.5):
        self.start_node = Node(start_config)
        self.goal_node = Node(goal_config)
        self.joint_limits = joint_limits
        self.max_iter = max_iter
        self.step_size = step_size
        self.goal_prob = goal_prob
        self.nodes = [self.start_node]

    def plan(self):
        for i in range(self.max_iter):
            q_rand = self.random_config()
            q_near = self.nearest_neighbor(q_rand)
            q_new = self.steer(q_near, q_rand)

            if self.is_collision_free(q_near, q_new):
                self.nodes.append(q_new)
                q_new.parent = q_near

                if self.is_goal(q_new):
                    return self.extract_path(q_new)

        return None
    
    def random_config(self):
        if random.random() < self.goal_prob:
            return self.goal_node.joint_angles

        q_rand = []
        for i in range(6):
            q_rand.append(random.uniform(self.joint_limits[i][0], self.joint_limits[i][1]))

        return q_rand

    def nearest_neighbor(self, q_rand):
        min_dist = float('inf')
        nearest_node = self.nodes[0]

        for node in self.nodes:
            dist = self.distance(node.joint_angles, q_rand)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node

        return nearest_node

    def steer(self, q_near, q_rand):
        dist = self.distance(q_near.joint_angles, q_rand)
        if dist < self.step_size:
            return Node(q_rand)

        q_new = []
        for i in range(6):
            direction = q_rand[i] - q_near.joint_angles[i]
            if abs(direction) > self.step_size:
                direction = math.copysign(self.step_size, direction)
            q_new.append(q_near.joint_angles[i] + direction)

        return Node(q_new)

    def is_collision_free(self, q_near, q_new):
        # Check if the line segment connecting q_near and q_new is collision-free
        # print("q_new",q_new.joint_angles)
        return True
    def is_goal(self, q_new):
        return self.distance(q_new.joint_angles, self.goal_node.joint_angles) < 0.1

    def extract_path(self, q_new):
        path = []
        node = q_new

        while node is not None:
            path.append(node.joint_angles)
            node = node.parent

        path.reverse()
        return path

    def distance(self, q1, q2):
        dist = 0
        for i in range(6):
            dist += (q1[i] - q2[i]) ** 2
        return math.sqrt(dist)

# Example usage

# import rtde_receive
# rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.25")
# actual_q = rtde_r.getActualQ()

class RRTConnect:
    def __init__(self, start_config, goal_config, joint_limits, max_iter=10000, step_size=5, goal_prob=0.5):
        self.start_tree = RRT(start_config, goal_config, joint_limits, max_iter, step_size, goal_prob)
        self.goal_tree = RRT(goal_config, start_config, joint_limits, max_iter, step_size, goal_prob)
        self.path = None

    def plan(self):
        for i in range(self.start_tree.max_iter):
            q_rand = self.start_tree.random_config()
            q_near = self.start_tree.nearest_neighbor(q_rand)
            q_new = self.start_tree.steer(q_near, q_rand)

            if self.start_tree.is_collision_free(q_near, q_new):
                self.start_tree.nodes.append(q_new)
                q_new.parent = q_near

                q_nearest_to_new = self.goal_tree.nearest_neighbor(q_new.joint_angles)
                q_new_to_nearest = self.goal_tree.steer(q_nearest_to_new, q_new.joint_angles)

                if self.goal_tree.is_collision_free(q_nearest_to_new, q_new_to_nearest):
                    self.goal_tree.nodes.append(q_new_to_nearest)
                    q_new_to_nearest.parent = q_nearest_to_new

                    if self.start_tree.is_goal(q_new) and self.goal_tree.is_goal(q_new_to_nearest):
                        path_start = self.start_tree.extract_path(q_new)
                        path_goal = self.goal_tree.extract_path(q_new_to_nearest)
                        path_goal.reverse()
                        self.path = path_start + path_goal
                        return self.path

        return None

goal = [0,0,0,0,0,0]   #
# start_config = np.rad2deg([0.09142494201660156, -0.19819052637133794, 0.49121362367738897, -2.3806711635985316, -0.22527820268739873, -0.2640135923968714])
start_config = [0,0,0,0,0,60]
goal_config = goal

joint_limits = [np.deg2rad([-360, 360]),np.deg2rad([0, -180]),np.deg2rad([-130, 130]),np.deg2rad([-360, 360]),np.deg2rad([-360, 360]),np.deg2rad([-360, 360])]
rrt = RRTConnect(start_config, goal_config, joint_limits)
path = rrt.plan()
print(path)
