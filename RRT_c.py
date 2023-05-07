import numpy as np
from scipy.spatial.distance import euclidean

class RRTConnect:
    def __init__(self, start, goal, obstacles, step_size=5, max_iter=1000, threshold=0.1):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.step_size = step_size
        self.max_iter = max_iter
        self.threshold = threshold

    def sample_point(self):
        return np.random.uniform(low=-np.pi, high=np.pi, size=6)

    def find_closest_node(self, tree, point):
        distances = [euclidean(node, point) for node in tree]
        closest_node_idx = np.argmin(distances)
        return closest_node_idx, tree[closest_node_idx]

    def check_collision(self, point):
        return False

    def steer(self, from_point, to_point):
        distance = euclidean(from_point, to_point)
        if distance < self.step_size:
            return to_point
        else:
            direction = (to_point - from_point) / distance
            return from_point + direction * self.step_size

    def plan(self):
        start_tree = [self.start]
        goal_tree = [self.goal]

        for i in range(self.max_iter):
            if i % 2 == 0:
                random_point = self.sample_point()
                _, closest_node = self.find_closest_node(start_tree, random_point)
                new_point = self.steer(closest_node, random_point)
                if not self.check_collision(new_point):
                    start_tree.append(new_point)
                    print(start_tree)
                    _, closest_goal = self.find_closest_node(goal_tree, new_point)
                    distance_to_goal = euclidean(new_point, closest_goal)
                    if distance_to_goal < self.threshold:
                        path = [new_point]
                        while closest_node != self.start:
                            _, closest_node = self.find_closest_node(start_tree, closest_node)
                            path.append(closest_node)
                        path.reverse()
                        path.extend(goal_tree)
                        return path

            else:
                random_point = self.sample_point()
                _, closest_node = self.find_closest_node(goal_tree, random_point)
                new_point = self.steer(closest_node, random_point)
                if not self.check_collision(new_point):
                    goal_tree.append(new_point)
                    _, closest_start = self.find_closest_node(start_tree, new_point)
                    distance_to_start = euclidean(new_point, closest_start)
                    if distance_to_start < self.threshold:
                        path = [new_point]
                        while closest_node != self.goal:
                            _, closest_node = self.find_closest_node(goal_tree, closest_node)
                            path.append(closest_node)
                        path.reverse()
                        path.extend(start_tree)
                        return path

        return None

import matplotlib.pyplot as plt

start = np.array([0, 0, 0, 0, 0, 0])
goal = np.array([60,0,0,0,0,0])
obstacles = [([0.2, 0.4], [0.2, 0.4], [0.2, 0.4]), 
             ([0.6, 0.8], [0.6, 0.8], [0.6, 0.8])]

rrt = RRTConnect(start, goal, obstacles)
path = rrt.plan()

if path:
    print("Path found!")
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(start[0], start[1], start[2], marker='*', color='g', s=200)
    ax.scatter(goal[0], goal[1], goal[2], marker='*', color='r', s=200)
    for obstacle in obstacles:
        ax.plot([obstacle[0][0], obstacle[0][1], obstacle[0][1], obstacle[0][0], obstacle[0][0]],
                [obstacle[1][0], obstacle[1][0], obstacle[1][1], obstacle[1][1], obstacle[1][0]],
                [obstacle[2][0], obstacle[2][0], obstacle[2][0], obstacle[2][0], obstacle[2][0]], 'k')
    for i in range(len(path)-1):
        ax.plot([path[i][0], path[i+1][0]], [path[i][1], path[i+1][1]], [path[i][2], path[i+1][2]], 'b')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()
else:
    print("Path not found!")
