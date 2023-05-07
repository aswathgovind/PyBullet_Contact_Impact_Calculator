import pybullet as py
import pybullet_utils.bullet_client as bc

# from Utils.Ur5e_Robot_State import *

import pybullet_data
import numpy as np
import math
import time 
import pickle
from Trial_path_planner import *

p = bc.BulletClient(connection_mode=py.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True)

Robot_pos = [0, 0, 0]
Robot_ori = [0, 0, 0, 1]
Robot = p.loadURDF("Support/Ur5e_Urdf/urdf/UR_SSR.urdf", Robot_pos, useFixedBase=True,flags=p.URDF_USE_SELF_COLLISION)
p.resetBasePositionAndOrientation(Robot, Robot_pos,Robot_ori)

Table_pos = [0, 0, -0.1]
Table_ori = [0, 0, 0]
Table = p.loadURDF("Support/Collsions_Objects/Bed/Bed.urdf",Table_pos,p.getQuaternionFromEuler(Table_ori),useFixedBase=True,)

box_pos = [0.5, 0.25, -0.1]
box_orient = [0, 0, 90]
Box = p.loadURDF("Support/Collsions_Objects/Anatomy.urdf",box_pos,p.getQuaternionFromEuler(box_orient),useFixedBase=True)

box2_pos = [-0.5, 0.25, -0.1]
box2_orient = [0, 0, 90]
Box2 = p.loadURDF("Support/Collsions_Objects/Anatomy.urdf",box2_pos,p.getQuaternionFromEuler(box2_orient),useFixedBase=True)

ConstraintLoadList = []
CollisionLoadList = []

# for i in range(p.getNumJoints(Robot)):
#     jointInfo = p.getJointInfo(Robot, i)
#     print(jointInfo,list(range(1,7,1)))

def Init_SelfCollision():
    p.setCollisionFilterPair(0,0,7,9,0)
    p.setCollisionFilterPair(0,0,6,9,0)

def Check_collision(Joint_angles):
    while 1:
        # Pose = get_joint_vals() 
        p.stepSimulation()
        init_pose = Joint_angles
        # Pose = [0,np.deg2rad(-90),np.deg2rad(90),0,0,0]
        Pose = [np.deg2rad(float(init_pose[0])),np.deg2rad(float(init_pose[1])),np.deg2rad(float(init_pose[2])),np.deg2rad(float(init_pose[3])),np.deg2rad(float(init_pose[4])),np.deg2rad(float(init_pose[5]))]

        for count in list(range(1,7,1)):
            p.setJointMotorControl2(bodyIndex=Robot,jointIndex = count,controlMode=p.POSITION_CONTROL,targetPosition=Pose[count-1],force=100000000)

        for count in range(50):
            p.stepSimulation()

        cont_pts1 =p.getContactPoints(Robot,Robot)
        # print("Self-Collision",cont_pts1)

        if(len(cont_pts1) > 0):
            p.addUserDebugText("       Collision",[0.0,0.05,0],[1,0,0],3,0.1)
            return True
        else:
            cont_pts2 = p.getContactPoints(Robot,Table)
            cont_pts3 = p.getContactPoints(Robot,Box)
            cont_pts7 = p.getContactPoints(Robot,Box)
            if(len(cont_pts2) > 0 or len(cont_pts3) > 0 or len(cont_pts7) > 0):
                # p.addUserDebugText("       Collision",[0.0,0.05,0],[1,0,0],3,1)
                # time.sleep(0.5)
                # p.addUserDebugText(" ",[0.0,0.05,0],[1,0,0],3,0.1)
                return True
            else:
                # p.addUserDebugText("       NO Collision",[0.0,0.05,0],[0,1,0],3,1)
                # time.sleep(0.5)
                # p.addUserDebugText(" ",[0.0,0.05,0],[1,0,0],3,0.1)
                return False

for i in range(40):
    p.stepSimulation()

Init_SelfCollision()

for i in range(250):
    p.stepSimulation()

import math
import random

class Node:
    def __init__(self, joint_angles):
        self.joint_angles = joint_angles
        self.parent = None

class RRT:
    def __init__(self, start_config, goal_config, joint_limits, max_iter=10000, step_size=2, goal_prob=0.05):
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
        # print(q_rand)
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
        return Check_collision(q_new.joint_angles)

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
        print(math.sqrt(dist))
        return math.sqrt(dist)

# # Example usage
start_config = [-120,-100,-90,0,0,0] 
goal_config = [-120,-100,-90,75,80,300]
# joint_limits = [[-math.pi, math.pi]] * 6
joint_limits = [np.deg2rad([-360, 360]),np.deg2rad([0, -180]),np.deg2rad([-130, 130]),np.deg2rad([-360, 360]),np.deg2rad([-360, 360]),np.deg2rad([-360, 360])]
rrt = RRT(start_config, goal_config, joint_limits)
path = rrt.plan()
print(path)

if(path != None):
    # 'Path Visualisation'
    p1 = bc.BulletClient(connection_mode=py.GUI)

    from Visualiser_code import *
    # with open('servo_points', 'rb') as fp:
    #     path = pickle.load(fp)
    startPose = path[0]
    RoboMoveSim(p1,path,startPose)

    from ActualRobotMovement import *
    ActualMovement(path,a = 0.35,v = 0.25,r = 0.02)

else:
    print("Could not find path")

# while(1):
# instr = time.time()
# for count in range(750):
#     p.stepSimulation()
#     J1 = random.uniform(-360, 360)
#     J2 = random.uniform(0, -180)
#     J3 = random.uniform(-130, 130)
#     J4 = random.uniform(-360, 360)
#     J5 = random.uniform(-360, 360)
#     J6 = random.uniform(-360, 360)

#     Check_collision([J1,J2,J3,J4,J5,J6])
#     # time.sleep(0.3)


# endst = (time.time() - instr)
# print(endst)