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

data = np.load('recorded_jointangles.npz')
obsPose = data['car_pose']

Robot_pos = [0, 0, 0]
Robot_ori = [0, 0, 0, 1]
Robot = p.loadURDF("Support/Ur5e_Urdf/urdf/UR_SSR.urdf", Robot_pos, useFixedBase=True,flags=p.URDF_USE_SELF_COLLISION)
p.resetBasePositionAndOrientation(Robot, Robot_pos,Robot_ori)

Table_pos = [0, 0, -0.1]
Table_ori = [0, 0, 0]
Table = p.loadURDF("Support/Collsions_Objects/Bed/Bed.urdf",Table_pos,p.getQuaternionFromEuler(Table_ori),useFixedBase=True,)

box_pos = obsPose[0]
box_orient = [0, 0, 90]
Box = p.loadURDF("Support/Collsions_Objects/Anatomy.urdf",[-box_pos[0],-box_pos[1],box_pos[2]],p.getQuaternionFromEuler(box_orient),useFixedBase=True)

box2_pos = obsPose[1]
box2_orient = [0, 0, 90]
Box2 = p.loadURDF("Support/Collsions_Objects/Anatomy.urdf",[-box2_pos[0],-box2_pos[1],box2_pos[2]],p.getQuaternionFromEuler(box2_orient),useFixedBase=True)

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
            return False
        else:
            cont_pts2 = p.getContactPoints(Robot,Table)
            cont_pts3 = p.getContactPoints(Robot,Box)
            cont_pts7 = p.getContactPoints(Robot,Box)
            if(len(cont_pts2) > 0 or len(cont_pts3) > 0 or len(cont_pts7) > 0):
                p.addUserDebugText("       Collision",[0.0,0.05,0],[1,0,0],3,0.1)
                return False
            else:
                p.addUserDebugText("       NO Collision",[0.0,0.05,0],[0,1,0],3,0.1)
                return True

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
        return math.sqrt(dist)

# Example usage

# import rtde_receive
# rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.25")
# actual_q = rtde_r.getActualQ()

goal = [0,-90,-128,-90,0,0]   #
start_config = np.rad2deg([0.09142494201660156, -0.19819052637133794, 0.49121362367738897, -2.3806711635985316, -0.22527820268739873, -0.2640135923968714])
# start_config = [0,-90,-170,-90,0,0]
goal_config = goal

joint_limits = [np.deg2rad([-360, 360]),np.deg2rad([0, -180]),np.deg2rad([-130, 130]),np.deg2rad([-360, 360]),np.deg2rad([-360, 360]),np.deg2rad([-360, 360])]
rrt = RRT(start_config, goal_config, joint_limits)
path = rrt.plan()
print(path)

'Path Visualisation'
p1 = bc.BulletClient(connection_mode=py.GUI)

from Visualiser_code import *
startPose = path[0]
RoboMoveSim(p1,path,startPose)


flag = int(input("Move robot:-"))
if(flag == 1):
    from ActualRobotMovement import *
    ActualMovement(path,a = 0.35,v = 0.25,r = 0.02)