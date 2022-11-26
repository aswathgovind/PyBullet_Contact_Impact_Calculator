import pybullet as p

from Utils.Ur5e_Robot_State import *

import pybullet_data
import numpy as np
import math
import pybullet_utils.bullet_client as bc

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True)
Robot = p.loadURDF("Support/Ur5e_Urdf/urdf/UR_SSR.urdf", [0, 0, 0.5], useFixedBase=True,flags=p.URDF_USE_SELF_COLLISION)
p.resetBasePositionAndOrientation(Robot, [0, 0, 0], [0, 0, 0, 1])


CollisionList_init = [[0.370,0.05,0,1.5,3.5,1,0]] #,[0.600,0,0,1.5,3.5,1,0],[0.700,0,0,1.5,3.5,1,0]]

ConstraintLoadList = []
CollisionLoadList = []

# for i in range(p.getNumJoints(Robot)):
#     jointInfo = p.getJointInfo(Robot, i)
#     # print(jointInfo,list(range(1,7,1)))


def Init_SelfCollision():
    collisionFilterGroup =0
    collisionFilterMask = 0
    p.setCollisionFilterGroupMask(Robot, 9, collisionFilterGroup, collisionFilterMask)
    p.setCollisionFilterGroupMask(Robot, 8, collisionFilterGroup, collisionFilterMask)
    p.setCollisionFilterGroupMask(Robot, 7, collisionFilterGroup, collisionFilterMask)
    p.setCollisionFilterGroupMask(Robot, 6, collisionFilterGroup, collisionFilterMask)
    #p.setCollisionFilterGroupMask(robotID, 5, collisionFilterGroup, collisionFilterMask)
    #p.setCollisionFilterGroupMask(robotID, 4, collisionFilterGroup, collisionFilterMask)
    #p.setCollisionFilterGroupMask(robotID, 3, collisionFilterGroup, collisionFilterMask)
    #p.setCollisionFilterGroupMask(robotID, 2, collisionFilterGroup, collisionFilterMask)
    #p.setCollisionFilterGroupMask(robotID, 1, collisionFilterGroup, collisionFilterMask)
    #p.setCollisionFilterGroupMask(robotID, 0, collisionFilterGroup, collisionFilterMask)


def init_collision(CollisionList):
    for count in range(0,len(CollisionList),1):
        temp_CollisionList = CollisionList[count]
        pos = temp_CollisionList[0:3]
        orient = temp_CollisionList[3:7]

        CollisionLoadList.append(p.loadURDF("Support/Collsions_Objects/Collision1.urdf",pos,orient))
        ConstraintLoadList.append(p.createConstraint(CollisionLoadList[count], -1, -1, -1, p.JOINT_FIXED, [0, 0, 0],parentFramePosition=[0,0,0],childFramePosition=pos,childFrameOrientation=orient))

init_collision(CollisionList_init)
Init_SelfCollision()

while 1:
    Pose = get_joint_vals()
    p.stepSimulation()

    # Pose = [0,np.deg2rad(-90),np.deg2rad(190),0,0,0]
    impact_flag = 0
    p.setJointMotorControlArray(bodyIndex=Robot,
                                            controlMode=p.POSITION_CONTROL,
                                            targetPositions=Pose,
                                            jointIndices= list(range(1,7,1)),positionGains=[.1,.1,.1,.1,.1,.1])

    for count in range(0,len(CollisionLoadList),1):
       cont_pts =p.getContactPoints(Robot,CollisionLoadList[count])

       if(len(cont_pts) > 0):
        #    print("Collision")
           col_flag = 1
           curr_pos = p.getBasePositionAndOrientation(CollisionLoadList[count])
           act_pos = CollisionList_init[count]
           print(math.dist(curr_pos[0],act_pos[0:3]))
           if(math.dist(curr_pos[0],act_pos[0:3]) > 0.02):
               print("High Impact")
               impact_flag = 1
       else:
           print("no collision")
           col_flag = 0

       cont_pts1 =p.getContactPoints(Robot,Robot)

       if(len(cont_pts1) > 0):
            self_col_flag = 1
            print("Self-Collision")
       else:
        #   print("no Self-collision")
          self_col_flag = 0
        
    if(col_flag == 1 or self_col_flag == 1):
        if(impact_flag == 1):
            p.addUserDebugText("        Collision with high impact",[0.0,0.05,0],[1,0,0],3,0.1)
        else:
            p.addUserDebugText("        Collision",[0.0,0.05,0],[1,0,0],3,0.1)

    elif(col_flag == 0 and self_col_flag == 0):
        p.addUserDebugText("       No Collision",[0.0,0.05,0],[0,1,0],3,0.1)

