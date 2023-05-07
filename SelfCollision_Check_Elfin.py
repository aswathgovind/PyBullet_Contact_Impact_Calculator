import pybullet as p

# from Utils.Ur5e_Robot_State import *
# from Utils.Elfin5_Robot_State import get_joint_vals
import pybullet_data
import numpy as np

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True)

Robot = p.loadURDF("Support\Eflin5\elfin5_ssr3.urdf", [0, 0, 0.5], useFixedBase=True,flags=p.URDF_USE_SELF_COLLISION)
p.resetBasePositionAndOrientation(Robot, [0, 0, 0], [0, 0, 0, 1])

# for i in range(p.getNumJoints(Robot)):
#     jointInfo = p.getJointInfo(Robot, i)
#     print(jointInfo)

def Init_SelfCollision():
    # When run without filters there the only collision it say is between 
    p.setCollisionFilterPair(0,0,7,9,0)

Init_SelfCollision()

# Init pose movement should avoid collision detection
while 1:
    # Pose = get_joint_vals(np) 
    Pose = [0,np.deg2rad(-90),np.deg2rad(90),0,0,0] 
    p.stepSimulation()

    # Pose = [0,np.deg2rad(-90),np.deg2rad(190),0,0,0]

    impact_flag = 0

    p.setJointMotorControlArray(bodyIndex=Robot,
                                            controlMode=p.POSITION_CONTROL,
                                            targetPositions=Pose,
                                            jointIndices= list(range(2,8,1)),positionGains=[.1,.1,.1,.1,.1,.1])

    cont_pts1 =p.getContactPoints(Robot,Robot)
    # print("Self-Collision",cont_pts1)

    if(len(cont_pts1) > 0):
        self_col_flag = 1
    else:
        self_col_flag = 0

    if(self_col_flag == 1):
        p.addUserDebugText("       Self - Collision",[0.0,0.05,0],[1,0,0],3,0.1)
        cont_pts1 = ()

    elif(self_col_flag == 0):
        p.addUserDebugText("       No Collision",[0.0,0.05,0],[0,1,0],3,0.1)
        cont_pts1 = ()