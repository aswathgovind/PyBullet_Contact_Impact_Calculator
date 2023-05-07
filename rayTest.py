import numpy as np
import pybullet as pyBulletObj
import pybullet_utils.bullet_client as bc
import math

data = np.load('recorded_jointangles.npz')
obsPose = data['car_pose']
p1 = bc.BulletClient(connection_mode=pyBulletObj.GUI)

Robot_pos = [0, 0, 0]
Robot_ori = [0, 0, 0, 1]
# Robot = pyBulletObj.loadURDF("Support/Ur5e_Urdf/urdf/UR_SSR.urdf", Robot_pos, useFixedBase=True,flags=pyBulletObj.URDF_USE_SELF_COLLISION)
# pyBulletObj.resetBasePositionAndOrientation(Robot, Robot_pos,Robot_ori)

Table_pos = [0, 0, -0.1]
Table_ori = [0, 0, 0]
Table = pyBulletObj.loadURDF("Support/Collsions_Objects/Bed/Bed.urdf",Table_pos,pyBulletObj.getQuaternionFromEuler(Table_ori),useFixedBase=True,)

box_pos = obsPose[0]
box_orient = [0, 0, 90]
Box = pyBulletObj.loadURDF("Support/Collsions_Objects/Anatomy.urdf",[-box_pos[0],-box_pos[1],box_pos[2]],pyBulletObj.getQuaternionFromEuler(box_orient),useFixedBase=True)

box2_pos = obsPose[1]
box2_orient = [0, 0, 90]
Box2 = pyBulletObj.loadURDF("Support/Collsions_Objects/Anatomy.urdf",[-box2_pos[0],-box2_pos[1],box_pos[2]-0.25],pyBulletObj.getQuaternionFromEuler(box2_orient),useFixedBase=True)

box3_pos = obsPose[1]
box3_orient = [0, 0, 90]
Box3 = pyBulletObj.loadURDF("Support/Collsions_Objects/Anatomy.urdf",[-box2_pos[0]+0.05,-box2_pos[1]+0.5,box_pos[2]-0.25],pyBulletObj.getQuaternionFromEuler(box2_orient),useFixedBase=True)


#RayTest
rayFrom = []
rayTo = []
rayIds = []

numRays = 150

rayLen = 2

rayHitColor = [1, 0, 0]
rayMissColor = [0, 1, 0]

replaceLines = True

for i in range(numRays):
    rayFrom.append([-box_pos[0],-box_pos[1],box_pos[2]])
    n = 2
    # rotmat = p.

    rayTo.append([
        rayLen * math.sin(n * math.pi * float(i) / numRays),
        rayLen * math.cos(n * math.pi * float(i) / numRays), box_pos[2]
    ])

    if (replaceLines):
        rayIds.append(pyBulletObj.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor))
    else:
        rayIds.append(-1)

while(1):
    pyBulletObj.stepSimulation()
    # for count in list(range(1,7,1)):
    #     pyBulletObj.setJointMotorControl2(bodyIndex=Robot,jointIndex = count,controlMode=pyBulletObj.POSITION_CONTROL,targetPosition=np.deg2rad(startPose[count-1]),force=100000000)

    
    for j in range(8):
        results = pyBulletObj.rayTestBatch(rayFrom, rayTo, Box2)

  #for i in range (10):
  #	p.removeAllUserDebugItems()

    for i in range(numRays):
      hitObjectUid = results[i][0]

      if (hitObjectUid < 0):
        hitPosition = [0, 0, 0]
        pyBulletObj.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor, replaceItemUniqueId=rayIds[i])
      else:
        hitPosition = results[i][3]
        pyBulletObj.addUserDebugLine(rayFrom[i], hitPosition, rayHitColor, replaceItemUniqueId=rayIds[i])
        # print("HIT",hitPosition)
    
    for j in range(8):
        results = pyBulletObj.rayTestBatch(rayFrom, rayTo, Box3)

  #for i in range (10):
  #	p.removeAllUserDebugItems()

    for i in range(numRays):
      hitObjectUid = results[i][0]

      if (hitObjectUid < 0):
        hitPosition = [0, 0, 0]
        pyBulletObj.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor, replaceItemUniqueId=rayIds[i])
      else:
        hitPosition = results[i][3]
        pyBulletObj.addUserDebugLine(rayFrom[i], hitPosition, rayHitColor, replaceItemUniqueId=rayIds[i])
        # print("HIT",hitPosition)