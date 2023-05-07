
import time 
import numpy as np

def RoboMoveSim(pyBulletObj, Movement_cmd,startPose):
    CollisionList = []
    ConstraintList = []
    listPos = []

    data = np.load('recorded_jointangles.npz')
    obsPose = data['car_pose']

    Robot_pos = [0, 0, 0]
    Robot_ori = [0, 0, 0, 1]
    Robot = pyBulletObj.loadURDF("Support/Ur5e_Urdf/urdf/UR_SSR.urdf", Robot_pos, useFixedBase=True,flags=pyBulletObj.URDF_USE_SELF_COLLISION)
    pyBulletObj.resetBasePositionAndOrientation(Robot, Robot_pos,Robot_ori)

    Table_pos = [0, 0, -0.1]
    Table_ori = [0, 0, 0]
    Table = pyBulletObj.loadURDF("Support/Collsions_Objects/Bed/Bed.urdf",Table_pos,pyBulletObj.getQuaternionFromEuler(Table_ori),useFixedBase=True,)

    box_pos = obsPose[0]
    box_orient = [0, 0, 90]
    Box = pyBulletObj.loadURDF("Support/Collsions_Objects/Anatomy.urdf",[-box_pos[0],-box_pos[1],box_pos[2]],pyBulletObj.getQuaternionFromEuler(box_orient),useFixedBase=True)

    box2_pos = obsPose[1]
    box2_orient = [0, 0, 90]
    Box2 = pyBulletObj.loadURDF("Support/Collsions_Objects/Anatomy.urdf",[-box2_pos[0],-box2_pos[1],box2_pos[2]],pyBulletObj.getQuaternionFromEuler(box2_orient),useFixedBase=True)

    flag = False
    for step in range(200):
        pyBulletObj.stepSimulation()

    while(1):
        if(flag == False):
            for step in range(200):
                pyBulletObj.stepSimulation()

            # pyBulletObj.setJointMotorControlArray(bodyIndex=Robot,
            #                                 controlMode=pyBulletObj.POSITION_CONTROL,
            #                                 targetPositions=np.deg2rad(startPose),
            #                                 jointIndices= list(range(1,7,1)),positionGains=[0.001,.001,.001,.001,.001,.001])
            for count in list(range(1,7,1)):
                pyBulletObj.setJointMotorControl2(bodyIndex=Robot,jointIndex = count,controlMode=pyBulletObj.POSITION_CONTROL,targetPosition=np.deg2rad(startPose[count-1]),force=100000000)
                print("from bullet",np.deg2rad(startPose[count-1]))
            for step in range(2000):
                pyBulletObj.stepSimulation()
            # time.sleep(5)
            # for count in range(0,len(Movement_cmd),1):
            #     pyBulletObj.stepSimulation()
            #     pyBulletObj.setJointMotorControlArray(bodyIndex=Robot,
            #                                 controlMode=pyBulletObj.POSITION_CONTROL,
            #                                 targetPositions=np.deg2rad(Movement_cmd[count]),
            #                                 jointIndices= list(range(1,7,1)),positionGains=[0.0001,.0001,.0001,.0001,.0001,.0001])
            #     for step in range(550):
            #         pyBulletObj.stepSimulation()
            for i in Movement_cmd:  
                for count in list(range(1,7,1)):
                    tp = np.deg2rad(i[count-1])
                    pyBulletObj.setJointMotorControl2(bodyIndex=Robot,jointIndex = count,controlMode=pyBulletObj.POSITION_CONTROL,targetPosition=tp,force=100000000)
                    print("from bullet",tp,i[count-1])
                    for step in range(10):
                        pyBulletObj.stepSimulation()
                print(i)

                time.sleep(10./240.)
            for step in range(1000):
                pyBulletObj.stepSimulation()

        flag = True
        for step in range(1):
            pyBulletObj.stepSimulation()

        time.sleep(5)
        break 

    # time.sleep(5./240.)
    CollisionList.clear()
    ConstraintList.clear()
    listPos.clear()
    pyBulletObj.disconnect()
    return False