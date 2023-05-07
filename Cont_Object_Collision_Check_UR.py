import pybullet as p

# from Utils.Ur5e_Robot_State import *

import pybullet_data
import numpy as np
import math
from skspatial.objects import Plane
import transforms3d as t
from Utils.setupGUI_params import *
# from timeit import default_timer as timer

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

DEBUG_FLAG = True

ConstraintLoadList = []
CollisionLoadList = []
LineList = []

# for i in range(p.getNumJoints(Robot)):
#     jointInfo = p.getJointInfo(Robot, i)
#     # print(jointInfo,list(range(1,7,1)))

def MMtoMetersinPybullet(Position):
    return [-Position[0]/1000,-Position[1]/1000,Position[2]/1000]

def Vec2UnitVec(vec):
    norm = np.sqrt( np.square(vec[0]) + np.square(vec[1]) + np.square(vec[2]))
    vector = vec / norm
    return vector

def d2r(deg):
    return np.deg2rad(deg)

def FormVector(StartpntOfVec,EndpntOfVec):
    Vector = np.asarray(EndpntOfVec) - np.asarray(StartpntOfVec)
    return Vector

def retreive_fin_orientation(entry,target):
    needleTrajVect = np.asarray(target) - np.asarray(entry)
    denom = np.sqrt( np.square(needleTrajVect[0]) + np.square(needleTrajVect[1]) + np.square(needleTrajVect[2]))
    UnitneedleTrajVect_zminus = -(needleTrajVect/ denom)
    latentPlane = Plane(entry,Vec2UnitVec(needleTrajVect))
    PlanePoints = latentPlane.to_points()
    #   print(PlanePoints)

    UnitneedleTrajVect_y = Vec2UnitVec(FormVector(entry,PlanePoints[0]))

    UnitneedleTrajVect_x = np.cross(UnitneedleTrajVect_y,UnitneedleTrajVect_zminus)

    OrientMat = np.vstack((Vec2UnitVec(UnitneedleTrajVect_x),Vec2UnitVec(UnitneedleTrajVect_y),Vec2UnitVec(UnitneedleTrajVect_zminus)))
    #   print(OrientMat)
    OrientMat = np.transpose(OrientMat)
    #   print(OrientMat)
    #   print("Det2",np.linalg.det(OrientMat))
    startOrientationfromTranform = t.quaternions.mat2quat(OrientMat)

    return startOrientationfromTranform[1],startOrientationfromTranform[2],startOrientationfromTranform[3],startOrientationfromTranform[0]

def Pose_trans4_Pybullet(position):
    print("pose vs transformed pose:=",position, [-(position[0]),-(position[1]),(position[2])])
    return [-(position[0]),-(position[1]),(position[2])]

def init_implant_collision(pyBulletObj,EntryList,TargetList):
    for count in range(0,len(EntryList),1):
        pos = EntryList[count]
        OrientCol = retreive_fin_orientation([-EntryList[count][0]/1000,-EntryList[count][1]/1000,EntryList[count][2]/1000],[-TargetList[count][0]/1000,-TargetList[count][1]/1000,TargetList[count][2]/1000])
        CollisionLoadList.append(pyBulletObj.loadURDF("Support/Collsions_Objects/Implant.urdf",pos,OrientCol)) 
        # ConstraintList.append(pyBulletObj.createConstraint(CollisionList[count], -1, -1, -1, pyBulletObj.JOINT_FIXED, [0, 0, 0],parentFramePosition=[0,0,0],childFramePosition=[-EntryList[count][0]/1000,-EntryList[count][1]/1000,EntryList[count][2]/1000]))
        ConstraintLoadList.append(pyBulletObj.createConstraint(CollisionLoadList[count], -1, -1, -1, pyBulletObj.JOINT_FIXED, [0, 0, 0],parentFramePosition=[0,0,0],childFramePosition=[-EntryList[count][0]/1000,-EntryList[count][1]/1000,EntryList[count][2]/1000],childFrameOrientation=OrientCol))

        LineList.append(pyBulletObj.addUserDebugLine(lineFromXYZ=MMtoMetersinPybullet(EntryList[count]),
                                lineToXYZ=MMtoMetersinPybullet(TargetList[count]),
                                lineColorRGB=[1,0,0],
                                lineWidth=0.05,
                                lifeTime=0))
        pyBulletObj.stepSimulation()

    for step in range(1500):
        pyBulletObj.stepSimulation()

def removeExistingLines(p,LinesList):
    for count in range(0,len(LinesList),1):
        p.removeUserDebugItem(LineList[count])
    return []

def init_Other_collision(pyBulletObj,poseList):
    CollisionLoadList,ConstraintLoadList  = [],[]

    for count in range(0,len(poseList)+1,1):

        if(count < 3):
            curr_pose = poseList[count]
            postion = curr_pose[0:3]
            orient = curr_pose[3:6]

        if(count == 0):
            postion = [-0.40,0.110,0.0]
            orient = [120,0,270]
            CollisionLoadList.append(pyBulletObj.loadURDF("Support/Collsions_Objects/Anatomy.urdf",Pose_trans4_Pybullet(postion),p.getQuaternionFromEuler(orient)))
        elif(count == 1):
            postion = [-0.55,0.110,-0.2010]
            orient = [90,0,360]
            CollisionLoadList.append(pyBulletObj.loadURDF("Support/Collsions_Objects/Collision1.urdf",Pose_trans4_Pybullet(postion),p.getQuaternionFromEuler(orient)))
        elif(count == 2):
            postion = [-0.30,0.110,0.0]
            orient = [0,0,300]
            CollisionLoadList.append(pyBulletObj.loadURDF("Support/Collsions_Objects/Anatomy.urdf",Pose_trans4_Pybullet(postion),p.getQuaternionFromEuler(orient)))
        elif(count == 3):
            postion = [-0.40,0.110,0.0]
            orient = [0,0,300]
            CollisionLoadList.append(pyBulletObj.loadURDF("Support/Collsions_Objects/Bed/Bed.urdf",postion,p.getQuaternionFromEuler(orient)))
        #ConstraintList.append(pyBulletObj.createConstraint(CollisionList[count], -1, -1, -1, pyBulletObj.JOINT_FIXED, [0, 0, 0],parentFramePosition=[0,0,0],childFramePosition=[-EntryList[count][0]/1000,-EntryList[count][1]/1000,EntryList[count][2]/1000]))
        if(count != 2):
            ConstraintLoadList.append(pyBulletObj.createConstraint(CollisionLoadList[count], -1, -1, -1, pyBulletObj.JOINT_FIXED, [0, 0, 0],parentFramePosition=[0,0,0],childFramePosition=Pose_trans4_Pybullet(postion),childFrameOrientation=p.getQuaternionFromEuler(orient)))

    return CollisionLoadList

def ApplyCollisionBetween(On,FromList):
    for count in range(0,len(FromList),1):
        p.setCollisionFilterPair(On,FromList[count],-1,-1,0)

def CollisionFilter(Needle_ID_List,Object_ID_List):
    for count in range(0,len(Object_ID_List),1):
        for count_i in range(0,len(Needle_ID_List),1):
            p.setCollisionFilterPair(Object_ID_List[count],Needle_ID_List[count_i],-1,-1,0)
        ApplyCollisionBetween(Object_ID_List[count],Object_ID_List)

def DrawDebugLines(p,start_point,end_point):
    start_point_onA = [start_point[0],start_point[1],start_point[2]]
    end_point_onB = [end_point[0],end_point[1],end_point[2]]

    p.addUserDebugLine(lineFromXYZ=start_point_onA,
                                lineToXYZ=end_point_onB,
                                lineColorRGB=[1,0,0],
                                lineWidth=0.05,
                                lifeTime=0)

EntryList = [[-512.781, -70.2329907681076, 211.7073958495319],
[-587.556, -88.974, 217.594],
# [-586.4246886928735, -129.5155046159462, 211.17239584953194],
# [-508.9554953840538, -73.72031130712645, 144.51839584953194],
[-549.032, -89.44052658050256, 147.9469468389949],
[-531.1165265805026, -248.67900000000003, 79.3229468389949]]

TargetList = [[-512.781,-100.256, 99.66],
[-587.556, -88.974, 101.594],
# [-560.424, -114.504, 99.125],
# [-523.967, -99.721, 32.471000000000004],
[-578.032,-139.67, 47.488],
[-581.346, -219.679, -21.135999999999996]]

# print(len(EntryList),len(TargetList))

init_implant_collision(p,EntryList,TargetList)
data = np.load('recorded_jointangles.npz')
print(data['car_pose'])
CollisionList = data['car_pose']

OtherCollisionLoadList = init_Other_collision(p,CollisionList)

Robot = p.loadURDF("Support/Ur5e_Urdf/urdf/UR_SSR.urdf", [0, 0, 0.5], useFixedBase=True,flags=p.URDF_USE_SELF_COLLISION)
p.resetBasePositionAndOrientation(Robot, [0, 0, 0], [0, 0, 0, 1])

# print("list",CollisionLoadList,OtherCollisionLoadList)
CollisionFilter(Needle_ID_List = CollisionLoadList,Object_ID_List = OtherCollisionLoadList)

# Init pose movement should avoid collision detection
connect_flag = p.addUserDebugParameter("replicate real robot",360,-360,-358)
sliderIds=setup_joint_sliders_UR(p)
poseSliderID = setup_pose_slider(p)

def Init_SelfCollision():
    p.setCollisionFilterPair(Robot,Robot,7,9,0)
    p.setCollisionFilterPair(Robot,Robot,6,9,0)

Init_SelfCollision()

while 1:
    connect = p.readUserDebugParameter(connect_flag)
    # print(connect)
    if(connect % 2 == 1):
        # Pose = get_joint_vals()
        Pose =[np.deg2rad(0),np.deg2rad(-90),np.deg2rad(90),np.deg2rad(0),np.deg2rad(0),np.deg2rad(0)]
    else:
        Pose = joint_vals_from_sliders(np,p,sliderIds)

    anatomy_pose = get_pose_slider_val(p,poseSliderID)
    
    p.resetBasePositionAndOrientation(OtherCollisionLoadList[2],anatomy_pose,p.getQuaternionFromEuler([0,0,300]))

    # start = timer()
    # ...
    p.stepSimulation()
    # end = timer()
    # print(end - start)    


    impact_flag = 0
    p.setJointMotorControlArray(bodyIndex=Robot,
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=Pose,
                                jointIndices= list(range(1,7,1)),positionGains=[.1,.1,.1,.1,.1,.1])

    col_flags = ''

    for count in range(0,len(CollisionLoadList),1):
       cont_pts =p.getContactPoints(Robot,CollisionLoadList[count])

       if(len(cont_pts) > 0):
            col_flags = col_flags + '1'
    
    for count in range(0,len(CollisionLoadList),1):
       cont_pts =p.getContactPoints(Robot,Robot)

       if(len(cont_pts) > 0):
            col_flags = col_flags + '1'
    
    for count in range(0,len(OtherCollisionLoadList),1):
       cont_pts =p.getContactPoints(Robot,OtherCollisionLoadList[count])
    #    print(OtherCollisionLoadList[count],Robot)
       if(len(cont_pts) > 0):
            col_flags = col_flags + '1'

    if(len(col_flags) > 0):
        p.addUserDebugText("        Collision",[0.0,0.05,0],[1,0,0],3,0.1)
        col_flags = ''
    else:
        p.addUserDebugText("       No Collision",[0.0,0.05,0],[0,1,0],3,0.1)
        col_flags = ''

        # (0, 7, 0, 9, -1, (0.816704733355519, 0.3820360265970122, 0.08094299804457913), (0.5136138371556384, 0.05874515062930882, 0.2448639046759159), (0.6414695474943128, 0.6842213161667519, -0.34692651979991246), 0.47249459835436347, 0.0, 0.0, (0.0, 0.0, 0.0), 0.0, (0.0, 0.0, 0.0))