import socket
import os
import math
import time


# #-------------------Math and Robot Modules
# ip = "192.168.2.1"
ip = "192.168.11.11"
port = int("10003")


#---------------------------------------------------------------------------------------------------------
#Modules to Make Motion  of the Robot with respect to the Robot
def connect(ip = "192.168.0.10", port = 10003):

    try:
        s = socket.socket()
        s.connect((ip, int(port)))
        print('robot connected succesfully to')
        print('ip - {}'.format(ip), '\n', 'port - {}'.format(port))
        return s
    except Exception as e:
        print(e)
        print("cant retrive the Actual Position of the Robot")

def TurnOff(s,port = 10003, OffRestart_Flag = 1, robotID = 0):
    """To restart or switch off the robot 

    Args:
        s ([Object]): [Soccet communication]
        port (int, optional): [Unique Port number]. Defaults to 10003.
        robotID (int, optional): [The unique identifier of the Robot]. Defaults to 0.
        OffRestart_Flag (int, mandatory): parameter to turn off ot restart the controller

    Returns:
        [list of Floats]: [Returns the EE position of the robot if successful]
        [String]: [if the command is unsuccesful]
        
    """
    try:
        
    
        data = str("OSCmd,{},;\n".format(robotID))
        data = data.encode()
        s.send(data)
        result= s.recv(port).decode()
        cut=result.split(',')
        if(cut[1]=="OK"):
            return list(map(float,cut[0 + 2: 6 + 2]))
        elif(cut[1]=="Fail"):
            print(result)
    except Exception as e:
        print(e)
        print("cant retrive the Actual Position of the Robot") 

def GetRobotState(s,port = 10003,robotID = 0):

    try:    
        data = str("ReadRobotState,{},;\n".format(robotID))
        data = data.encode()
        s.send(data)
        result= s.recv(port).decode()
        cut=result.split(',')
        if(cut[1]=="OK"):
            return cut
        elif(cut[1]=="Fail"):
            print(result)

    except Exception as e:
        print(e)
        print("cant retrive the Actual Position of the Robot")

def GetRobotState(s,port = 10003,robotID = 0):

    try:    
        data = str("ReadRobotState,{},;\n".format(robotID))
        data = data.encode()
        s.send(data)
        result= s.recv(port).decode()
        cut=result.split(',')
        if(cut[1]=="OK"):
            return cut
        elif(cut[1]=="Fail"):
            print(result)

    except Exception as e:
        print(e)
        print("cant retrive the Actual Position of the Robot")  

def getj(s,port = 10003, robotID = 0):

    try:
        
    
        data = str("ReadActPos,{},;\n".format(robotID))
        data = data.encode()
        s.send(data)
        result= s.recv(port).decode()
        cut=result.split(',')
        if(cut[1]=="OK"):
            return list(map(float,cut[0 + 2: 6 + 2]))
        elif(cut[1]=="Fail"):
            print(result)
    except Exception as e:
        print(e)
        print("cant retrive the Actual Position of the Robot") 


def getl(s,port = 10003,robotID = 0):

    try:
        data = str("ReadActPos,{},;\n".format(robotID))
        data = data.encode()
        s.send(data)
        result= s.recv(port).decode()
        cut=result.split(',')
        if(cut[1]=="OK"):
            return list(map(float,cut[6 + 2:12 + 2]))
        elif(cut[1]=="Fail"):
            print(result)
    except Exception as e:
        print(e)
        print("cant retrive the Actual Position of the Robot") 

# WayPoint,0,420,0,445,180,0,180,0,0,90,0,90,0,TCP,Base,50,360,0,0,1,0,0,0,0,;

def WayPoint_J(s,angles, port=10003, robotID=0):

    l = angles
    # p = pose
    try:
        
        # data = "WayPoint,0,{},{},{},{},{},{},{},{},{},{},{},{},TCP_Aruv,Base,500.000000,1000.000000, 0.000000,{},{}, 0, 0, 0, ,;".format(p[0], p[1], p[2], p[3], p[4], p[5],l[0], l[1], l[2], l[3], l[4], l[5],move_type,isUseJoint) 
        data = "WayPoint,0,420,0,445,180,0,0,{},{},{},{},{},{},TCP_Aruv,Base,50,360,0,0,1,0,0,0,0,;".format(l[0], l[1], l[2], l[3], l[4], l[5])

        # data = "MoveJ,0,{},{},{},{},{},{},;".format(
        #     l[0], l[1], l[2], l[3], l[4], l[5])
        print(data)
        data = data.encode()
        s.send(data)
        result = s.recv(port).decode()
        cut = result.split(',')
        time.sleep(0.01)

        
        if(cut[1] == "OK"):
            while 1:
                s.send(b"ReadRobotState,0,;")
                time.sleep(0.01)
                result = s.recv(port).decode()
                cut = result.split(',')
                if(cut[1] == "OK" and cut[2] == "0"):
                    # time.sleep(0.1)
                    break
        elif(cut[1] == "Fail"):
            print(result)
        
        time.sleep(0.1)
    except Exception as e:
        print(e)
        print("cant retrive the Actual Position of the Robot")

def WayPoint_L(s,pose, port=10003, robotID=0):

    # l = angles
    p = pose
    try:
        
        # data = "WayPoint,0,{},{},{},{},{},{},{},{},{},{},{},{},TCP_Aruv,Base,500.000000,1000.000000, 0.000000,{},{}, 0, 0, 0, ,;".format(p[0], p[1], p[2], p[3], p[4], p[5],l[0], l[1], l[2], l[3], l[4], l[5],move_type,isUseJoint) 
        # data = "WayPoint,0,420,0,445,180,0,0,{},{},{},{},{},{},TCP_Aruv,Base,50,360,0,0,1,0,0,0,0,;".format(l[0], l[1], l[2], l[3], l[4], l[5])
        data = "WayPoint,0,{},{},{},{},{},{},0,0,90,0,90,0,TCP,Base,50,360,0,1,0,0,0,0,0,;".format(p[0], p[1], p[2], p[3], p[4], p[5])

        # data = "MoveJ,0,{},{},{},{},{},{},;".format(
        #     l[0], l[1], l[2], l[3], l[4], l[5])
        print(data)
        data = data.encode()
        s.send(data)
        result = s.recv(port).decode()
        cut = result.split(',')
        time.sleep(0.01)

        
        if(cut[1] == "OK"):
            while 1:
                s.send(b"ReadRobotState,0,;")
                time.sleep(0.01)
                result = s.recv(port).decode()
                cut = result.split(',')
                if(cut[1] == "OK" and cut[2] == "0"):
                    # time.sleep(0.1)
                    break
        elif(cut[1] == "Fail"):
            print(result)
        
        time.sleep(0.1)
    except Exception as e:
        print(e)
        print("cant retrive the Actual Position of the Robot")

def StartPushMovePath(s, Trajectory_name,override,radius, port=10003, robotID=0):
    ID = robotID
    Traj_name = Trajectory_name
    speed_ratio = override
    blend_radius = radius
    try:
        data = "StartPushMovePath,{},{},{},{},;".format(ID,Traj_name,speed_ratio,blend_radius) 
        # data = "MoveJ,0,{},{},{},{},{},{},;".format(
        #     l[0], l[1], l[2], l[3], l[4], l[5])
        print(data)
        data = data.encode()
        s.send(data)
        result = s.recv(port).decode()
        cut = result.split(',')
        time.sleep(0.01)

        if(cut[1] == "OK"):
            while 1:
                s.send(b"ReadRobotState,0,;")
                time.sleep(0.01)
                result = s.recv(port).decode()
                cut = result.split(',')
                if(cut[1] == "OK" and cut[2] == "0"):
                    # time.sleep(0.1)
                    break
        elif(cut[1] == "Fail"):
            print(result)
        
        time.sleep(0.1)
    except Exception as e:
        print(e)
        print("cant perform StartPushMovePath")

def PushMovePathJ(s,tracjectName, Joint_angs, port=10003, robotID=0):
    ID = robotID
    Traj_name = tracjectName
    Joint_angs = Joint_angs

    try:
        data = "PushMovePathJ,{},{},{},{},{},{},{},{},;".format(ID,Traj_name,Joint_angs[0],Joint_angs[1],Joint_angs[2],Joint_angs[3],Joint_angs[4],Joint_angs[5]) 
        # data = "MoveJ,0,{},{},{},{},{},{},;".format(
        #     l[0], l[1], l[2], l[3], l[4], l[5])
        print(data)
        data = data.encode()
        s.send(data)
        result = s.recv(port).decode()
        cut = result.split(',')
        time.sleep(0.01)

        if(cut[1] == "OK"):
            while 1:
                s.send(b"ReadRobotState,0,;")
                time.sleep(0.01)
                result = s.recv(port).decode()
                cut = result.split(',')
                if(cut[1] == "OK" and cut[2] == "0"):
                    # time.sleep(0.1)
                    break
        elif(cut[1] == "Fail"):
            print(result)
        
        time.sleep(0.1)
    except Exception as e:
        print(e)
        print("cant perform PushMovePathJ")



def StopRobot(s,robotID=0):

    try:    
        data = str(" GrpStop, {},; \n".format(robotID))
        data = data.encode()
        s.send(data)
        result= s.recv(port).decode()
        cut=result.split(',')
        if(cut[1]=="OK"):
            return cut
        elif(cut[1]=="Fail"):
            print(result)

    except Exception as e:
        print(e)
        print("cant retrive the Actual Position of the Robot")

def PushMovePathL(s,tracjectName, cart_poses, port=10003, robotID=0):
    ID = robotID
    Traj_name = tracjectName
    cartesian_poses = cart_poses

    try:
        data = "PushMovePathL,{},{},{},{},{},{},{},{},;".format(ID,Traj_name,cartesian_poses[0],cartesian_poses[1],cartesian_poses[2],cartesian_poses[3],cartesian_poses[4],cartesian_poses[5]) 
        # data = "MoveJ,0,{},{},{},{},{},{},;".format(
        #     l[0], l[1], l[2], l[3], l[4], l[5])
        print(data)
        data = data.encode()
        s.send(data)
        result = s.recv(port).decode()
        cut = result.split(',')
        time.sleep(0.01)

        if(cut[1] == "OK"):
            while 1:
                s.send(b"ReadRobotState,0,;")
                time.sleep(0.01)
                result = s.recv(port).decode()
                cut = result.split(',')
                if(cut[1] == "OK" and cut[2] == "0"):
                    # time.sleep(0.1)
                    break
        elif(cut[1] == "Fail"):
            print(result)
        
        time.sleep(0.1)
    except Exception as e:
        print(e)
        print("cant perform PushMovePathL")

def EndPushMovePath(s,tracjectName,port=10003, robotID=0):
    ID = robotID
    Traj_name = tracjectName

    try:
        data = "EndPushMovePath,{},{},;\n".format(ID,Traj_name) 
        # data = "MoveJ,0,{},{},{},{},{},{},;".format(
        #     l[0], l[1], l[2], l[3], l[4], l[5])
        print(data)
        data = data.encode()
        s.send(data)
        result = s.recv(port).decode()
        cut = result.split(',')
        time.sleep(0.01)

        if(cut[1] == "OK"):
            while 1:
                s.send(b"ReadRobotState,0,;")
                time.sleep(0.01)
                result = s.recv(port).decode()
                cut = result.split(',')
                if(cut[1] == "OK" and cut[2] == "0"):
                    # time.sleep(0.1)
                    break
        elif(cut[1] == "Fail"):
            print(result)
        
        time.sleep(0.1)
    except Exception as e:
        print(e)
        print("cant perform EndPushMovePath")

def MovePath(s,tracjectName,port=10003, robotID=0):
    ID = robotID
    Traj_name = tracjectName

    try:
        data = "MovePath,{},{},;\n".format(ID,Traj_name) 
        # data = "MoveJ,0,{},{},{},{},{},{},;".format(
        #     l[0], l[1], l[2], l[3], l[4], l[5])
        print(data)
        data = data.encode()
        s.send(data)
        result = s.recv(port).decode()
        cut = result.split(',')
        time.sleep(0.01)

        if(cut[1] == "OK"):
            while 1:
                s.send(b"ReadRobotState,0,;")
                time.sleep(0.01)
                result = s.recv(port).decode()
                cut = result.split(',')
                if(cut[1] == "OK" and cut[2] == "0"):
                    # time.sleep(0.1)
                    break
        elif(cut[1] == "Fail"):
            print(result)
        
        time.sleep(0.1)
    except Exception as e:
        print(e)
        print("cant perform MovePath")

def ReadMovePathState(s,traj_name,port = 10003, robotID = 0):
    Trajectory_name = traj_name
    try:
        data = str("ReadMovePathState,{},{},;\n".format(robotID,Trajectory_name))
        data = data.encode()
        s.send(data)
        result= s.recv(port).decode()
        cut=result.split(',')
        if(cut[1]=="OK"):
            print(os.getcwd())
            return list(map(float,cut[2]))
        elif(cut[1]=="Fail"):
            print(result)
    except Exception as e:
        print(e)
        print("cant retrive the Actual Position of the Robot") 