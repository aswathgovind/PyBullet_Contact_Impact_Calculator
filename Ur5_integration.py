import rtde_control
import rtde_receive
import time
import csv
import numpy as np
import pickle
poses = []


r_flag = True

def record_p():
    ctr = 0
    rtde_r = rtde_receive.RTDEReceiveInterface("127.0.0.1")
    st = time.time() 
    print("start recording ... ")
    while (time.time() - st < 20):
        poses.append(rtde_r.getActualQ())
        print(poses[ctr])
        ctr+=1
        time.sleep(0.015)
    with open('servo_points', 'wb') as fp:
        pickle.dump(poses, fp)
    print(len(poses))
    print(time.time() - st)
    print("recording done...")

def play_p(poses_n =None):
    rtde_c = rtde_control.RTDEControlInterface("127.0.0.1")
    # Parameters
    velocity = 3.1
    acceleration = 3.1
    dt = 1.0/500  # 2ms
    lookahead_time = 0.1
    gain = 300

    if(poses_n == None):
        with open('servo_points', 'rb') as fp:
            poses_n = pickle.load(fp) 
    
    rtde_c.moveJ(poses_n[0])
    # Execute 500Hz control loop for 2 seconds, each cycle is 2ms
    st = time.time()
    for i in range(len(poses_n)):       
        print(str(poses_n[i][0]) + "," + str(poses_n[i][1]) + "," + str(poses_n[i][2]) + "," +  str(poses_n[i][3]) + "," + str(poses_n[i][4]) + "," + str(poses_n[i][5]))
        if(rtde_c.servoJ(poses_n[i], velocity, acceleration, dt, lookahead_time, gain)):
            pass
        else:
            break       
        i=i+1
        time.sleep(0.015)      
    print("time taken to play: " + str(time.time() - st))
    rtde_c.servoStop()
    rtde_c.stopScript()

# record_p()

# play_p()
# # Parameters
# acceleration = 0.5
# dt = 1.0/500  # 2ms
Movement_cmd = [[0,-90,0,-90,0,30],[0,-90,0,-90,0,35]]
# Move to initial joint position with a regular moveJ

rtde_c = rtde_control.RTDEControlInterface("192.168.1.25")
for joint_q in Movement_cmd:
    rtde_c.moveJ(np.deg2rad(joint_q))
    print(joint_q)

rtde_c.stopScript()
