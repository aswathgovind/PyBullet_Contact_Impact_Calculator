import numpy as np
from Utils.Ur5e_Robot_State import *
# s = connect('192.168.11.11')
# SetUCSByName(s,'Base')
joint_ang = []
cart_pose = []
noOfpoints= 2 #int(input("Enter no of Joint values:"))

for i in range(0,noOfpoints,1):
    record_flag = bool(input("Record Joint value:-"))
    if(record_flag):
        recorded_pose = get_tool_pose()
        cart_pose.append(recorded_pose[0:6])
        print(cart_pose[i])

print("the cartesian values are",cart_pose)

np.savez('recorded_jointangles.npz',car_pose = cart_pose)
data = np.load('recorded_jointangles.npz')
print(data['car_pose'])