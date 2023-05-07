import rtde_receive
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.25")
def get_joint_vals():
    actual_q = rtde_r.getActualQ()
    return actual_q

def get_tool_pose():
    pose = rtde_r.getActualTCPPose()
    return pose