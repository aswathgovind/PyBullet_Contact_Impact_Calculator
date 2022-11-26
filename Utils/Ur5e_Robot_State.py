import rtde_receive
rtde_r = rtde_receive.RTDEReceiveInterface("172.16.101.224")

def get_joint_vals():
    actual_q = rtde_r.getActualQ()
    return actual_q