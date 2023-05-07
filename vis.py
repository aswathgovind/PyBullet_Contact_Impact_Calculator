import rtde_receive
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.25")
actual_q = rtde_r.getActualQ()
print(actual_q)