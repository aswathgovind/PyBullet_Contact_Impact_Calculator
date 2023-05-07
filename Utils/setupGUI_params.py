def setup_joint_sliders_UR(Pyb_Obj):
    joint_1_param = Pyb_Obj.addUserDebugParameter("joint 1",-360,360,0)
    joint_2_param = Pyb_Obj.addUserDebugParameter("joint 2",-360,360,-90)
    joint_3_param = Pyb_Obj.addUserDebugParameter("joint 3",-360,360,90)
    joint_4_param = Pyb_Obj.addUserDebugParameter("joint 4",-360,360,0)
    joint_5_param = Pyb_Obj.addUserDebugParameter("joint 5",-360,360,0)
    joint_6_param = Pyb_Obj.addUserDebugParameter("joint 6",-360,360,0)

    return [joint_1_param,joint_2_param,joint_3_param,joint_4_param,joint_5_param,joint_6_param]

def joint_vals_from_sliders(np,Pyb_Obj,JSliders_uni):
    joint_val = []
    for count in range(0,len(JSliders_uni),1):
        joint_val.append(np.deg2rad(Pyb_Obj.readUserDebugParameter(JSliders_uni[count])))
    return joint_val

def setup_pose_slider(Pyb_Obj):
    # [-0.30,0.110,0.0]
    X = Pyb_Obj.addUserDebugParameter("X",-0.50,0.20,-0.30)
    Y = Pyb_Obj.addUserDebugParameter("Y",-0.50,0.20,-0.110)
    Z = Pyb_Obj.addUserDebugParameter("Z",-0.50,0.20,0)

    # X = Pyb_Obj.addUserDebugParameter("X",-360,360,0)
    return [X,Y,Z]

def get_pose_slider_val(Pyb_Obj,Sliders_uni):
    pose = []
    for count in range(0,len(Sliders_uni),1):
        pose.append(Pyb_Obj.readUserDebugParameter(Sliders_uni[count]))
    return pose