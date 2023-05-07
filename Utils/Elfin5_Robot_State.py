from Utils.Elfin_new_API import connect,getj

s = connect('192.168.0.10')

def get_joint_vals(np):
    joint_vals = getj(s)
    return [np.deg2rad(joint_vals[0]),np.deg2rad(joint_vals[1]),np.deg2rad(joint_vals[2]),np.deg2rad(joint_vals[3]),np.deg2rad(joint_vals[4]),np.deg2rad(joint_vals[5])]