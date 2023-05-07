import math
from math import *
import math3d as m3d
import numpy as np
import time
import socket 
import os
import numpy as np
# import tqdm

def ActualMovement(jv,a,v,r):
    
    # print(jv,a,v,r,robo)
    a = "a = " + str(a)
    v = "v = " + str(v)
    r = "r = " + str(r)

    filename = "example1.script"
    f = open (filename, "wb")
    f.write("def movement():\n".encode("utf8"))
    f.write("  while(True):\n".encode("utf8"))
    # print(len(jv))
    for i in range (len(jv)):
        message = str(list(np.deg2rad(jv[i])))
        print(message)
        if i == len(jv)-1:
            movej = "    "+"movej"+"("+message+","+a+","+v+")"+"\n"
            movej = movej.encode("utf8")
            f.write(movej)
        else:    
            movej = "    "+"movej"+"("+message+","+a+","+v+","+r+")"+"\n"
            movej = movej.encode("utf8")
            f.write(movej)
    f.write("    halt\n".encode("utf8"))
    f.write("    end\n".encode("utf8"))
    f.write("  end\n".encode("utf8"))
    f.write("end\n".encode("utf8"))
    f.close()
    
    HOST = "192.168.1.25" #Robot IP address
    # HOST = "127.0.0.1"
    PORT = 30002 # UR secondary client
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    f = open (filename, "rb") 
    l = f.read(1024)
    count =1
    while (l):
        s.send(l)
        print(count)
        count = count+1
        print(l)
        l = f.read(1024)
    s.close()
    counter = 1
    return 0

def GoHome():
    # HOST = "172.16.101.224" #Robot IP address
    HOST = "127.0.0.1"
    PORT = 30002 # UR secondary client
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    s.send (b'movej([0.0004188790204786391, -1.571599178250814, 1.4663558243555561, 0.8723679200243257, 1.5716340848358539, -3.1412435877393943],a = 0.35,v = 0.25,r = 0.02)')
    s.close()
    counter = 1
    return 0