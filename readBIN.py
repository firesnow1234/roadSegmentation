# -*- coding: utf-8 -*-
"""
Created on Mon Aug 13 20:06:25 2018

@author: Yuki
"""
import struct 
import numpy as np


#读取激光雷达数据二进制文件， 返回激光雷达的数组 n*4
def getBin(filename):
    f = open(filename, "rb")
    lidarList = []
    data = '0'
    while True:
        data = f.read(4)
        if len(data) == 0:
            break
        elem = struct.unpack("f", data)[0]
        lidarList.append(elem)
    f.close()
    lidarArray = np.array(lidarList)
    lidarArray = lidarArray.reshape(len(lidarArray)//4, 4)
    return lidarArray
lidarData = getBin("um_000000.bin")




#读取激光雷达标定数据， 返回 P0 和 velo2cam
def readCalibration(filename):
    f2 = open(filename,"r")
    lines = f2.readlines()
    num = []
    for line3 in lines:
        print (line3)
        num.append(line3.split(" "))
    P0 = []
    velo2cam=[]
    for i in range(1, len(num[0])):
        P0.append(float(num[0][i]))
        velo2cam.append(float(num[5][i]))
    P0 = np.array(P0)
    P0 = P0.reshape((3,4))
    velo2cam = np.array(velo2cam)
    velo2cam = velo2cam.reshape((3,4))
    return P0, velo2cam    

p0, v1 = readCalibration("um_000000.txt")
    






