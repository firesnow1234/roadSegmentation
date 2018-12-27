#!/usr/bin/env python
# -*- coding: utf-8 -*
#import rospy
#from std_msgs.msg import Header
import itertools
import numpy as np
import cv2
import pykitti
import time

from PIL import Image
#import pcl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

#from sensor_msgs.msg import PointCloud2, PointField

#find interceptor for every frame
#input: hori angle of every point
def find_interceptor(dangle_h):
    index = []
    end = False
    scaBegin = True
    interceptor = 0
    while 1:
        #print(j)
        if end == True:
            break
        #print(interceptor)
        for i in range(0,50):
            if interceptor+i+1>=len(dangle_h):
                end=True
                #print("end")
                break
            if dangle_h[interceptor+i] == 1:
                if scaBegin:
                    index.append(interceptor+i+1)
                    scaBegin = False
                else:
                    scaBegin = True
                interceptor = interceptor + i +50
                break
        interceptor = interceptor+i
    #print(index)
    #print(index)
    return index

def compute_hori_angle(velo):
    angle_h =[]
    angle_v = []
    j=0
    
    for i in velo:
        if j==0:
            lastveh = np.arctan2(i[1],-i[0])*180/np.pi
        j+=1
        #if j>93780 or j<93720:
        #   continue
        
        ve = np.arctan2(i[2],np.sqrt(i[0]**2+i[1]**2))*180/np.pi
        ve_h = np.arctan2(i[1],-i[0])*180/np.pi
        #ve_h = int(ve_h*100000)/100000.0
        #ve = int(ve)
        ve = (int(ve*1000))/1000.0
        angle_v.append(ve)
        angle_h.append(ve_h)
        lastveh=ve_h
        
    #print(angle_h)
    #plt.figure()
    #plt.plot(angle_v)
    #plt.show()
    dangle_h=[]
    
    j=0
    i=0

    k=0    

    index = []
    index.append(0)
    maxs = 0
    lasts = 0
    backs = 0
    select = True
    selectcount = 0
    for s in angle_h:
        if i ==0 :
            lasts=s
            backs=s
        if select and selectcount < 10:
            selectcount+=1
        else:
            select = False
            selectcount = 0
       # if maxs<s and (s-lasts)<=90:
        #    index[k] = i
         #   maxs=s
          #  dangle_h.append(1)
        if not select:
        #if 1:
            if  (s-lasts)>90 and s-backs>90:
            #print(s-lasts,i)
                print(i-index[-1])
                index.append(i)
                
                k+=1
                maxs=s
                select = True
                dangle_h.append(1)
            else:
                dangle_h.append(0)
        backs = lasts
        lasts = s
        
        i=i+1
    #print(angle_h)
    print(select)
    print(selectcount)
    return angle_v,angle_h,dangle_h,index

def plus_minus(velo,index):
    count = 0
    p = 0
    angle_v_raw=[]
    for i in velo:
        
        if (p > index[count]-2) and count<63:
            count+=1
        p+=1
        if count <= 32:
            angle_v_raw.append(np.arctan2(i[2]-0.2,np.sqrt(i[0]**2+i[1]**2))*180/np.pi)
        else:
            angle_v_raw.append(np.arctan2(i[2]-0.12,np.sqrt(i[0]**2+i[1]**2))*180/np.pi)
    return angle_v_raw
# Change this to the directory where you store KITTI data
basedir = '/home/luolun/Odometry/dataset'
#basedir = '/home/luolun/data/Raw data/Residential/2011_09_30/2011_09_30_drive_0027_sync/velodyne_points'

# Specify the dataset to load
sequence = '01' 


dataset = pykitti.odometry(basedir, sequence, frames=range(0, 100, 1))         #function range generate a sequence staring from 0 and ending at 20 with a step of 5 
                                                                              #odometry.py line 139: dataset.velo is iterator for N*4 array of 
third_velo = iter(itertools.islice(dataset.velo, 0, None))
print(dataset.calib.T_cam0_velo)
for s in third_velo:
#third_velo是激光雷达数据的一个迭代器，s是一帧激光雷达数据，s的数据类型是np.array，一行为一个数据，数据字段是[x,y,z,r],r是当前点的反射率
#如果雷达中只有两个点(1,2,3,0.1),(2,3,1,0.5)，那么s就是np.array([[1,2,3,0.1],[2,3,1,0.5]])
    countnot0 = 0
    #s = dataset.calib.T_cam0_velo.dot(s.T)
    #s = s.T
    lidar_image = np.zeros((64,2190))                 #lidar_image_*保存最终的Image结果，像素值是1/x
    lidar_image_x = np.zeros((64,2190))
    lidar_image_y = np.zeros((64,2190))
    lidar_image_z = np.zeros((64,2190))
    angle_v,angle_h,dangle_h,index = compute_hori_angle(s)  #这里是分线算法，把雷达帧s帧中处于同一线的点放在一起
    
    #下面的算法将点分布到lidar_image中
    indAngleHold=0
    for i in range(len(index)-1):    #对每一线的激光点
      lastAngle=0  #这俩个变量不管
      indAnglePoint=0 
      for j in range(index[i],index[i+1]):      #对每一线激光点的每一个点
        dis = np.sqrt(s[j,0]**2+s[index[i],1]**2+s[index[i],2]**2) 
        dis_z = s[j,2]
        dis_y = s[j,1]
        dis_x = s[j,0]
        ind_cur_point = np.rint((180-angle_h[j])/0.1728) 
        ind_cur_point = int(ind_cur_point)
        
        if lidar_image[i,ind_cur_point]!=0:
          countnot0+=1
        #lidar_image[i,ind_cur_point] = dis
        #print(lastAngle-angle_h[j],angle_h[j])
        
        #给lidar-image赋像素值，如果要改的话if和else要同时改
        if np.fabs(lastAngle-angle_h[j])>0.1728*1.5:
          lidar_image[i,ind_cur_point] = dis            
          lidar_image_x[i,ind_cur_point] = dis_x
          lidar_image_y[i,ind_cur_point] = dis_y
          lidar_image_z[i,ind_cur_point] = dis_z
          indAnglePoint = j
          lastHold = ind_cur_point
          #print(ind_cur_point)
        else:
          lidar_image[i,j - indAnglePoint + lastHold] = dis
          lidar_image_x[i,ind_cur_point] = dis_x
          lidar_image_y[i,ind_cur_point] = dis_y
          lidar_image_z[i,ind_cur_point] = dis_z
          #print(lastHold, j - indAnglePoint + lastHold)
        
        lastAngle=angle_h[j]
    #lidar_image = lidar_image_z
    #给像素值取倒数得到视差图
    for i in range(64):
      for j in range(2190):
        if lidar_image_x[i,j]<0.1:  #控制视差值大小，最大为10
          lidar_image_x[i,j]=0
        else:
          lidar_image_x[i,j]=1/lidar_image_x[i,j]
    lidar_image=lidar_image_x
    #lidar_image = np.rint(lidar_image/np.max(lidar_image)*256)
    #print(countnot0)
    #lidar_image = np.rint(lidar_image)*256
    
    #index = find_interceptor(dangle_h)
    #angle_v_raw = plus_minus(s,index)
    #f2=plt.figure()
    #ax2 = f2.add_subplot(111, projection='3d')
    #for i in range(0,3):
    #    if i%2 == 0:
    #        ax2.plot(s[index[i]:index[i+1]-1,0],s[index[i]:index[i+1]-1,1],s[index[i]:index[i+1]-1,2],color = "b")
    #    else:
    #        ax2.plot(s[index[i]:index[i+1]-1,0],s[index[i]:index[i+1]-1,1],s[index[i]:index[i+1]-1,2],color = "g")
    #ax2.plot(s[index[0]:index[1],0],s[index[0]:index[1],1],s[index[0]:index[1],2],color = "g")
    #plt.plot(range(index[10],index[11]),np.array(angle_v[index[10]-1:index[11]-1]))
    #plt.plot(range(index[10],index[11]),np.abs(np.array(angle_h[index[10]-1:index[11]-1]))/90)
    #@ss = []
    #lastkk = angle_h[0]
    #for kk in angle_h:
    #    ss.append((kk)/90)
    #    lastkk=kk
    
    #
    #
    
    #plt.plot(angle_v,label="vertical angle")
    #plt.plot(angle_h,label='Azimuth angle')
    #
    #plt.imshow(lidar_image[:,lidar_image.shape[1]/8*3:lidar_image.shape[1]/8*5])lidar_image = cv2.Canny(lidar_image, 50, 150) 
    #lidar_image = cv2.GaussianBlur(lidar_image,(3,3),0)  
    #lidar_image = cv2.Sobel(lidar_image,cv2.CV_64F,1,0,ksize=3)
    #lidar_image -= np.min(lidar_image) 
    #lidar_image = cv2.Canny(lidar_image,50,150)
    plt.imshow(lidar_image,cmap=plt.cm.jet)
    plt.figure()
    plt.imshow(lidar_image[:,520:1560],cmap=plt.cm.jet)
    plt.title('back')
    ss=np.zeros((64,1040))
    ss[:,0:520] = lidar_image[:,1561:2081]
    ss[:,520:1040] = lidar_image[:,0:520]
    #取前方视野
    sss = np.zeros((64,1040))
    for i in range(ss.shape[1]):
      sss[:,i] = ss[:,ss.shape[1]-i-1]
    plt.figure()
    plt.imshow(sss[:,260:780],cmap=plt.cm.jet)
    plt.title('front')
    fontImage = sss[:,260:780]
   
    #直方图
    histogram = np.zeros((64,1000))  #直方图变量
    for i in range(64):
      for j in range(520):
        indhis = int(fontImage[i,j]/0.001)  #以0.001为分辨率，统计直方图的值
        histogram[i,indhis] += 1
    imRGB = Image.new("RGB",(2190,64))
    for i in range(64):
      for j in range(2190):
        imRGB.putpixel((j,i),(int(lidar_image_z[i,j]),int(lidar_image_y[i,j]),int(lidar_image_x[i,j])))
    
    #imRGB.show()
    #plt.legend(fontsize='x-large')
    #plt.xlabel('index')
    #plt.ylabel('angle')
    #plt.plot([1,2,3])
    #print(index)
    #print(len(index))
    #for i in range(0,63):
    #    print(angle_v_raw[index[i]])
    #print(angle_v[index[0]],angle_v[index[1]],angle_v[index[2]])
    #print(angle_h[0])
    #plt.grid(True, linestyle = "-.", color = "b", linewidth = "1")  
    
    #plt.figure()
    #plt.imshow(lidar_image_x[:,520:1560])
    #plt.imshow(lidar_image_y[:,520:1560])
    #plt.imshow(lidar_image_z[:,520:1560])
    plt.figure()
    plt.imshow(histogram)
    plt.show()
    #for i in range(index[0],index[1]):
    #    print(angle_v[i])

#s[index[i]:int((index[i+1]+index[i])/2),2],
#s[index[i]:int((index[i+1]+index[i])/2),2],
'''
#print(dangle_h)
print(index)
print(j)
plt.figure()
plt.plot(dangle_h)
plt.show()
'''
"""
f2 = plt.figure()
#n, bins, patches = plt.hist(angle, bins=(max(angle)-min(angle))*10, normed=1, facecolor='green', alpha=0.75)
r = np.arange(-24.2,5,0.01)
rr = np.arange(-24,5,1)
r=r.reshape(his.shape[0],his.shape[1])
rr=rr.reshape(his2.shape[0],his2.shape[1])
#print(max(angle),min(angle))
s = plt.stem(r,his)
plt.show()

#print(his)

"""
    


