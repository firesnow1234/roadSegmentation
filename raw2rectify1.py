#!/usr/bin/env python
# -*- coding: utf-8 -*

#ros
#import rospy
import os
import sys                                                                    
import signal  

#basic python tools
import itertools
import numpy as np
import string
import time
import math
import cv2
#odometry handler
import pykitti

import readBIN


#plot
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

def quit(signum, frame):  
    print (''  )
    print ('stop fusion')  
    sys.exit()  
                                                         
file_list0=os.listdir('F:\\ivLAB\\201807\\Lidar\\Road\\data_road\\training\\gt_image_2')       
file_list0.sort()

file_list1=os.listdir('F:\\ivLAB\\201807\\Lidar\\Road\\data_road\\training\\image_2')       
file_list1.sort()

file_list_bin=os.listdir('F:\\ivLAB\\201807\\Lidar\\Road\\data_road_velodyne\\training\\velodyne')
file_list_bin.sort()

file_list_calib = os.listdir('F:\\ivLAB\\201807\\Lidar\\Road\\data_road\\training\\calib')
file_list_calib.sort()

count=0


if True: 
  camera = cv2.imread('F:\\ivLAB\\201807\\Lidar\\Road\\data_road\\training\\gt_image_2\\' + file_list0[count])
  camera2 = cv2.imread('F:\\ivLAB\\201807\\Lidar\\Road\\data_road\\training\\image_2\\' + file_list1[count])
  pointCloud  = readBIN.getBin("F:\\ivLAB\\201807\\Lidar\\Road\\data_road_velodyne\\training\\velodyne\\" + file_list_bin[count])
  pointCloud=pointCloud[0:pointCloud.shape[0]:1,:]  
  P, velo2cam = readBIN.readCalibration("F:\\ivLAB\\201807\\Lidar\\Road\\data_road\\training\\calib\\" + file_list_calib[count])
  Tr = np.zeros((4,4))
  Tr[0:3, :] = velo2cam
  Tr[3][3] = 1
  P0=np.zeros((4,4))
  P0[0:3,0:4]= P
  P0[3,:]= [0, 0 ,0 ,1]
  #camera = cv2.GaussianBlur(camera,(3,3),0)  
  #camera = cv2.Canny(camera, 50, 150) 
  count+=1
  plt.clf()
  plt.imshow(camera,cmap='gray')
  #plt.imshow(camera)
  print(camera.shape)
  x=[]
  y=[]
  color=[]

  cPointCloud=np.zeros((pointCloud.shape[0],pointCloud.shape[1]))
  depthPointCloud=np.zeros((pointCloud.shape[0],1))
  countOutPoint = 0
  for i in range(pointCloud.shape[0]):
    point=pointCloud[i,:]
    point[3]=1    
    newPoint=P0.dot(Tr.dot(point.T))
    if (Tr.dot(point.T))[2] < 0:      
      continue
   
    newPoint=newPoint/newPoint[2]
    
    depthPointCloud[i,0]=np.sqrt((point[0]*point[0]+point[1]*point[1]+point[2]*point[2]))
   
    if newPoint[1]>0 and newPoint[1]<camera.shape[0]-1 and newPoint[0]>0 and newPoint[0]<camera.shape[1]-1:     
      if 1:
        countOutPoint+=1
        cPointCloud[countOutPoint-1,:]=pointCloud[i,:]        
        x.append(int(newPoint[0]))
        y.append(int(newPoint[1]))
        color.append(1/(depthPointCloud[i,0]))

  temp=cPointCloud[0:countOutPoint,:]  
  plt.scatter(x,y,c=color,cmap=plt.cm.jet,marker='.',s=0.2)
  print('inpoint:',countOutPoint,pointCloud.shape[0])
  print('next:')
  #plt.figure()
  #plt.imshow(camera)
  plt.figure()
  plt.imshow(camera2)
  plt.scatter(x,y,c=color,cmap=plt.cm.jet,marker='.',s=1)
  plt.show()
  plt.pause(0.1)
            
        
                  
  










  