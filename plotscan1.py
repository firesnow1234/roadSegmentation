#!/usr/bin/env python
# -*- coding: utf-8 -*
#import rospy
#from std_msgs.msg import Header
import itertools
import numpy as np
import cv2
import pykitti
import time
import readBIN
import ransacPlane
import os

from PIL import Image
#import pcl
import matplotlib.pyplot as plt
from myRansac import *
from mpl_toolkits.mplot3d import Axes3D
import math
import matlab.engine
import numpy as np
import scipy.io as io
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
    #print(select)
   # print(selectcount)
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
basedir = 'Odometry\\dataset'
#basedir = '/home/luolun/data/Raw data/Residential/2011_09_30/2011_09_30_drive_0027_sync/velodyne_points'

# Specify the dataset to load
sequence = '07' 

dataset = pykitti.odometry(basedir, sequence, frames=range(556, 557, 1))         #function range generate a sequence staring from 0 and ending at 20 with a step of 5 
                                                                              #odometry.py line 139: dataset.velo is iterator for N*4 array of 
third_velo = iter(itertools.islice(dataset.velo, 0, None))
#print(dataset.calib.T_cam0_velo)

file_list_bin=os.listdir('F:\\ivLAB\\201807\\Lidar\\Road\\data_road_velodyne\\training\\velodyne')
file_list1=os.listdir('F:\\ivLAB\\201807\\Lidar\\Road\\data_road\\training\\image_2')
file_list0=os.listdir('F:\\ivLAB\\201807\\Lidar\\Road\\data_road\\training\\gt_image_2')
file_list_calib = os.listdir('F:\\ivLAB\\201807\\Lidar\\Road\\data_road\\training\\calib')
file_list1.sort()
file_list0.sort()
file_list_calib.sort()
file_list_bin.sort()

count = 0
if True:
    sourceLidar  = readBIN.getBin("F:\\ivLAB\\201807\\Lidar\\Road\\data_road_velodyne\\training\\velodyne\\" + file_list_bin[count])
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
    s = sourceLidar
    s[:,3]=1
    countnot0 = 0
    
    lidar_image = np.zeros((64,2190))
    lidar_image_x = np.zeros((64,2190))
    lidar_image_y = np.zeros((64,2190))
    lidar_image_z = np.zeros((64,2190))
    
    lidar_image_all = np.zeros((64,2190,4))
    
    angle_v,angle_h,dangle_h,index = compute_hori_angle(s)
    
    indAngleHold=0
    
    lidar_image_correspondance = np.zeros((64,2190))
        
    for i in range(len(index)-1):
      lastAngle=0
      for j in range(index[i],index[i+1]):
        dis = np.sqrt(s[j,0]**2+s[index[i],1]**2+s[index[i],2]**2) 
        dis_z = s[j,2]
        dis_y = s[j,1]
        dis_x = s[j,0]
        ind_cur_point = np.rint((180-angle_h[j])/0.1728) 
        ind_cur_point = int(ind_cur_point)
        
        if lidar_image[i,ind_cur_point]!=0:
          countnot0+=1
        
        if np.fabs(lastAngle-angle_h[j])>0.1728*1.5:
          lidar_image[i,ind_cur_point] = dis
          lidar_image_x[i,ind_cur_point] = dis_x
          lidar_image_y[i,ind_cur_point] = dis_y
          lidar_image_z[i,ind_cur_point] = dis_z
          indAnglePoint = j
          lastHold = ind_cur_point
          
          lidar_image_all[i, ind_cur_point, :] = s[j, :]
          lidar_image_correspondance[i,ind_cur_point] = j
        else:
          lidar_image[i,j - indAnglePoint + lastHold] = dis
          lidar_image_x[i,j - indAnglePoint + lastHold] = dis_x
          lidar_image_y[i,j - indAnglePoint + lastHold] = dis_y
          lidar_image_z[i,j - indAnglePoint + lastHold] = dis_z
          
          lidar_image_all[i, j - indAnglePoint + lastHold, :] = s[j, :]
          
          lidar_image_correspondance[i,j - indAnglePoint + lastHold] = j
          
          
          
        lastAngle=angle_h[j]
        
    #lidar_image = lidar_image_z
    for i in range(64):
      for j in range(2190):
        if lidar_image_x[i,j]<0.001:
          lidar_image_x[i,j]=0
        else:
          lidar_image_x[i,j]=1/lidar_image_x[i,j]
    lidar_image=lidar_image_x
    '''
    plt.imshow(lidar_image,cmap=plt.cm.jet)
    plt.figure()
    plt.imshow(lidar_image[:,520:1560],cmap=plt.cm.jet)
    plt.title('back')
    '''
    #取前方视野
    ss=np.zeros((64,1040))
    ss[:,0:520] = lidar_image[:,1561:2081]
    
    front_cor= np.zeros((64,1040))
    
    
    ss[:,520:1040] = lidar_image[:,0:520]
    
    front_cor[:,0:520] = lidar_image_correspondance[:,1561:2081]
    front_cor[:,520:1040] = lidar_image_correspondance[:,0:520]
    
    lidar_image_correspondance = np.zeros((64,1040))
    sss = np.zeros((64,1040))
    for i in range(ss.shape[1]):
      sss[:,i] = ss[:,ss.shape[1]-i-1]
      lidar_image_correspondance[:,i] = front_cor[:,ss.shape[1]-i-1]
      
    '''
    plt.figure()
    plt.imshow(sss[:,260:780],cmap=plt.cm.jet)
    plt.title('Front-View')
    '''
    
    frontImage = sss[:,260:780]
    lidar_image_correspondance = lidar_image_correspondance[:,260:780]
    
    print('start')
    time_start=time.time()
    #完全获取雷达xyzr数据
    lidar_xyzr = np.ones((64,1040,4))
    lidar_xyzr[:, 0:520,:] = lidar_image_all[:,1561:2081,:]
    lidar_xyzr[:,520:1040,:] = lidar_image_all[:,0:520,:]
    lidar_tmp = np.zeros((64,1040,4))
    for i in range(lidar_xyzr.shape[1]):
        lidar_tmp[:,i,:] = lidar_xyzr[:, lidar_xyzr.shape[1]-i-1]
    frontLidar = lidar_tmp[:,260:780,:]
    
    #直方图
    histogram = np.zeros((64,1000))
    for i in range(20,64):
      for j in range(520):
        indhis = int(frontImage[i,j]/0.001)
        histogram[i,indhis] += 1
    imRGB = Image.new("RGB",(2190,64))
    for i in range(64):
      for j in range(2190):
        imRGB.putpixel((j,i),(int(lidar_image_z[i,j]),int(lidar_image_y[i,j]),int(lidar_image_x[i,j])))
    '''
    plt.figure()
    plt.imshow(histogram)
    plt.show()  
    '''

    
    #二值化
    histArray =[]
    for x in range(64):
        for y in range(1, 1000):
            if histogram[x][y] > 20:
                histArray.append([y,x])
                
    histPoints = np.array(histArray) 
    hist = []
    for i in range(len(histPoints)):
        hist.append([histPoints[i,0],histPoints[i,1]])
    
    #ransac直线拟合
    m,b = plot_best_fit(hist)
   # print('m:', m, '\nb:', b)
    
    #路面分割的超参数
    alpha = 0.5
    beta = 1.2
    roadSegment = np.zeros([len(frontImage), len(frontImage[0])])
    
    # i: 激光扫描线,  j: 激光雷达获取图的前方视野点
    for i in range(len(frontImage)):
        for j in range(len(frontImage[0])):
            light = int(frontImage[i,j] / 0.001)
            #case1: water
            if(frontImage[i,j] == 0 and i <= m*light + beta*b):
                roadSegment[i][j] = 0
            # case2： posetive obstacle
            elif(i <  m*light + alpha*b):
                roadSegment[i][j] = 1
            #case3: negative
            elif(i > m*light + beta*b):
                roadSegment[i][j] = 2
            #case4: road line
            elif(i >= m*light + alpha*b and i <= m*light + beta*b):
                roadSegment[i][j] = 3
    
    #print('totally cost',time_end-time_start)
    #归一化卷积核滤波
    finalMap=cv2.blur(roadSegment,(5,5))
    '''
    plt.figure()            
    plt.imshow(roadSegment,cmap=plt.cm.jet)
    plt.savefig("r.jpg")    
    plt.title('Road Segmentation')
    plt.figure()   
    plt.imshow(finalMap,cmap=plt.cm.jet)    
    plt.show()
    '''
    
    #保存点云数据 20181205    
    lidarKitti = sourceLidar[:,:3]
    lidarKitti = lidarKitti.T
    io.savemat('lidarKitti_' + file_list_bin[count], {'data': lidarKitti})  
    
    lidarFront = frontLidar[:,:,:3]
    lidarFront2 = np.reshape(lidarFront,(-1,3))
    lidarFront3 = lidarFront2.T
    io.savemat('lidarKittiFront_' + file_list_bin[count], {'data': lidarFront3})
    
    lidarFrontFun1 = np.reshape(frontLidar, (-1,4))
    
    
    roadProp = np.zeros([len(frontImage), len(frontImage[0])])
    
    # i: 激光扫描线,  j: 激光雷达获取图的前方视野点
    #点（a，b）到直线Ax+By+C=0的距离为d=|Aa+Bb+C|/√(A^2+B^2)
    #直线 mx - y + b = 0
    maxDist = 0
    minDist = 0
    for i in range(len(frontImage)):
        for j in range(len(frontImage[0])):
            light = int(frontImage[i,j] / 0.001)
            if light == 0:
                continue
            dist = abs(m * light - i + b)/((-1)*(-1) + m * m)**0.5
            maxDist = max(maxDist, dist)
            minDist = min(minDist, dist)
       
    #print(maxDist)
    #print(minDist)
    
    maxDist = 16 
    for i in range(len(frontImage)):
        for j in range(len(frontImage[0])):
            light = int(frontImage[i,j] / 0.001)
            if light == 0:
                continue
            dist = abs(m * light - i + b)/((-1)*(-1) + m * m)**0.5
            if dist > 16:
                roadProp[i,j] =  0
                continue
            roadProp[i,j] =  1 - (dist / (maxDist - minDist))
    '''
    plt.figure()        
    plt.imshow(roadProp,cmap=plt.cm.jet)    
    plt.title('Road Probability')
    '''
    
    '''
    # 提取法线方向
    #in： frontLidar
    #out: prop
    #块大小 4*4
    '''
    
    # 调用matlab Ransac函数
    '''
    eng = matlab.engine.start_matlab()
    result = eng.ransac1212(count, 100)
    a,b,c,d = result[0][0], result[0][1], result[0][2], result[0][3]  # ax + by + cz + d = 0
    '''
    a = 0
    b = 0
    c = -1
    
    upVector = np.array([a,b,-c])
    normalVectorProp = np.zeros([len(frontLidar), len(frontLidar[0])])
    
    for i in range(0,64,4):
        for j in range(0,520,4):
            miniBox = frontLidar[i:i+4, j:j+4,:]
            #hDis = np.sqrt(np.sum(np.square(miniBox[0,0,:2] - miniBox[0,3,:2])))
            vDis = np.sqrt(np.sum(np.square(miniBox[0,0,:2] - miniBox[3,0,:2])))
            if(vDis < 0.1):
                j = j + 4
                continue
            x,y,z = miniBox[:,:,0], miniBox[:,:,1], miniBox[:,:,2]
            newBox = [x.reshape(-1),y.reshape(-1),z.reshape(-1)]
            box_cov = np.cov(newBox)
            eigenValue, eigenVector = np.linalg.eig(box_cov)
            sorted_indices = np.argsort(-eigenValue)
            least_evecs = eigenVector[:,sorted_indices[:-2:-1]]
            
            least_evecs = least_evecs.ravel()
            Lx = np.sqrt(least_evecs.dot(least_evecs))
            Ly = np.sqrt(upVector.dot(upVector))
            cos_angle=least_evecs.dot(upVector)/(Lx*Ly)
            angle=np.arccos(cos_angle)
            angle2=angle * 360/2/np.pi
            if angle2 < 30:
                normalVectorProp[i:i+4, j:j+4] = 1 - (angle2 / 30)
            else:
                normalVectorProp[i:i+4, j:j+4] = 0
    '''
    for i in range(64):
        for j in range(520):
            if normalVectorProp[i][j] == 0:
                roadProp[i][j] = roadProp[i][j] = 0
    '''
    
    '''
    ### 计算均值
    '''
    meanZProp = np.zeros([len(frontLidar), len(frontLidar[0])])
    zList = []
    for i in range(64):
        for j in range(520):
            if roadProp[i][j] > 0.5:
                zList.append(frontLidar[i,j,2])
    meanZ = np.mean(zList) 
    varZ = np.var(zList)
    for i in range(0,64,2):
        for j in range(0,520,2):
            miniBox2 = frontLidar[i:i+2,j:j+2,2]
            meanHight = np.mean(miniBox2)
            if abs(meanHight - meanZ) > 0.20:
                meanZProp[i:i+2,j:j+2] = 0
            else:
                meanZProp[i:i+2,j:j+2] = 1 - (abs(meanHight - meanZ)/0.25)
    '''
    for i in range(64):
        for j in range(520):
            if meanZProp[i][j] == 1:
                roadProp[i][j] = roadProp[i][j] = 0      
    
    '''
    
    
    '''
    ### 位置先验
    '''
    
    
    '''
    ### 概率融合
    '''
    finalProp = np.zeros([len(frontLidar), len(frontLidar[0])])
    
    finalMap = 2* roadProp + normalVectorProp + meanZProp
    
    '''
    ###  激光雷达点投影到图像中    
    '''  
    

    '''
    plt.clf()
    plt.imshow(camera,cmap='gray')
    plt.imshow(camera)
    print(camera.shape)
    '''
    x=[]
    y=[]
    color=[]
    
    cPointCloud=np.zeros((pointCloud.shape[0],pointCloud.shape[1]))
    depthPointCloud=np.zeros((pointCloud.shape[0],1))
    countOutPoint = 0
    position_radio = np.load("positionPriorRadio.npy")
    position_Prior = np.zeros([len(frontLidar), len(frontLidar[0])])
    
    for idx in range(len(lidar_image_correspondance)):
        for jdx in range(len(lidar_image_correspondance[0])):
            point=pointCloud[int(lidar_image_correspondance[idx, jdx]),:]
            point[3]=1
            newPoint=P0.dot(Tr.dot(point.T))
            if (Tr.dot(point.T))[2] < 0:
                continue
            newPoint=newPoint/newPoint[2] 

            
            if newPoint[1]>0 and newPoint[1]<camera.shape[0]-1 and newPoint[0]>0 and newPoint[0]<camera.shape[1]-1:    
                countOutPoint += 1
                cPointCloud[countOutPoint-1,:] = pointCloud[i,:]   
                position_Prior[idx,jdx] = position_radio[int(newPoint[1]), int(newPoint[0])]
                if (finalMap[idx, jdx] + position_Prior[idx,jdx])/5 > -1:
                    x.append(int(newPoint[0]))
                    y.append(int(newPoint[1]))
                    color.append( (finalMap[idx, jdx] + position_Prior[idx,jdx])/5)
    time_end=time.time()
    print('totally cost',time_end-time_start) 
    temp=cPointCloud[0:countOutPoint,:]  
    
    plt.scatter(x,y,c=color,cmap=plt.cm.jet,marker='.',s=0.2)
    print('inpoint:',countOutPoint,pointCloud.shape[0])
    
    plt.figure()
    plt.imshow(camera2)
    plt.scatter(x,y,c=color,cmap=plt.cm.jet,marker='.',s=0.2)
    plt.savefig('lidarKittiFront_' + file_list_bin[count] + ".png")
    plt.show()
    plt.pause(0.1)
    
    

    
   









