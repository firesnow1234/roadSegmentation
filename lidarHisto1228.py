# -*- coding: utf-8 -*-
"""
Created on Fri Dec 28 20:49:28 2018

@author: Yuki
"""

import itertools
import numpy as np
import cv2
import time
import os
import copy
from math import isnan,isinf
from PIL import Image
import matplotlib.pyplot as plt
from myRansac import *
import math
import scipy.io as io



'''
# 激光雷达投影矩阵变换
'''
def project(p_in, T):
    #dimension of data and projection matrix
    dim_norm = T.shape[0]
    dim_proj = T.shape[1]
    
    # do transformation in homogenuous coordinates
    p2_in = p_in        
    p2_out = np.transpose(np.dot(T, np.transpose(p2_in)))
    
    #normalize homogeneous coordinates
    temp1 = p2_out[:, 0:dim_norm-1]
    temp2 = p2_out[:, dim_norm-1]
    temp3 = np.ones((1, dim_norm-1))    
    temp4 = temp2.reshape(len(p2_in), 1)
    
    p_out = temp1 / np.dot(temp4, temp3)
    
    return p_out


'''
# 激光雷达到图像的投影显示
# in: 
'''
def lidar2img(lidar_index_cor2, velo_img, camera, finalProp, pro_lidar):
    x = []
    y = []
    color = []
    lidarTmp = []
    for idx in range(len(lidar_index_cor2)):
        for jdx in range(len(lidar_index_cor2[0])):
            lidarIdx = lidar_index_cor2[idx, jdx]
            lidarIdx2 = int(lidarIdx)
            newPoint = velo_img[lidarIdx2, :]           
            if newPoint[1]>0 and newPoint[1]<camera2.shape[0]-1 and newPoint[0]>0 and newPoint[0]<camera2.shape[1]-1 :
                x.append(int(newPoint[0]))
                y.append(int(newPoint[1]))
                color.append(finalProp[idx, jdx])    
                #color.append(1)
                lidarTmp.append(pro_lidar[lidarIdx2, :])
    
    plt.figure()
    plt.imshow(camera2)
    plt.scatter(x, y, c=color, cmap=plt.cm.jet, marker='.', s=0.5)
    #plt.scatter(x, y, c= 'y',  marker='.', s=0.5)
    plt.show()  
    return lidarTmp

'''
### 计算均值
#in: 前方激光雷达数据frontLidar
''' 
'''
    '''
    xyzList = []
    for i in range(frontLidar.shape[0]):
        for j in range(frontLidar.shape[1]):
            if roadProp[i][j] > 0.9:
                xyzList.append(frontLidar[i,j,:])
    xyzArray = np.array(xyzList)   
    mat_path = 'my_mat_save_path'
    s[np.where(np.isnan(s))]= 0 
    io.savemat(mat_path, {'name': s[:,:3]})
    
    eng = matlab.engine.start_matlab()
    result = eng.ransac1212(500)
    Pa,Pb,Pc,Pd = result[0][0], result[0][1], result[0][2], result[0][3]  # ax + by + d = z
    roadPlane = np.array((Pa,Pb,Pc,Pd))
    '''
'''
def mean_compute_prop(frontLidar):
    meanZProp = np.zeros([len(frontLidar), len(frontLidar[0])])
    zList = []
    for i in range(frontLidar.shape[0]):
        for j in range(frontLidar.shape[1]):
            if roadProp[i][j] > 0.9:
                zList.append(frontLidar[i,j,2])
    meanZ = np.mean(zList)
    varZ = np.var(zList)
    for i in range(0, frontLidar.shape[0],2):
        for j in range(0,frontLidar.shape[1],2):
            miniBox2 = frontLidar[i:i+2,j:j+2,2]                    
            meanHight = np.mean(miniBox2)
            if abs(meanHight - meanZ) > 1:
                meanZProp[i:i+2,j:j+2] = 0
            else:
                meanZProp[i:i+2,j:j+2] = 1 - (abs(meanHight - meanZ)/1)
    return meanZProp


'''
# 提取法线方向
#in： frontLidar
#out: prop
#块大小 4*4
''' 
def normal_compute_prop(frontLidar):
    for i in range(0, frontLidar.shape[0]):
        for j in range(0, frontLidar.shape[1]):
            if isnan(frontLidar[i,j,0]) or isinf(frontLidar[i,j,0]):
                frontLidar[i,j,:] = (0,0,0)   
    
    upVector = np.array([0,0,1])
    normalVectorProp = np.zeros([len(frontLidar), len(frontLidar[0])])
    extendTag = False
    for i in range(0, frontLidar.shape[0], 2):
        for j in range(0, frontLidar.shape[1], 2):
            miniBox = frontLidar[i:i+2, j:j+2,:]
            #hDis = np.sqrt(np.sum(np.square(miniBox[0,0,:2] - miniBox[0,3,:2])))
            vDis = np.sqrt(np.sum(np.square(miniBox[0,0,:2] - miniBox[1,0,:2])))
            d = 0
            while(vDis < 0.1):
                d += 2
                miniBox = frontLidar[i:i+2+d, j:j+2+d,:]
                vDis = np.sqrt(np.sum(np.square(miniBox[0,0,:2] - miniBox[2+d-1,0,:2])))
            #x,y,z = miniBox[:,:,0], miniBox[:,:,1], miniBox[:,:,2]
            
            x,y,z = [], [], []
            for m in range(miniBox.shape[0]):
                for n in range(miniBox.shape[1]):
                    if miniBox[m,n,0] != 0 or not isnan(miniBox[m,n,0]) or not isinf(miniBox[m,n,0]):
                        x.append(miniBox[m,n,0])
                        y.append(miniBox[m,n,1])
                        z.append(miniBox[m,n,2])                       
             
            #newBox = [x.reshape(-1), y.reshape(-1), z.reshape(-1)]
            newBox = [x,y,z]
            if len(x) == 0:
                normalVectorProp[i:i+2, j:j+2+d] = 0
                continue

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
                normalVectorProp[i:i+2, j:j+2+d] = 1 - (angle2 / 30)
            else:
                normalVectorProp[i:i+2, j:j+2+d] = 0
            j += d  
            
    return normalVectorProp





lidarName = "1545205830.33.txt"
lidar1 = np.loadtxt(lidarName)
tempLidar =np.zeros((lidar1.shape[0],lidar1.shape[1]))
for i in range(64):
    tempLidar[1737*i : 1737*i+1737, :] = lidar1[lidar1.shape[0]-i: 0: -64, :] #lidar[i:lidar.shape[0]:64,:]
    
lidar = tempLidar
lidar2 =lidar[:,:4]

camera2 = cv2.imread("1545205830.28.png")

# imshow project 
velo2imgFile = 'velo2img.txt'
velo2imgTxT = np.loadtxt(velo2imgFile)
pro_lidar = lidar[: ,:4]
pro_lidar[:,3] = 1

d = 4
idxList = []
for i in range(len(pro_lidar)):
    if pro_lidar[i,0] < d or isnan(pro_lidar[i,0]):
        idxList.append(i)

mini_pro_lidar = np.delete(pro_lidar, idxList, 0)
velo_img = project(pro_lidar, velo2imgTxT)


x = []
y = []
color = []
for i in range(len(velo_img)):
    newPoint = velo_img[i,:]
    if newPoint[1]>0 and newPoint[1] < camera2.shape[0]-1 and newPoint[0]>0 and newPoint[0]<camera2.shape[1]-1 and pro_lidar[i,0] > d:
        x.append(int(newPoint[0]))
        y.append(int(newPoint[1]))
        color.append(64*d // pro_lidar[i,0])    

plt.figure()
plt.imshow(camera2)
plt.scatter(x, y, c=color, cmap=plt.cm.jet, marker='.', s=0.5)
plt.show()


if True:      
    s = copy.deepcopy(lidar2)
    s[:,3]=1   
    # new computation
    index = np.arange(0, len(s),int(len(s)/64))     #每一条激光雷达的起始位置
    angle_h = np.arctan2(s[:,0],s[:,1])/np.pi*180   #
    lidar_image_dis_temp = s.reshape(64,1737,4)     
    lidar_image_dis = np.sqrt(lidar_image_dis_temp[:,:,0]**2+  lidar_image_dis_temp[:,:,1]**2+ lidar_image_dis_temp[:,:,2]**2)
    lidar_image_x = lidar_image_dis_temp[:,:,0]
    lidar_image_y = lidar_image_dis_temp[:,:,1]
    lidar_image_z = lidar_image_dis_temp[:,:,2]
    lidar_index_correspondance  = np.zeros((lidar_image_x.shape[0], lidar_image_x.shape[1]))   # 对应下标
    leftView, rightView = 0,0
    for i in range(lidar_image_x.shape[1]):
        if lidar_image_x[31][i] > 5 and not isnan(lidar_image_x[31][i]):
            leftView = i
            break
    for i in range(lidar_image_x.shape[1]-1, 0, -1):
        if lidar_image_x[31][i] > 5 and not isnan(lidar_image_x[31][i]):
            rightView = i
            break
    for i in range(len(index) -1):  # 64
        for j in range(index[i], index[i+1]):  # 1737
            lidar_index_correspondance[i,j%1737] = j
    
    lidar_image_dis[np.where(np.isnan(lidar_image_dis))]= 0
    lidar_image_x[np.where(np.isnan(lidar_image_x))]= 0    
    lidar_image_y[np.where(np.isnan(lidar_image_y))]= 0
    lidar_image_z[np.where(np.isnan(lidar_image_z))]= 0
    
    plt.imshow(lidar_image_x, cmap=plt.cm.jet)
    plt.show()
    
    for i in range(lidar_image_x.shape[0]):
      for j in range(lidar_image_x.shape[1]):
        if lidar_image_x[i,j] < 0.001:
          lidar_image_x[i,j] = 0
        else:
          lidar_image_x[i,j]= 1 / lidar_image_x[i,j] 
    
    lidar_image_x[np.where(lidar_image_x ==np.Inf)] = 0
    
    lidar_image_front = lidar_image_x[:,leftView: rightView]     #    x[:,172:1040]
    lidar_index_cor2 = lidar_index_correspondance[:, leftView: rightView] 
    lidar_image_front2 = lidar_image_front  
  
    
    lidar_image_front3 = np.zeros((lidar_image_front.shape[0], lidar_image_front.shape[1]))
    lidar_index_cor3 = np.zeros((lidar_index_cor2.shape[0], lidar_index_cor2.shape[1]))
    for i in range(int(lidar_image_front3.shape[1])):
       lidar_image_front3[:,i] = lidar_image_front[:, lidar_image_front.shape[1]-i-1]
       lidar_index_cor3[:, i] = lidar_index_cor2[:, lidar_index_cor2.shape[1] -i-1] 
    
    frontLidar = np.zeros((lidar_index_cor3.shape[0], lidar_index_cor3.shape[1], 3 ))  # 前方激光雷达数据xyz
    for i in range(lidar_index_cor3.shape[0]):
        for j in range(lidar_index_cor3.shape[1]):
            frontLidar[i][j] = lidar2[int(lidar_index_cor3[i][j]),:3] 
            

    lidar_image_front[np.where(np.isinf(lidar_image_front))] = 0  
    plt.figure()
    plt.imshow(lidar_image_front, cmap=plt.cm.jet)    
    plt.title('front')
    plt.figure()
    plt.imshow(lidar_image_front3, cmap=plt.cm.jet)    
    plt.title('front')
              
    #直方图
    histogram = np.zeros((64,1000))
    for i in range(lidar_image_front3.shape[0]):
      for j in range(lidar_image_front3.shape[1]):
        indhis = int(lidar_image_front3[i,j]*1000)
        if indhis < 1000:
            histogram[i,indhis] += 1
    
    plt.figure()
    plt.imshow(histogram)
    plt.title('histogram')
    plt.show()      
    
    #二值化
    histArray =[]
    for x in range(20, 64):
        for y in range(1, 1000):
            if histogram[x][y] > 20:
                histArray.append([y,x])
                
    histPoints = np.array(histArray) 
    hist = []
    for i in range(len(histPoints)):
        hist.append([histPoints[i,0],histPoints[i,1]])
    
    #ransac直线拟合 y=mx+b
    m,b = plot_best_fit(hist)
    print('m:', m, '\nb:', b)
    
    #路面分割的超参数
    alpha = 0.5
    beta = 1.2
    roadSegment = np.zeros([len(lidar_image_front3), len(lidar_image_front3[0])])
    
    # i: 激光扫描线,  j: 激光雷达获取图的前方视野点
    for i in range(len(lidar_image_front3)):
        for j in range(len(lidar_image_front3[0])):
            light = int(lidar_image_front3[i,j] / 0.001)
            if light > 1000:
                light = 0
            #case1: water
            if(lidar_image_front3[i,j] == 0 and i <= m*light + beta*b):
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
    
    plt.figure()            
    plt.imshow(roadSegment,cmap=plt.cm.jet)
    plt.savefig("r.jpg")    
    plt.title('Road Segmentation')
    
    plt.figure()   
    plt.imshow(finalMap,cmap=plt.cm.jet)    
    plt.show()
    
    
    roadProp = np.zeros([len(lidar_image_front3), len(lidar_image_front3[0])])
    
    # i: 激光扫描线,  j: 激光雷达获取图的前方视野点
    #点（a，b）到直线Ax+By+C=0的距离为d=|Aa+Bb+C|/√(A^2+B^2)
    #直线 mx - y + b = 0
    maxDist = 0
    minDist = 0
    for i in range(len(lidar_image_front3)):
        for j in range(len(lidar_image_front3[0])):
            light = int(lidar_image_front3[i,j] / 0.001)
            if light == 0:
                continue
            dist = abs(m * light - i + b)/((-1)*(-1) + m * m)**0.5
            maxDist = max(maxDist, dist)
            minDist = min(minDist, dist)
    
    maxDist = 16 
    for i in range(len(lidar_image_front3)):
        for j in range(len(lidar_image_front3[0])):
            light = int(lidar_image_front3[i,j] / 0.001)
            if light == 0:
                continue
            dist = abs(m * light - i + b)/((-1)*(-1) + m * m)**0.5
            if dist > 16:
                roadProp[i,j] =  0
                continue
            roadProp[i,j] =  1 - (dist / (maxDist - minDist))
    

    
    meanZProp = mean_compute_prop(frontLidar)
    
    
    

    
    '''
    ###  激光雷达点投影到图像中    
    ###  project velo2img
    '''
    lidarTmp = lidar2img(lidar_index_cor3, velo_img, camera2,  roadProp, pro_lidar)
    
    #lidarTmp = lidar2img(lidar_index_cor3, velo_img, camera2,  normalVectorProp, pro_lidar)
    
    lidarTmp = lidar2img(lidar_index_cor3, velo_img, camera2,  meanZProp, pro_lidar)
    
    lidarTmp = lidar2img(lidar_index_cor3, velo_img, camera2,    meanZProp+ 2*roadProp, pro_lidar)

        
    
    

    
    

    
   









