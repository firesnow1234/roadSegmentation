# -*- coding: utf-8 -*-
"""
Created on Wed Dec  5 10:56:49 2018

@author: Yuki
"""

import sys
 
'''
输入两点，建立直线方程y＝kx＋b。
输入第3点，求点到直线的距离。
'''
 
#输入两点p1, p2坐标
sys.stdout.write('Input two points:\n')
line = sys.stdin.readline()
x1, y1, x2, y2 = (float(x) for x in line.split())
 
#计算k,b
k = (y2 - y1) / (x2 - x1)
b = y1 - k * x1
 
#输入第三点p3坐标
sys.stdout.write('Input the third point:\n')
line = sys.stdin.readline()
x3, y3 = (float(x) for x in line.split())
 
#计算点p3到直线距离
sys.stdout.write('The dictionary is:\n')
d = abs(k * x3 - y3 + b)/((-1)*(-1) + k * k)**0.5
sys.stdout.write(str(d))
sys.stdout.write('\n')