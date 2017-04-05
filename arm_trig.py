#!/usr/bin/env python

# -*- coding: utf-8 -*-
"""
Created on Thu Mar 30 20:35:27 2017

@author: rafi
"""

import cv2 # complete overkill, but I know how to use that for drawing
import numpy as np
from math import sin, cos, asin, acos, atan, sqrt, pi

def find_rotation(T3):
    b3 = atan(T3[2] / T3[0])
    T2 = (T3[0]*cos(b3) + T3[2] * sin(b3), T3[1])
    return b3, T2
    
def find_angles(T3, a, l1, l2, l3):
    b3, T2 = find_rotation(T3)
    print b3, T2
    P2 = (T2[0] - l3*cos(a), T2[1] -l3*sin(a))
    print P2
    d_sq = P2[0]*P2[0] + P2[1]*P2[1]
    d = sqrt(d_sq)
    print d
    b1 = acos( (P2[0]*P2[0]+P2[1]*P2[1]-l1*l1-l2*l2) / (-2*l1*l2) )
    c1 = asin(P2[1]/d)
    c0 = acos( (l2*l2 - l1*l1 - d_sq) / (-2*l1*d) )
    b0 = pi/2 - c1 - c0
    b2 = pi/2 - a - b0 + b1
    print b0/pi*180, b1/pi*180, b2/pi*180, b3/pi*180
    return b0, b1, b2, b3
    
def to_img(P):
    return (P[0] + 100, 100 - P[1])
    
#assuming b3 == 0 for now    
def draw_arm(b0, b1, b2, b3, l1, l2, l3):
    img = np.zeros([300, 300, 3], np.uint8)
    P0 = (0,0)
    P1 = (P0[0] + int(l1*cos(pi/2-b0)), P0[1] + int(l1*sin(pi/2-b0)))
    P2 = (int(P1[0]+l2*sin(b1-b0)), int(P1[1] - l2*cos(b1-b0)))
    P3 = (int(P2[0]+l3*sin(b2-b1+b0)), int(P2[1] + l3*cos(b2-b1+b0)))
    print P1, P2, P3
    cv2.line(img, to_img(P0), to_img(P1), (0,255,0))
    cv2.line(img, to_img(P1), to_img(P2), (255, 0, 0))
    cv2.line(img, to_img(P2), to_img(P3), (0,0,255))
    cv2.imshow('arm', img)
    cv2.waitKey(0)
    
if __name__ == '__main__': 
    l1 = 50.0
    l2 = 40.0
    l3 = 10.0
    a = 0
    T3 = (50.0, 40.0, 0)
    
    b0, b1, b2, b3 = find_angles((10,30,5), a, l1, l2, l3)
    draw_arm(b0, b1, b2, b3, l1, l2, l3)
    b0, b1, b2, b3 = find_angles((10,30,10), a, l1, l2, l3)
    draw_arm(b0, b1, b2, b3, l1, l2, l3)
    b0, b1, b2, b3 = find_angles((10,30,20), a, l1, l2, l3)
    draw_arm(b0, b1, b2, b3, l1, l2, l3)