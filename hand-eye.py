#!/usr/bin/env python

import time
import serial
from math import sin, cos, asin, acos, atan, sqrt, pi
import logging
logging.basicConfig(level=logging.INFO)

import time
import numpy as np
import cv2
import pyrealsense as pyrs
from scipy.stats import threshold
from scipy.ndimage.filters import gaussian_filter
from scipy.ndimage.morphology import grey_dilation, grey_erosion

#angles are expressed as follows:
#a0: rotation angle from straight towards controller
#a1: shoulder angle where -pi/2 is flat back and 0 is straight up
#a2: elbow angle where 0 folds into shoulder, pi is straight continue of shoulder
#a3: wrist up/down where pi/2 is right angle "out", pi continues elbow and 3pi/2 is straight angle "in"
#a4: wrist rotate where -pi/2 is right angle counter clockwise, 0 is straight, pi/2 is clockwise
#a5: just 0-pi where 0 is open and pi is closed

servo_angles = [[0, pi, -1800/pi, 2200], 
                [-pi/2, pi/2, 1650/pi, 1370], 
                [0, pi, 1560/pi, 740], 
                [pi/2, 3*pi/2, -1850/pi, 3275], 
                [-pi/2,pi/2, 1800/pi, 1500],
                [0,pi, 1300/pi, 700]]

l1 = 15   #cm, == 5"7/8
l2 = 18.5 #cm == 7"3/8
l3 = 11.5 #cm == 4"1/2

def xform_to_arm(coor):
    return([-coor[0]/10, -coor[1]/10, -coor[2]/10])
    
def servo_angle(i, a):
    min = servo_angles[i][0]
    max = servo_angles[i][1]
    if a < min:
        a = a + 2*pi
    if a > max:
        a = a - 2*pi
#    print 'i, a: ', i, a
    if (a < min or a > max):
        return -1
    sa = (a * servo_angles[i][2] + servo_angles[i][3])
#    print 'sa: ', sa
    return int(sa)

def move_to_angle(ser, angles, time):
# angles is a 5 tuple of angles in radians or None for no instruction to that servo
    print angles
    out = ''
    if (len(angles) != 6):
        print('angles should be a 6-tuple')
        return -1
    for (i,a) in enumerate(angles):
        if a!=None:
            sa = servo_angle(i,a)
            if (sa == -1):
                print('bad servo angle for servo', i)
                return -1
            out = out + '#{}P{}'.format(i, sa)
    out = out + 'T{}\r'.format(time)
    print out
    ser.write(out)

def zero_position(ser):
    ser.write('#0P1300#1P1000#2P1200#3P1450#4P1500#5P2000T1000\r')
    time.sleep(2)
def rest(ser):
    zero_position(ser)
    ser.write('#1P0#2P0#3P0\r')
    time.sleep(1)

def find_rotation(T3):
    if T3[0] == 0:
        b0 = pi/2
        T2 = (T3[2], T3[1])
    elif T3[0] > 0:
        b0 = atan(T3[2] / T3[0])
        T2 = (T3[0]*cos(b0) + T3[2] * sin(b0), T3[1])
    else:
        b0 = atan(T3[0] / -T3[2])
        T2 = (-T3[0]*sin(b0) + T3[2] * cos(b0), T3[1])
        b0 = pi/2 + b0
    return b0, T2
    
#T3 is the (x,y,z) coordinates in cm where (0,0,0) is the base of the arm
#x is parallel to the base board towards the controller
#y is up
#z is out
def find_angles(T3, a):
    print 'find_angles( ', T3, a, ')'
    T3 = (float(T3[0]), float(T3[1]), float(T3[2]))
    b0, T2 = find_rotation(T3)
    P2 = (T2[0] - l3*cos(a), T2[1] -l3*sin(a))
    d_sq = P2[0]*P2[0] + P2[1]*P2[1]
    d = sqrt(d_sq)
    b2 = acos( (P2[0]*P2[0]+P2[1]*P2[1]-l1*l1-l2*l2) / (-2*l1*l2) )
    c1 = asin(P2[1]/d)
    c0 = acos( (l2*l2 - l1*l1 - d_sq) / (-2*l1*d) )
    b1 = pi/2 - c1 - c0
    b3 = 3*pi/2 + a + b1 - b2
    print b0/pi*180, b1/pi*180, b2/pi*180, b3/pi*180
    return b0, b1, b2, b3

thresh_lo = 700 # that's actually the physics of the R200 - it can detect from 50cm 
thresh_hi = 1200 #ignore anything beyond 1m for now
min_contour = 100 #area of smallest contour that we care about
min_palm_circle = 10
max_depth_hist = 10

def push_depth_hist(depth_hist, d):
    if (len(depth_hist) > max_depth_hist):
        depth_hist = depth_hist[:-1]
           
    #first remove small areas of zeros
    d = grey_dilation(d, size=(5,5)) 
    #and add to history
    depth_hist.insert(0, d)
    return depth_hist            

def smooth_depth(depth_hist):
    #now give the history average for each pixel:
    #return depth_hist[0]
    smooth = sum(depth_hist) / len(depth_hist)
    return smooth
    
def get_depth(d, pixel):
    return d[pixel[1],pixel[0]]

# Good idea, but doesn't produce good results given the color of my wall...    
def filter_hand_hsv(color, depth, hand):
    mask = np.zeros(depth.shape, np.uint8)   
    cv2.drawContours(mask, [hand], 0, 255, -1)
    kernel = np.ones((5,5), np.uint8)
    dilated_mask = cv2.dilate(mask, kernel, iterations = 1)
    color_hand = cv2.bitwise_and(color, color, mask=dilated_mask)
    blur = cv2.blur(color_hand, (3,3)) # remove noise
    hsv = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)
    [H, S, V] = cv2.split(hsv)
    edges = cv2.Canny(S, 100, 200)
    #kernel = np.ones((3,3), np.uint8)
    #dilate_edges = cv2.dilate(edges, kernel, iterations = 1)
    dilate_edges = edges
    _, contours, _ = cv2.findContours(dilate_edges,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(color_hand, contours, 0, 255, 3)
    return color_hand
    #return cv2.cvtColor(dilate_edges, cv2.COLOR_GRAY2BGR)

    
def my_canny(image, sigma=0.7):
    lower = 10
    upper = 200
    edged = cv2.Canny(image, lower, upper)
    return edged

green_lower = np.array([50, 80, 30],np.uint8)
green_upper = np.array([100, 255, 255],np.uint8)
red_lower = np.array([150, 80, 30],np.uint8)
red_upper = np.array([250, 255, 255],np.uint8)

def detect_color(color, depth, lower_thresh, upper_thresh):
    #look for saturated green and return its contour and center
    hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    greenMask = cv2.inRange(hsv, lower_thresh, upper_thresh)
    # apply a series of erosions and dilations to the mask
    # using an elliptical kernel
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    greenMask = cv2.erode(greenMask, kernel, iterations = 2)
    greenMask = cv2.dilate(greenMask, kernel, iterations = 2)
     
    # blur the mask to help remove noise, then apply the
    # mask to the frame
    greenMask = cv2.GaussianBlur(greenMask, (3, 3), 0)
    
    #now find the contours of the mask
    _, contours, _ = cv2.findContours(greenMask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #print 'found ', len(contours), ' green contours'
    #and keep the biggest one:
    c = None
    c_area = 0
    c_center = None
    for contour in contours:
        M = cv2.moments(contour)
        if M['m00'] > c_area:
            c = contour
            c_area = M['m00']
            c_center = (int(M['m10']/M['m00']), int(M['m01']/M['m00']))

    return c, c_center
    
def draw_arm(c,d,arm, arm_cent):
    mask = np.zeros(d.shape, np.uint8)   
    cv2.drawContours(mask, [arm], 0, 255, -1)
    #pic = cv2.bitwise_and(c, c, mask=mask)
    pic = cv2.bitwise_and(c, c)
    cv2.drawContours(pic, [arm], 0, (255, 0, 0), 2)
    cv2.circle(pic, arm_cent, 5, [0,0,255], 2)
    cv2.putText(pic, '({:3.0f})'.format(get_depth(d, arm_cent)), (0,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0))
    return pic
    
def smooth(c, r, hist):
    if (len(hist) > 10):
        hist = hist[:-1]
    if c == None:
        cx = None
        cy = None
    else:
        cx = c[0]
        cy = c[1]
    hist.insert(0, (cx, cy ,r))
    c_sum_x = 0
    c_sum_y = 0
    r_sum = 0
    n = 0
    for elm in hist:
        if (elm[0] != None):
            c_sum_x += elm[0]
            c_sum_y += elm[1]
            r_sum += elm[2]
            n += 1
    if n > 0:
        return (c_sum_x/n, c_sum_y/n), r_sum/n, hist
    else:
        return None, None, hist

    
if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 9600)
    zero_position(ser)
    T_init = [0,20,10]
    b0,b1,b2,b3 = find_angles(T_init,0)
    move_to_angle(ser, (b0,b1,b2,b3,0,pi), 2000)
    time.sleep(2)    
    target = None
    arm = None

    pyrs.start()
    dev = pyrs.Device()
    
    #Use appropriate settings here to get the exposure you need
    dev.set_device_option(pyrs.constants.rs_option.RS_OPTION_COLOR_ENABLE_AUTO_EXPOSURE, 1)
    dev.set_device_option(pyrs.constants.rs_option.RS_OPTION_R200_LR_AUTO_EXPOSURE_ENABLED, 1)
    #dev.set_device_option(pyrs.constants.rs_option.RS_OPTION_COLOR_EXPOSURE, 3)
    #dev.set_device_option(pyrs.constants.rs_option.RS_OPTION_R200_LR_EXPOSURE, 3)

    cnt = 0
    last = time.time()
    smoothing = 0.9;
    fps_smooth = 30
    hist = []
    depth_hist = []
    #get one frame to know the dimensions
    dev.wait_for_frame()
    c = dev.colour
    cd = np.concatenate((c,c), axis=1)
    outFile = cv2.VideoWriter('/home/rafi/Videos/hand-eye.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), fps_smooth, (len(cd[0]), len(cd)), True)
    
    state = 'find_arm'
    while True:
        
        cnt += 1
        if (cnt % 10) == 0:
            now = time.time()
            dt = now - last
            fps = 10/dt
            fps_smooth = (fps_smooth * smoothing) + (fps * (1.0-smoothing))
            last = now

        dev.wait_for_frame()
        c = dev.colour
        d = dev.dac * dev.depth_scale * 1000 #this makes d be the depth in mm
        c = cv2.cvtColor(c, cv2.COLOR_RGB2BGR)
        depth_hist = push_depth_hist(depth_hist, d)
        d = smooth_depth(depth_hist)
        pic = np.zeros(c.shape, np.uint8)
        
        if (state == 'find_arm'):
            blob, blob_cent = detect_color(c,d, green_lower, green_upper)
            if blob != None:
                pic = draw_arm(c,d,blob, blob_cent)
                bx,by,bw,bh = cv2.boundingRect(blob)
                cv2.rectangle(pic,(bx,by),(bx+bw,by+bh),(0,0,255),1)
                b_area = bw*bh
                area = cv2.contourArea(blob)
                #print b_area, area, (b_area - area) / area
                if (b_area - area) < (0.25 * area):
                    arm = blob
                    arm_cent = blob_cent
                    cv2.rectangle(pic,(bx,by),(bx+bw,by+bh),(0,255,255),2)
                    arm_cent_p = np.array([arm_cent[0] , arm_cent[1]], np.uint)
                    camera_T = dev.deproject_pixel_to_point(arm_cent_p, get_depth(d, arm_cent_p))
                    print 'camera_T: ', camera_T
                    state = 'wait_target'
            else:
                pic = cv2.bitwise_and(c, c)
        if (state == 'wait_target'):
            blob, blob_cent = detect_color(c,d, red_lower, red_upper)
            if (blob != None):
                pic = draw_arm(c,d,blob, blob_cent)
                bx,by,bw,bh = cv2.boundingRect(blob)
                cv2.rectangle(pic,(bx,by),(bx+bw,by+bh),(0,0,255),1)
                b_area = bw*bh
                area = cv2.contourArea(blob)
                #print b_area, area, (b_area - area) / area
                if (b_area - area) < (0.25 * area):
                    target = blob
                    target_cent = blob_cent
                    target_cent_p = np.array([target_cent[0] , target_cent[1]], np.uint)
                    target_T = dev.deproject_pixel_to_point(target_cent_p, get_depth(d, target_cent_p))
                    cv2.rectangle(pic,(bx,by),(bx+bw,by+bh),(0,255,255),2)
                    cv2.putText(pic, 'press <space> to start.', (0,100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0))
                    cv2.putText(pic, '({},{},{})'.format(int(target_T[0]), int(target_T[1]), int(target_T[2])), (0,150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0))
                else:
                    target = None
            else:
                pic = cv2.bitwise_and(c, c)
            cv2.circle(pic, arm_cent, 5, [0,0,255], 2)
        if (state == 'move_to_target'):
            pic = cv2.bitwise_and(c, c)
            cv2.putText(pic, 'press <space> to restart.', (0,100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0))
        
        cd = np.concatenate((c,pic), axis=1)
       
        cv2.imshow('', cd)
        if (outFile != None):
            outFile.write(cd)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord(' '):
            if state == 'wait_target' and target != None:
                state = 'move_to_target'
                print 'target_T: ', target_T, 'camera_T: ', camera_T
                delta = np.array(target_T) - np.array(camera_T)
                print 'delta: ', delta
                new_T = np.array(T_init)+np.array(xform_to_arm(delta))
                print 'T_init: ', T_init, 'new_T: ', new_T
                b0,b1,b2,b3 = find_angles(new_T,0)
                move_to_angle(ser, (b0,b1,b2,b3,0,pi), 2000)
            elif state == 'move_to_target':
                state = 'wait_target'
        elif key == ord('f'):
            cv2.imwrite('tmp.jpg', cd)
            cv2.waitKey(0)

    outFile.release()
    rest(ser)

#        elif key == ord('1'):
#            T[0] = T[0]-5
#            b0,b1,b2,b3 = find_angles(T,0)
#            move_to_angle(ser, (b0,b1,b2,b3,0,pi), 2000)
#        elif key == ord('2'):
#            T[0] = T[0]+5
#            b0,b1,b2,b3 = find_angles(T,0)
#            move_to_angle(ser, (b0,b1,b2,b3,0,pi), 2000)
#        elif key == ord('3'):
#            T[1] = T[1]-5
#            b0,b1,b2,b3 = find_angles(T,0)
#            move_to_angle(ser, (b0,b1,b2,b3,0,pi), 2000)
#        elif key == ord('4'):
#            T[1] = T[1]+5
#            b0,b1,b2,b3 = find_angles(T,0)
#            move_to_angle(ser, (b0,b1,b2,b3,0,pi), 2000)
#        elif key == ord('5'):
#            T[2] = T[2]-5
#            b0,b1,b2,b3 = find_angles(T,0)
#            move_to_angle(ser, (b0,b1,b2,b3,0,pi), 2000)
#        elif key == ord('6'):
#            T[2] = T[2]+5
#            b0,b1,b2,b3 = find_angles(T,0)
#            move_to_angle(ser, (b0,b1,b2,b3,0,pi), 2000)
