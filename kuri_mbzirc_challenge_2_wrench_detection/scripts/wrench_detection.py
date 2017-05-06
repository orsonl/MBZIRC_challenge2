#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created on 10/04/2016
@author: Husameldin Mukhtar
@author: Abdullah Abduldayem (template) 
"""

import sys
import rospy
import rospkg
import cv2
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import Image, RegionOfInterest
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial import distance
from collections import OrderedDict
from kuri_mbzirc_challenge_2_msgs.msg import ObjectPose
import numpy as np
from numpy.linalg import inv
from matplotlib import pyplot as plt
from pylab import *
from imutils import perspective
from imutils import contours
import imutils
from datetime import datetime
import time
import os
import glob
from os.path import expanduser
home = expanduser("~")

node_name = "wrench_detection_node"

clos_kernel_sz = 5;

act_wrs_w = float(0.30)

#===========================================================================================
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('kuri_mbzirc_challenge_2_wrench_detection')

#img_s = cv2.imread(pkg_path + '/images/wrench_3.PNG')
img_s = cv2.imread(pkg_path + '/images/wrenches_one_contour.png')
out_s = img_s.copy()

#cv2.imshow("img_s", img_s)
#cv2.waitKey(10)

gray_s = cv2.cvtColor(img_s, cv2.COLOR_BGR2GRAY)
thresh_s = cv2.threshold(gray_s, 128, 255, cv2.THRESH_BINARY_INV)[1]

#cv2.imshow("thresh_s", thresh_s)
#cv2.waitKey(10)

#cnts = cv2.findContours(thresh1.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
cnts_s = cv2.findContours(thresh_s.copy(), cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
cnts_s = cnts_s[0] if imutils.is_cv2() else cnts_s[1]
#=============================================================================================

def auto_canny(image, sigma=0.33):
    # compute the median of the single channel pixel intensities
    v = np.median(image)

    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)

    # return the edged image
    return edged

class WrenchDetection:

    def __init__(self, is_bypass):
        self.is_bypass = is_bypass

        self.is_node_enabled = False
        if (self.is_bypass):
            print("Bypassing: " + node_name + " node enabled")
            self.enableNode()

        ## Initialize variables
        self.tool_size = '19mm'

        sizes = OrderedDict({
            "16mm": (36, 200),
            "17mm": (38, 205),
            "18mm": (40, 215),
            "19mm": (42, 230),
            "20mm": (45, 240),
            "21mm": (47, 250),
            "22mm": (49, 260),
            "23mm": (51, 270),
            "24mm": (53, 280),
            "25mm": (55, 290),
            "26mm": (57, 300)})

        """        
        sizes = OrderedDict({
            "15mm": (32.8, 200),
            "16mm": (33.9, 210),
            "17mm": (35.9, 220),
            "18mm": (37.8, 230),
            "19mm": (39.9, 240),
            "20mm": (41.4, 250),
            "21mm": (43.4, 260),
            "22mm": (45.9, 270),
            "23mm": (47.8, 280),
            "24mm": (49.9, 290),
            "25mm": (51.2, 300),
            "26mm": (53.2, 310)})
        """
        """

        sizes = OrderedDict({
            "15mm": (32.8),
            "16mm": (33.9),
            "17mm": (35.9),
            "18mm": (37.8),
            "19mm": (39.9),
            "20mm": (41.4),
            "21mm": (43.4),
            "22mm": (45.9),
            "23mm": (47.8),
            "24mm": (49.9),
            "25mm": (51.2),
            "26mm": (53.2)})
        """

        self.lab = np.zeros((len(sizes), 1, 2))
        self.toolSizes = []

        # loop over the dictionary
        for (i, (name, size)) in enumerate(sizes.items()):
            self.lab[i] = size
            self.toolSizes.append(name)


        self.bridge = CvBridge()
        self.image_width = 1
        self.image_height = 1

        self.tool_indx_vec =[]
        self.win_size = 15
        self.right_tool_idx = -1
        self.confidence = 0

        ## Set up callbacks
        self.tool_ROI = RegionOfInterest()
        self.tool_ROI_pub = rospy.Publisher("/ch2/detection/tool/bb_pixel", RegionOfInterest, queue_size = 1)

        self.tool_pos = ObjectPose()
        self.tool_pos_pub = rospy.Publisher("/ch2/detection/tool/center_pose", ObjectPose, queue_size = 1)

        rospy.loginfo("Started wrench detection node. Currently on standby")

        while (not rospy.is_shutdown()):
            subs = self.getNumOfSubscribers()
            if (subs > 0 and not self.is_node_enabled):
                self.enableNode()

            elif (subs == 0 and self.is_node_enabled and not self.is_bypass):
                self.disableNode()

            time.sleep(0.05) # delays for 50ms

    def getNumOfSubscribers(self):
        s = self.tool_ROI_pub.get_num_connections() + \
            self.tool_pos_pub.get_num_connections()

        return s

    def enableNode(self):
        print("Enabled node " + node_name)
        self.is_node_enabled = True
        self.camera_sub = rospy.Subscriber('/softkinetic_camera/rgb/image_mono', Image, self.camera_callback, queue_size=1, buff_size=2**24, tcp_nodelay=True)
        self.cameraInfo_sub = rospy.Subscriber("/softkinetic_camera/rgb/camera_info",CameraInfo,self.get_camera_info, queue_size = 1)

    def disableNode(self):
        print("Disabled node " + node_name)
        self.is_node_enabled = False
        self.camera_sub.unregister()
        self.cameraInfo_sub.unregister()

    def get_camera_info(self, msg):
        self.image_width = msg.width
        self.image_height = msg.height
        self.camera_K = msg.K
        self.camera_secs = msg.header.stamp.secs
        self.camera_nsecs = msg.header.stamp.nsecs

    def label(self, cntD):

        # initialize the minimum distance found thus far
        minDist = (np.inf, None)

        # loop over the known label values
        for (i, row) in enumerate(self.lab):
                d = distance.euclidean(row[0], cntD)

                # if the distance is smaller than the current distance,
                # then update the bookkeeping variable
                if d < minDist[0]:
                        minDist = (d, i)

        # return the size label with the smallest distance
        return self.toolSizes[minDist[1]]


    def camera_callback(self, data):
        # Return if node is not enabled
        if (not self.is_node_enabled):
            return


        # Node is enabled, process the camera data
        try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
                print(e)

        self.tool_ROI = RegionOfInterest()
        self.tool_pos = ObjectPose()

        self.tool_pos.header.stamp.secs = self.camera_secs
        self.tool_pos.header.stamp.nsecs = self.camera_nsecs

        WW=self.image_width
        HH=self.image_height

        fx=self.camera_K[0]
        fy=self.camera_K[4]
        u0=self.camera_K[5]
        v0=self.camera_K[2]

        K=np.matrix([[fx, 0, u0, 0], [0, fy, v0, 0], [0, 0, 1, 0]])
        K_INV=pinv(K)

        img = cv_image.copy()

        output = img.copy()

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (11, 11), 0)
        #thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)[1]
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,11,2)

        # perform edge detection, then perform a dilation + erosion to close gaps in between object edges
        edged = cv2.Canny(blurred, 20, 150)
        edged = cv2.dilate(edged, None, iterations=1)
        edged = cv2.erode(edged, None, iterations=1)
        edged = cv2.erode(edged, None, iterations=1)

        edged2 = auto_canny(blurred)
        edged3 = cv2.dilate(edged2.copy(), None, iterations=1)
        edged4 = cv2.erode(edged3.copy(), None, iterations=1)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(12,24))
        filled = cv2.morphologyEx(edged4, cv2.MORPH_CLOSE, kernel)


        # find contours in the thresholded image and initialize the shape detector
        #cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        #cnts = cv2.findContours(edged4.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cnts = cv2.findContours(filled.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]

        # sort the contours from left-to-right and initialize the
        (cnts, _) = contours.sort_contours(cnts)

        # loop over the contours
        simil = []
        cX_v= []
        cY_v= []
        wr_cent_v= []
        wr_contours = []
        t_h_v = []
        wr_tc_v = []
        wrenches = []
        wr_count = 0
        toolIdentified = False
        for c in cnts:
            # compute the center of the contour, then detect the name of the
            # shape using only the contour
            M = cv2.moments(c)
            hu = cv2.HuMoments(M)

            retSim1 = cv2.matchShapes(cnts_s[0],c,1,0.0)
            retSim2 = cv2.matchShapes(cnts_s[0],c,2,0.0)
            retSim3 = cv2.matchShapes(cnts_s[0],c,3,0.0)

            # multiply the contour (x, y)-coordinates by the resize ratio,
            # then draw the contours and the name of the shape on the image
            c = c.astype("float")
            c *= 1
            c = c.astype("int")
            text = "{}".format(shape)

            area = cv2.contourArea(c)

            # approximate the contour
            #peri = cv2.arcLength(c, True)
            #approx = cv2.approxPolyDP(c, 0.01 * peri, True)
            (x, y, w, h) = cv2.boundingRect(c)

            #print(img.shape[0])
            # if the contour is too large or too small, ignore it
            if h < 0.3*img.shape[0] or x<5 or y<5 or x+w > WW-5 or y+h > HH-5:
                continue

            aspectRatio = w / float(h)

            (xc,yc),radius = cv2.minEnclosingCircle(c)
            minEncCirArea = math.pi*(radius**2)

            minEncircleA_ratio = minEncCirArea/(area+1)

            # compute the rotated bounding box of the contour
            box = cv2.minAreaRect(c)
            box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
            box = np.array(box, dtype="int")

            # order the points in the contour such that they appear
            # in top-left, top-right, bottom-right, and bottom-left
            # order, then draw the outline of the rotated bounding
            # box
            box = perspective.order_points(box)
            (tl, tr, br, bl) = box

            #out3=img.copy()
            #print(aspectRatio,retSim1,retSim2,retSim3) 
            #cv2.drawContours(out3, [c], -1, (0, 0, 255), 3)
            #cv2.imshow("out3", out3)
            #cv2.waitKey(3)

            dA = distance.euclidean(tl, bl)
            dB = distance.euclidean(tl, tr)

            aspectRatio2 = dB/dA

            minRectArea_ratio = (dA*dB)/(area+1)
            
            hull = cv2.convexHull(c)
            hull_area = cv2.contourArea(hull)
            solidity = float(area)/(hull_area+1)

            keepRatio = aspectRatio > 0.9 and aspectRatio < 1.15
            #keepSimilarity = retSim1 < 2.9 and retSim2 < 2 and retSim3 < 1.5  # tune for outdoor specially retSim2 
            keepSimilarity = retSim2 < 2 and retSim3 < 1.0 
            #keepSolidity = solidity > 0.4 and solidity < 0.7
            #keepAreaRatio = minRectArea_ratio > 2 and minRectArea_ratio < 3

            #Circle = len(approx)>8 and aspectRatio > 0.8 and aspectRatio < 1.3 and minEncircleA_ratio > 1 and minEncircleA_ratio < 1.3

            if keepRatio and keepSimilarity:

                wr_count = wr_count + 1

                #cX = int((M["m10"] / M["m00"]))
                #cY = int((M["m01"] / M["m00"]))

                #cX_v = np.hstack((cX_v,cX))
                #cY_v = np.hstack((cY_v,cY))

                #wr_cent = (cX,cY)

                wrs_contour = c

                cv2.rectangle(output, (x,y), (x+w,y+h), (255,0,0), 2)

                cv2.drawContours(output, [box.astype("int")], -1, (0, 0, 255), 2)
                cv2.drawContours(output, [c], -1, (0, 255, 0), 2)

                subimg = img[y+h/2:y+h,x:x+w]
                subfilled = filled[y+h/2:y+h,x:x+w]
                subout = subimg.copy()

                #cv2.imshow("subimg", subimg)
                #cv2.waitKey(3)

                subcnts = cv2.findContours(subfilled.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                subcnts = subcnts[0] if imutils.is_cv2() else subcnts[1]

                detected_sizes = []
                wr_subcnts = ()
                if len(subcnts) > 0: 

                    # sort the contours from left-to-right and initialize the
                    (subcnts, _) = contours.sort_contours(subcnts)

                    subcnt_idx = 0
                    for subc in subcnts:
                        
                        cv2.drawContours(subout, [subc], -1, (0, 255, 0), 2)

                        (x_sub, y_sub, w_sub, h_sub) = cv2.boundingRect(subc)

                        # compute the rotated bounding box of the contour
                        sbox = cv2.minAreaRect(subc)
                        sbox = cv2.cv.BoxPoints(sbox) if imutils.is_cv2() else cv2.boxPoints(sbox) 
                        sbox = np.array(sbox, dtype="int")

                        # order the points in the contour such that they appear
                        # in top-left, top-right, bottom-right, and bottom-left
                        # order, then draw the outline of the rotated bounding
                        # box
                        sbox = perspective.order_points(sbox)
                        (tl_sub, tr_sub, br_sub, bl_sub) = sbox

                        dH_sub = distance.euclidean(tl_sub, bl_sub)
                        dW_sub = distance.euclidean(tl_sub, tr_sub)

                        #if dH_sub*dW_sub*1.0 < 0.02*h*w:
                        if h_sub < 0.2*h:
                            continue

                        subcnt_idx = subcnt_idx + 1;

                        wrs_Z = fx*(act_wrs_w/w)

                        wr_h=wrs_Z*dH_sub/fx
                        wr_w=wrs_Z*dW_sub/fx

                        #wr_h=wrs_Z*h_sub/fx
                        #wr_w=wrs_Z*w_sub/fx

                        h_offset = wrs_Z*(h/2)/fx - 0.01

                        tool_wm = wr_w
                        tool_hm= wr_h + h_offset

                        #pnl_cX = pnl_x + pnl_w/2

                        #pnl_cY = pnl_y + pnl_h/2

                        #p_pxl_hom=np.matrix([[pnl_cY],[pnl_cX],[1]])
                        #P_mtr_hom=np.dot(K_INV,p_pxl_hom)
                        #P_mtr=P_mtr_hom*(pnl_Z/P_mtr_hom[2][0])

                        cv2.drawContours(subout, [sbox.astype("int")], -1, (0, 0, 255), 2)

                        #cv2.rectangle(output, (x_sub+x,y_sub+y), (x_sub+x+w_sub,y_sub+y+h_sub+h/2), (255,0,0), 2)

                        #print(tl_sub)
                        #print('Hello')
                        cv2.putText(subout, "W={:.2f}".format(wr_w*1000), (x_sub,y_sub+h_sub/2-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                        cv2.putText(subout, "H={:.2f}".format((wr_h + h_offset)*1000), (x_sub,y_sub+h_sub/2+10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
                        #cv2.circle(output, (pnl_cX, pnl_cY), 3, (0, 0, 255), -1)

                        size = self.label(np.array([tool_wm*1000,tool_hm*1000]))
                        #size = self.label(np.array([tool_wm*1000]))

                        detected_sizes=np.append(detected_sizes,size) 

                        #print(type(wr_subcnts),type(subc))
                        wr_subcnts = wr_subcnts + (subc,)

                        if size == self.tool_size:
                            #tool_contour = wr
                            toolIdentified = True

                            tool_indx = subcnt_idx    
                      
                            #tool_x = x_sub+x 
                            #tool_y = y_sub+y
                            #tool_w = w_sub
                            #tool_h = h_sub+h/2
                            
                            #print(tool_x,tool_y,tool_w,tool_h)
                            #print(x,y,w,h,x_sub,y_sub,w_sub,h_sub)

                cv2.imshow("subout", subout)
                cv2.waitKey(3)

                #print(detected_sizes)


                if len(detected_sizes) > 4 and len(detected_sizes) < 7 :
                    if toolIdentified:

                        self.tool_indx_vec = np.append(self.tool_indx_vec,tool_indx)  # is np.append memory an issue?
                        self.tool_indx_vec = self.tool_indx_vec[-2*self.win_size:]

                        tool_indx_win = self.tool_indx_vec[-self.win_size:] 
                        hist, bin_edges = np.histogram(tool_indx_win,array(range(1,len(detected_sizes)+1)), density=True)

                        self.right_tool_idx = np.argmax(hist)
                        self.confidence = hist[self.right_tool_idx]
                        #print(array(range(1,len(detected_sizes)+1)))

                    if self.right_tool_idx > 0 and len(self.tool_indx_vec) > self.win_size:      
                        (x_sub, y_sub, w_sub, h_sub) = cv2.boundingRect(wr_subcnts[self.right_tool_idx])
                        tool_x = x_sub+x 
                        tool_y = y_sub+y
                        tool_w = w_sub
                        tool_h = h_sub+h/2

                        #subout2 = subimg.copy()
                        #cv2.rectangle(subout2, (int(x_sub), int(y_sub)), (int(x_sub) + int(w_sub), int(y_sub) + int(h_sub)), (255, 0, 255), 2)
                        #cv2.imshow("subout2", subout2)
                        #cv2.waitKey(10)
                        
                        cv2.rectangle(output, (int(tool_x), int(tool_y)), (int(tool_x) + int(tool_w), int(tool_y) + int(tool_h)), (255, 0, 255), 2)

                        self.tool_ROI.x_offset = tool_x
                        self.tool_ROI.y_offset = tool_y
                        self.tool_ROI.width = tool_w
                        self.tool_ROI.height = tool_h

                        #self.tool_ROI_pub.publish(self.tool_ROI)

                        tool_cX = tool_x + tool_w/2
                        tool_cY = tool_y + tool_h/2

                        tool_pxl_hom=np.matrix([[tool_cY],[tool_cX],[1]])
                        tool_mtr_hom=np.dot(K_INV,tool_pxl_hom)
                        tool_mtr=tool_mtr_hom*(wrs_Z/tool_mtr_hom[2][0])

                        cv2.putText(output, "X={}".format(-tool_mtr[0][0]), (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                        cv2.putText(output, "Y={}".format(-tool_mtr[1][0]), (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                        cv2.putText(output, "Z={}".format(tool_mtr[2][0]), (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

                        cv2.putText(output, "Confidence={:.2f}".format(self.confidence), (30, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 0, 255), 2)

                        cv2.circle(output, (int(tool_cX), int(tool_cY)), 3, (0, 0, 255), -1)

                        self.tool_pos.pose.position.x = -tool_mtr[0][0]
                        self.tool_pos.pose.position.y = -tool_mtr[1][0]
                        self.tool_pos.pose.position.z = tool_mtr[2][0]

                        #self.tool_pos.header.stamp.secs = int(str(self.camera_secs)[-3:]) 
                        self.tool_pos.header.stamp.secs = self.camera_secs
                        self.tool_pos.header.stamp.nsecs = self.camera_nsecs 

                        self.tool_pos.confidence = self.confidence

                        #self.tool_pos_pub.publish(self.tool_pos)
                            

        self.tool_pos_pub.publish(self.tool_pos)
        self.tool_ROI_pub.publish(self.tool_ROI)

        # show the output image
        cv2.imshow("out2", output)
        cv2.waitKey(3)


if __name__ == '__main__':
    rospy.init_node(node_name)

    is_bypass = False
    for i in range(0,len(sys.argv)):
        if sys.argv[1] == "-e":
          is_bypass = True
          break

    WrenchDetection(is_bypass)
    rospy.spin()
