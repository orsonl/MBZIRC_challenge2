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

node_name = "panel_detection_node"

clos_kernel_sz = 5;

act_panel_h = float(0.29)
act_valve_base = float(8.3/100)

#===========================================================================================
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('kuri_mbzirc_challenge_2_wrench_detection')

#img_s = cv2.imread(pkg_path + '/images/wrench_3.PNG')
img_s = cv2.imread(pkg_path + '/images/wrenches_one_contour.png')
out_s = img_s.copy()

gray_s = cv2.cvtColor(img_s, cv2.COLOR_BGR2GRAY)
thresh_s = cv2.threshold(gray_s, 128, 255, cv2.THRESH_BINARY_INV)[1]

cnts_s = cv2.findContours(thresh_s.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
cnts_s = cnts_s[0] if imutils.is_cv2() else cnts_s[1]

#cv2.drawContours(out_s, [cnts_s[0]], -1, (0, 255, 0), 2)
#cv2.imshow("out_s", out_s)
#cv2.waitKey(0)
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

class PanelDetection:

    def __init__(self, is_bypass):
        self.is_bypass = is_bypass

        self.is_node_enabled = False
        if (self.is_bypass):
            print("Bypassing: " + node_name + " node enabled")
            self.enableNode()


        ## Initialize variables
        self.tool_size = '19mm'
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

        self.lab = np.zeros((len(sizes), 1, 2))
        self.toolSizes = []

        # loop over the dictionary
        for (i, (name, size)) in enumerate(sizes.items()):
            self.lab[i] = size
            self.toolSizes.append(name)


        ## Set up callbacks
        self.bridge = CvBridge()

        self.image_width = 1
        self.image_height = 1

        self.detection_win = []
        self.win_size = 15
        self.confidence = 0

        self.panel_ROI = RegionOfInterest()
        self.panel_ROI_pub = rospy.Publisher("/ch2/detection/panel/bb_pixel", RegionOfInterest, queue_size = 1)

        self.wrenches_ROI = RegionOfInterest()
        self.wrenches_ROI_pub = rospy.Publisher("/ch2/detection/wrenches/bb_pixel", RegionOfInterest, queue_size = 1)

        self.valve_ROI = RegionOfInterest()
        self.valve_ROI_pub = rospy.Publisher("/ch2/detection/valve/bb_pixel", RegionOfInterest, queue_size = 1)

        self.panel_pos = ObjectPose()
        self.panel_pos_pub = rospy.Publisher("/ch2/detection/panel/center_pose", ObjectPose, queue_size = 1)

        self.wrenches_pos = ObjectPose()
        self.wrenches_pos_pub = rospy.Publisher("/ch2/detection/wrenches/center_pose", ObjectPose, queue_size = 1)

        self.valve_pos = ObjectPose()
        self.valve_pos_pub = rospy.Publisher("/ch2/detection/valve/center_pose", ObjectPose, queue_size = 1)


        rospy.loginfo("Started panel detection node. Currently on standby")

        while (not rospy.is_shutdown()):
            subs = self.getNumOfSubscribers()
            if (subs > 0 and not self.is_node_enabled):
                self.enableNode()

            elif (subs == 0 and self.is_node_enabled and not self.is_bypass):
                self.disableNode()

            time.sleep(0.05) # delays for 50ms

    def getNumOfSubscribers(self):
        s = self.panel_ROI_pub.get_num_connections() + \
            self.wrenches_ROI_pub.get_num_connections() + \
            self.valve_ROI_pub.get_num_connections() + \
            self.panel_pos_pub.get_num_connections() + \
            self.wrenches_pos_pub.get_num_connections() + \
            self.valve_pos_pub.get_num_connections()

        return s

    def enableNode(self):
        print("Enabled node " + node_name)
        self.is_node_enabled = True
        self.camera_sub = rospy.Subscriber('/kinect2/qhd/image_color', Image, self.camera_callback, queue_size=1, buff_size=2**24, tcp_nodelay=True)
        self.cameraInfo_sub = rospy.Subscriber("/kinect2/qhd/camera_info",CameraInfo,self.get_camera_info, queue_size = 1)

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

        self.panel_ROI = RegionOfInterest()
        self.wrenches_ROI = RegionOfInterest()
        self.valve_ROI = RegionOfInterest()

        self.panel_pos = ObjectPose()
        self.wrenches_pos = ObjectPose()
        self.valve_pos = ObjectPose()

        self.panel_pos.header.stamp.secs = self.camera_secs
        self.panel_pos.header.stamp.nsecs = self.camera_nsecs
        self.wrenches_pos.header.stamp.secs = self.camera_secs
        self.wrenches_pos.header.stamp.nsecs = self.camera_nsecs
        self.valve_pos.header.stamp.secs = self.camera_secs
        self.valve_pos.header.stamp.nsecs = self.camera_nsecs

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
        #blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        blurred = gray.copy()
        thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,3,2)
        #ret2, thresh2 = cv2.threshold(blurred,200,255,cv2.THRESH_BINARY_INV)

        # perform edge detection, then perform a dilation + erosion to close gaps in between object edges
        edged = cv2.Canny(blurred, 20, 150)
        edged = cv2.dilate(edged, None, iterations=1)
        edged = cv2.erode(edged, None, iterations=1)
        edged = cv2.erode(edged, None, iterations=1)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2,7))

        edged2 = auto_canny(blurred)
        edged3 = cv2.dilate(edged2.copy(), kernel, iterations=1)
        edged4 = cv2.erode(edged3.copy(), kernel, iterations=1)

        filled = cv2.morphologyEx(edged4, cv2.MORPH_CLOSE, kernel)

        #cv2.imshow("edged4", edged4)
        #cv2.waitKey(3)

        # find contours in the thresholded image and initialize the shape detector
        #cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cnts = cv2.findContours(edged4.copy(), cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        #cnts = cv2.findContours(filled.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]

        # sort the contours from left-to-right and initialize the
        (cnts, _) = contours.sort_contours(cnts)

        # loop over the contours
        simil = []
        cX_v= []
        cY_v= []
        wr_cent_v= []
        wrs_contour = []
        cr_contours = []
        pnl_contour = []
        t_h_v = []
        wrenches = []
        wr_count = 0
        cr_count = 0
        crc_cent_prev = (0,0)
        circleDetected = False
        toolIdentified = False
        det_flag = 0
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

            # if the contour is too large or too small, ignore it
            if area < 80 or area > 0.3*img.shape[0]*img.shape[1]:
                continue

            # approximate the contour
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.01 * peri, True)
            (x, y, w, h) = cv2.boundingRect(approx)

            aspectRatio = w / float(h)

            (xc,yc),radius = cv2.minEnclosingCircle(c)
            minEncCirArea = math.pi*(radius**2)

            minEncircleA_ratio = minEncCirArea/area

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

            dA = distance.euclidean(tl, bl)
            dB = distance.euclidean(tl, tr)

            aspectRatio2 = dB/dA

            minRectArea_ratio = (dA*dB)/area
            
            hull = cv2.convexHull(c)
            hull_area = cv2.contourArea(hull)
            solidity = float(area)/hull_area

            keepRatio = aspectRatio > 0.9 and aspectRatio < 1.2
            #keepSimilarity = retSim2 < 2.9 and retSim3 < 0.8
            keepSimilarity = retSim1 < 2 and retSim2 < 12 and retSim3 < 2.2  # tune for outdoor specially retSim2 
            #keepSolidity = solidity > 0.4 and solidity < 0.7
            #keepAreaRatio = minRectArea_ratio > 2 and minRectArea_ratio < 3

            Circle = len(approx)>8 and aspectRatio > 0.8 and aspectRatio < 1.3 and minEncircleA_ratio > 1 and minEncircleA_ratio < 1.3

            if keepRatio and keepSimilarity and cr_count == 1:

                cX = int((M["m10"] / M["m00"]))
                cY = int((M["m01"] / M["m00"]))

                cX_v = np.hstack((cX_v,cX))
                cY_v = np.hstack((cY_v,cY))

                wr_cent = (cX,cY)
              
                cent_dist = distance.euclidean(wr_cent, crc_cent)

                if cent_dist > 4*crc_h and cent_dist < 8*crc_h and h > 2*crc_h:
                    wr_count = wr_count + 1
                    wrs_contour = c

                    #cv2.drawContours(output, [box.astype("int")], -1, (0, 255, 0), 2)

                    (wrs_x, wrs_y, wrs_w, wrs_h) = cv2.boundingRect(wrs_contour)
                    cv2.rectangle(output, (wrs_x, wrs_y), (wrs_x + wrs_w, wrs_y + wrs_h), (255, 0, 0), 2)
                    self.wrenches_ROI.x_offset = wrs_x
                    self.wrenches_ROI.y_offset = wrs_y
                    self.wrenches_ROI.width = wrs_w
                    self.wrenches_ROI.height = wrs_h
                    
                    #self.wrenches_ROI_pub.publish(self.wrenches_ROI)

            if Circle:
                (crc_x, crc_y, crc_w, crc_h) = cv2.boundingRect(c)

                crc_cent = (crc_x + crc_w/2, crc_y + crc_h/2)

                # The following is added to avoid detecting co-centric circles
		circ_dist = distance.euclidean(crc_cent, crc_cent_prev)
                if circ_dist > 20 :     
                    cr_count = cr_count + 1
                    cr_contours = np.append(cr_contours,c)
                    crc_cent_prev = crc_cent
       
        if cr_count == 1:

            cr_contours = np.reshape(cr_contours,(1,-1,2))
            cr_contours = cr_contours.astype(int)
            (vlv_x, vlv_y, vlv_w, vlv_h) = cv2.boundingRect(cr_contours)
            cv2.rectangle(output, (vlv_x, vlv_y), (vlv_x + vlv_w, vlv_y + vlv_h), (255, 0, 0), 2)
            self.valve_ROI.x_offset = vlv_x
            self.valve_ROI.y_offset = vlv_y
            self.valve_ROI.width = vlv_w
            self.valve_ROI.height = vlv_h

            #self.valve_ROI_pub.publish(self.valve_ROI)

            #if errPer < 80 and  wr_count < 7:

            if wr_count == 1:
		# If the wrenches are detected, return the panel region of interest
		pnl_contour = np.append(wrs_contour,cr_contours)
		pnl_contour = np.reshape(pnl_contour,(1,-1,2))
		pnl_contour = pnl_contour.astype(int)
		(pnl_x, pnl_y, pnl_w, pnl_h) = cv2.boundingRect(pnl_contour)
		cv2.rectangle(output, (pnl_x, pnl_y), (pnl_x + pnl_w, pnl_y + pnl_h), (0, 0, 255), 2)

		rospy.loginfo("Found Panel")
                det_flag = 1
   
		self.panel_ROI.x_offset = pnl_x
		self.panel_ROI.y_offset = pnl_y
		self.panel_ROI.width = pnl_w
		self.panel_ROI.height = pnl_h

		#self.panel_ROI_pub.publish(self.panel_ROI)

		#=====================================================================================
		pnl_box = cv2.minAreaRect(pnl_contour)
		pnl_box = cv2.cv.BoxPoints(pnl_box) if imutils.is_cv2() else cv2.boxPoints(pnl_box)
		pnl_box = np.array(pnl_box, dtype="int")
		# order the points in the contour such that they appear
		# in top-left, top-right, bottom-right, and bottom-left
		# order, then draw the outline of the rotated bounding
		# box
		pnl_box = perspective.order_points(pnl_box)
		(pnl_tl, pnl_tr, pnl_br, pnl_bl) = pnl_box
		pnl_dH = distance.euclidean(pnl_tl, pnl_bl)
		pnl_dW = distance.euclidean(pnl_bl, pnl_br)

                #cv2.drawContours(output, [pnl_box.astype("int")], -1, (0, 255, 0), 2)

		#=====================================================================================

		#pnl_hight=pnl_h
                pnl_hight=pnl_dH

		pnl_Z = fx*(act_panel_h/float(pnl_hight))

		pnl_cX = pnl_x + pnl_w/2
		pnl_cY = pnl_y + pnl_h/2

		p_pxl_hom=np.matrix([[pnl_cY],[pnl_cX],[1]])
		P_mtr_hom=np.dot(K_INV,p_pxl_hom)
		P_mtr=P_mtr_hom*(pnl_Z/P_mtr_hom[2][0])

		self.panel_pos.pose.position.x = -P_mtr[0][0]
		self.panel_pos.pose.position.y = -P_mtr[1][0]
		self.panel_pos.pose.position.z = P_mtr[2][0]

                self.panel_pos.header.stamp.secs = self.camera_secs
                self.panel_pos.header.stamp.nsecs = self.camera_nsecs

                self.panel_pos.confidence = self.confidence

		#self.panel_pos_pub.publish(self.panel_pos)

		cv2.putText(output, "X={}".format(-P_mtr[0][0]), (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
		cv2.putText(output, "Y={}".format(-P_mtr[1][0]), (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
		cv2.putText(output, "Z={}".format(P_mtr[2][0]), (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

                cv2.putText(output, "Confidence={:.2f}".format(self.confidence), (30, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 0, 255), 2)

		cv2.circle(output, (pnl_cX, pnl_cY), 3, (0, 0, 255), -1)

		cv2.line(output, (WW/2,0), (WW/2,HH), (0, 0, 255), 1)
		cv2.line(output, (0,HH/2), (WW,HH/2), (0, 0, 255), 1)


		#==============================================

		wrs_cX = wrs_x + wrs_w/2
		wrs_cY = wrs_y + wrs_h/2

		wrs_pxl_hom=np.matrix([[wrs_cY],[wrs_cX],[1]])
		wrs_mtr_hom=np.dot(K_INV,wrs_pxl_hom)
		wrs_mtr=wrs_mtr_hom*(pnl_Z/wrs_mtr_hom[2][0])

		self.wrenches_pos.pose.position.x = -wrs_mtr[0][0]
		self.wrenches_pos.pose.position.y = -wrs_mtr[1][0]
		self.wrenches_pos.pose.position.z = wrs_mtr[2][0]

                self.wrenches_pos.header.stamp.secs = self.camera_secs
                self.wrenches_pos.header.stamp.nsecs = self.camera_nsecs

                self.wrenches_pos.confidence = self.confidence

		#self.wrenches_pos_pub.publish(self.wrenches_pos)

		#==============================================

		vlv_cX = vlv_x + vlv_w/2
		vlv_cY = vlv_y + vlv_h/2

		vlv_pxl_hom=np.matrix([[vlv_cY],[vlv_cX],[1]])
		vlv_mtr_hom=np.dot(K_INV,vlv_pxl_hom)
		vlv_mtr=vlv_mtr_hom*(pnl_Z/vlv_mtr_hom[2][0])

		self.valve_pos.pose.position.x = -vlv_mtr[0][0]
		self.valve_pos.pose.position.y = -vlv_mtr[1][0]
		self.valve_pos.pose.position.z = vlv_mtr[2][0]

                self.valve_pos.header.stamp.secs = self.camera_secs
                self.valve_pos.header.stamp.nsecs = self.camera_nsecs

                self.valve_pos.confidence = self.confidence

		#self.valve_pos_pub.publish(self.valve_pos)

        self.detection_win = np.append(self.detection_win,det_flag)

        if len(self.detection_win) > self.win_size:
            self.detection_win = self.detection_win[-self.win_size:]

            self.confidence = np.sum(self.detection_win)/len(self.detection_win)

        self.panel_ROI_pub.publish(self.panel_ROI)
        self.valve_ROI_pub.publish(self.valve_ROI)
        self.wrenches_ROI_pub.publish(self.wrenches_ROI)

        self.panel_pos_pub.publish(self.panel_pos)
        self.valve_pos_pub.publish(self.valve_pos)
        self.wrenches_pos_pub.publish(self.wrenches_pos)

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

      PanelDetection(is_bypass)
      rospy.spin()
