#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created on 10/04/2016
@author: Abdullah Abduldayem (template)
@author: Husameldin Mukhtar 
"""

import sys
import rospy
import rospkg
import cv2
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from sensor_msgs.msg import Image, RegionOfInterest
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from kuri_mbzirc_challenge_2_msgs.msg import WrenchDetectionAction
from scipy.spatial import distance
from collections import OrderedDict
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

node_name = "wrench_detection_server"
topic_name = "wrench_detection"

clos_kernel_sz = 5;

#===========================================================================================
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('kuri_mbzirc_challenge_2_wrench_detection')

img_s = cv2.imread(pkg_path + '/images/wrench_3.PNG')
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

class WrenchDetectionServer:

	def __init__(self):
		self.is_node_enabled = True

		# Set up callbacks
		self.bridge = CvBridge()
		self.camera_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.camera_callback, queue_size=1)

		self.panel_ROI = RegionOfInterest()
		self.panel_ROI_pub = rospy.Publisher("/ch2/detection/panel_bb", RegionOfInterest, queue_size = 1)

		self.wrenches_ROI = RegionOfInterest()
		self.wrenches_ROI_pub = rospy.Publisher("/ch2/detection/wrenches_bb", RegionOfInterest, queue_size = 1)

		self.valve_ROI = RegionOfInterest()
		self.valve_ROI_pub = rospy.Publisher("/ch2/detection/valve_bb", RegionOfInterest, queue_size = 1)

		self.tool_ROI = RegionOfInterest()
		self.tool_ROI_pub = rospy.Publisher("/ch2/detection/tool_bb", RegionOfInterest, queue_size = 1)

		# Start actionlib server
		#self.server = actionlib.SimpleActionServer(topic_name, WrenchDetectionAction, self.execute, False)
		#self.server.start()

		rospy.loginfo("Started wrench detection node. Currently on standby")

		self.tool_size = '14mm'

		sizes = OrderedDict({
				"12mm": (14, 82),
				"13mm": (15, 86),
				"14mm": (16, 90),
				"15mm": (17, 94),
				"18mm": (20, 105),
				"19mm": (22, 109)})

		self.lab = np.zeros((len(sizes), 1, 2))
		self.toolSizes = []

		# loop over the dictionary
		for (i, (name, size)) in enumerate(sizes.items()):
			self.lab[i] = size
			self.toolSizes.append(name)

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


	def execute(self, goal_msg):
		# This node was called, perform any necessary changes here
		self.is_node_enabled = True
		rospy.loginfo("Wrench detection node enabled")


	def camera_callback(self, data):
		# Return if node is not enabled
		if (not self.is_node_enabled):
		    return


		# Node is enabled, process the camera data
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		img = cv_image.copy()
		#img_org = cv2.imread('/home/mbzirc-01/Pictures/wrenches_blk_2.jpg')
		#img_org = cv2.imread('/home/mbzirc-01/Pictures/panel_query.JPG')
		#img = imutils.resize(img_org, width=640)

		output = img.copy()

		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		blurred = cv2.GaussianBlur(gray, (5, 5), 0)
		#thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)[1]
		thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,3,2)

		# perform edge detection, then perform a dilation + erosion to close gaps in between object edges
		edged = cv2.Canny(blurred, 20, 150)
		edged = cv2.dilate(edged, None, iterations=1)
		edged = cv2.erode(edged, None, iterations=1)
		edged = cv2.erode(edged, None, iterations=1)

		edged2 = auto_canny(blurred)
		edged3 = cv2.dilate(edged2.copy(), None, iterations=1)
		edged4 = cv2.erode(edged3.copy(), None, iterations=1)

		kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(4,50))
		filled = cv2.morphologyEx(edged4, cv2.MORPH_CLOSE, kernel)

		"""
		cv2.imshow("thresh", thresh)
		cv2.waitKey(10)

		cv2.imshow("edged4", edged4)
		cv2.waitKey(10)

		cv2.imshow("filled", filled)
		cv2.waitKey(10)
		"""

		# find contours in the thresholded image and initialize the shape detector
		#cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		cnts = cv2.findContours(filled.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
		cnts = cnts[0] if imutils.is_cv2() else cnts[1]

		# sort the contours from left-to-right and initialize the
		(cnts, _) = contours.sort_contours(cnts)

		#zlab = ToolLabeler()

		# loop over the contours
		simil = []
		wr_contours = []
		cr_contours = []
		pnl_contour = []
		dim_v = []
		wr_count = 0
		cr_count = 0
		circleDetected = False
		toolIdentified = False
		for c in cnts:
			# compute the center of the contour, then detect the name of the
			# shape using only the contour
			M = cv2.moments(c)
			#cX = int((M["m10"] / M["m00"]))
			#cY = int((M["m01"] / M["m00"]))

			retSim = cv2.matchShapes(cnts_s[0],c,3,0.0)
			simil = np.hstack((simil,retSim))

		 
			# multiply the contour (x, y)-coordinates by the resize ratio,
			# then draw the contours and the name of the shape on the image
			c = c.astype("float")
			c *= 1
			c = c.astype("int")
			text = "{}".format(shape)

			# if the contour is too large or too small, ignore it
			if cv2.contourArea(c) < 80 or cv2.contourArea(c) > 0.1*img.shape[0]*img.shape[1]:
				continue

			# approximate the contour
			peri = cv2.arcLength(c, True)
			approx = cv2.approxPolyDP(c, 0.01 * peri, True)
			(x, y, w, h) = cv2.boundingRect(approx)
			aspectRatio = w / float(h)

			#print(len(approx),aspectRatio)
		 
			# compute the rotated bounding box of the contour
			#orig = image.copy()
			box = cv2.minAreaRect(c)
			box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
			box = np.array(box, dtype="int")
		 
			# order the points in the contour such that they appear
			# in top-left, top-right, bottom-right, and bottom-left
			# order, then draw the outline of the rotated bounding
			# box
			box = perspective.order_points(box)
			#cv2.drawContours(output, [box.astype("int")], -1, (0, 255, 0), 2)
			#print('hello',[box.astype("int")])
			(tl, tr, br, bl) = box

			dA = distance.euclidean(tl, bl)
			dB = distance.euclidean(tl, tr)

			keepRatio = aspectRatio > 0.1 and aspectRatio < 0.3
			keepSimilarity = retSim > 0.8

			Circle = len(approx)>8 and aspectRatio > 0.8 and aspectRatio < 1.2 

			if keepRatio and keepSimilarity:
				wr_count = wr_count + 1
				#wr_contours.append(c)
				wr_contours = np.append(wr_contours,c)
				#wr_contours = np.concatenate((wr_contours,c))
				cv2.drawContours(output, [c], -1, (0, 255, 0), 2)

				(t_x, t_y, t_w, t_h) = cv2.boundingRect(c)
				dim = (t_w, t_h)
				dim_v = np.append(dim_v,dim)

				size = self.label(np.array([t_w,t_h]))

				if size == self.tool_size:
					tool_contour = c
					toolIdentified = True
	

			if Circle:
				cr_count = cr_count + 1
				cr_contours = np.append(cr_contours,c) 
				#cv2.drawContours(output, [c], -1, (255, 0, 0), 2)			

			#size = zlab.label(np.array([dA,dB]))

			#print(int(bl[0]))

			#cv2.putText(output, "({:d},{:d})".format(int(dA),int(dB)), (int(bl[0])-15,int(bl[1])+25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 0, 2)
			#cv2.putText(output, size, (int(bl[0])-15,int(bl[1])+55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
		 

			# show the output image
			cv2.imshow("out1", output)
			cv2.waitKey(10)

		wr_contours = np.reshape(wr_contours,(1,-1,2))	
		wr_contours = wr_contours.astype(int)	
		(wrs_x, wrs_y, wrs_w, wrs_h) = cv2.boundingRect(wr_contours)
		cv2.rectangle(output, (wrs_x, wrs_y), (wrs_x + wrs_w, wrs_y + wrs_h), (255, 0, 0), 2)
		self.wrenches_ROI.x_offset = wrs_x
		self.wrenches_ROI.y_offset = wrs_y
		self.wrenches_ROI.width = wrs_w
		self.wrenches_ROI.height = wrs_h
		self.wrenches_ROI_pub.publish(self.wrenches_ROI)

		"""	
		wrs_Rect = cv2.minAreaRect(wr_contours)
		wrs_Rect = cv2.cv.BoxPoints(wrs_Rect) if imutils.is_cv2() else cv2.boxPoints(wrs_Rect)
		wrs_Rect = np.array(wrs_Rect, dtype="int")
		wrs_Rect = perspective.order_points(wrs_Rect)
		(wrs_tl, wrs_tr, wrs_br, wrs_bl) = wrs_Rect
		wrs_dA = distance.euclidean(wrs_tl, wrs_bl)
		wrs_dB = distance.euclidean(wrs_tl, wrs_tr)
		cv2.drawContours(output, [wrs_Rect.astype("int")], -1, (255, 0, 0), 2)
		"""

		if cr_count == 1 :

			cr_contours = np.reshape(cr_contours,(1,-1,2))
			cr_contours = cr_contours.astype(int)
			(vlv_x, vlv_y, vlv_w, vlv_h) = cv2.boundingRect(cr_contours)
			cv2.rectangle(output, (vlv_x, vlv_y), (vlv_x + vlv_w, vlv_y + vlv_h), (255, 0, 0), 2)
			self.valve_ROI.x_offset = vlv_x
			self.valve_ROI.y_offset = vlv_y
			self.valve_ROI.width = vlv_w
			self.valve_ROI.height = vlv_h

			self.valve_ROI_pub.publish(self.valve_ROI)

			# If the wrenches are detected, return the panel region of interest
			pnl_contour = np.append(wr_contours,cr_contours)
			pnl_contour = np.reshape(pnl_contour,(1,-1,2))	
			pnl_contour = pnl_contour.astype(int)	
			(pnl_x, pnl_y, pnl_w, pnl_h) = cv2.boundingRect(pnl_contour)
			cv2.rectangle(output, (pnl_x, pnl_y), (pnl_x + pnl_w, pnl_y + pnl_h), (0, 0, 255), 2)

			#print(dim_v)

			rospy.loginfo("Found Panel")

			self.panel_ROI.x_offset = pnl_x
			self.panel_ROI.y_offset = pnl_y
			self.panel_ROI.width = pnl_w
			self.panel_ROI.height = pnl_h

			self.panel_ROI_pub.publish(self.panel_ROI)

			#result = WrenchDetectionResult()

			#result.ROI = [pnl_x, pnl_y, pnl_w, pnl_h]
			#self.server.set_succeeded(result)

			# Disable the node since it found its target
			#self.is_node_enabled = False

		if toolIdentified:
			(tool_x, tool_y, tool_w, tool_h) = cv2.boundingRect(tool_contour)
			cv2.rectangle(output, (tool_x, tool_y), (tool_x + tool_w, tool_y + tool_h), (255, 0, 255), 2)

			self.tool_ROI.x_offset = tool_x
			self.tool_ROI.y_offset = tool_y
			self.tool_ROI.width = tool_w
			self.tool_ROI.height = tool_h

			self.tool_ROI_pub.publish(self.tool_ROI)
			
	

		# show the output image
		cv2.imshow("out2", output)
		cv2.waitKey(10)


		# If the wrenches are detected, return the region of interest
		#p1 = Point(0 ,0 ,0)
		#p2 = Point(10,0 ,0)
		#p3 = Point(10,10,0)
		#p4 = Point(0 ,10,0)

		#rospy.loginfo("Found wrench")
		#result = WrenchDetectionResult()

		#result.ROI = [p1, p2, p3, p4]
		#self.server.set_succeeded(result)

		# Disable the node since it found its target
		#self.is_node_enabled = False


if __name__ == '__main__':
      rospy.init_node(node_name)
      server = WrenchDetectionServer()
      rospy.spin()
