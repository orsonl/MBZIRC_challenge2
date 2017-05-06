#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created on 10/04/2016
@author: Abdullah Abduldayem (template)
"""

import rospy
import actionlib
import math
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from kuri_mbzirc_challenge_2_msgs.msg import WrenchDetectionAction

node_name = "wrench_detection_server"
topic_name = "wrench_detection"

class WrenchDetectionServer:

    def __init__(self, is_bypass):
        ##########
        ## Initialize state machine interface
        ##########
        self.is_bypass = is_bypass

        self.is_node_enabled = False
        if (self.is_bypass):
            self.is_node_enabled = True

        # Start actionlib server
        self.server = actionlib.SimpleActionServer(topic_name, WrenchDetectionAction, self.execute, False)
        self.server.start()

        rospy.loginfo("Started " + node_name + " node. Currently on standby")

        ##########
        ## Initialize node
        ##########
        # Variables
        self.variable = 0

        # Set up callbacks
        self.camera_sub = rospy.Subscriber('/camera', Image, self.camera_callback, queue_size=20)


    def execute(self, goal_msg):
        # This node was called, perform any necessary changes here
        self.is_node_enabled = True
        rospy.loginfo(node_name + " node enabled")


    def camera_callback(self, data):
        # Return if node is not enabled
        if (not self.is_node_enabled):
            return


        # Node is enabled, process the camera data

        # ...



        # If the wrenches are detected, return the region of interest
        p1 = Point(0 ,0 ,0)
        p2 = Point(10,0 ,0)
        p3 = Point(10,10,0)
        p4 = Point(0 ,10,0)

        rospy.loginfo("Found wrench")
        result = WrenchDetectionResult()

        result.ROI = [p1, p2, p3, p4]
        self.server.set_succeeded(result)

        # Disable the node since it found its target
        if (not self.is_bypass):
            self.is_node_enabled = False



if __name__ == '__main__':
      rospy.init_node(node_name)

      is_bypass = False
      if len(sys.argv) >= 2:
          is_bypass = True

      server = WrenchDetectionServer(is_bypass)
      rospy.spin()
