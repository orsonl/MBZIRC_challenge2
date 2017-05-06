#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 10/04/2016
@author: Tarek Taha, Abdullah Abduldayem

Naive code to perform static exploration

"""

import rospy
import math
from math import radians, degrees, cos, sin, tan, pi, atan2
import csv
import sys, os

import actionlib

from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, PoseWithCovariance, Twist, Point, Pose, PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from kuri_mbzirc_challenge_2_msgs.msg import PanelPositionAction, PanelPositionResult

topic_name = "panel_waypoint"

class mbzirc_panel_track():
    def __init__(self):
        rospy.init_node('panel_waypoint', anonymous=True)

        ############
        ## Variables
        ############
        self.desired_dist = 0.7

        self.is_node_enabled = False
        self.current_pose = [0,0,0,0]
        #self.gotOdom = False
        self.ar_callback_start_time = rospy.Time()
        self.pose_callback_start_time = rospy.Time()

        ############
        ## Set up topic handlers
        ############
        #self.pose_sub   = rospy.Subscriber("/odometry/filtered", Odometry, self.poseCallback)

        # Start actionlib server
        self.server = actionlib.SimpleActionServer(topic_name, PanelPositionAction, self.execute, False)
        self.server.start()

        rospy.loginfo("Started panel_waypoint node. Currently on standby")


    def execute(self, goal_msg):
        # This node was called, perform any necessary changes here
        self.enableNode()
        rospy.loginfo("panel_waypoint node enabled")

        # Wait until we've detected the panel
        r = rospy.Rate(30)
        while (self.is_node_enabled and not rospy.is_shutdown() and not self.server.is_preempt_requested()):
            r.sleep()

        if (rospy.is_shutdown()):
            return

        if self.server.is_preempt_requested():
            self.server.set_preempted()
            return

        self.server.set_succeeded(self.result_)

    def enableNode(self):
        self.is_node_enabled = True
        self.marker_sub = rospy.Subscriber('/ch2/detection/panel/center_pose', PoseStamped, self.panelCallback, queue_size=5)

    def disableNode(self):
        self.is_node_enabled = False
        self.marker_sub.unregister()

    def poseCallback(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        z = data.pose.pose.position.z
        roll, pitch, yaw = euler_from_quaternion([data.pose.pose.orientation.x,
                                                 data.pose.pose.orientation.y,
                                                 data.pose.pose.orientation.z,
                                                 data.pose.pose.orientation.w])
        self.current_pose = [x,y,z,yaw]
        self.gotOdom = True

        # Show message once a second
        if (rospy.Time.now() - self.pose_callback_start_time > rospy.Duration(1)):
            self.pose_callback_start_time = rospy.Time.now()
            rospy.loginfo("Current Location x: " + str(round(x,3)) + "\ty: " + str(round(y,3)) + "\tz: " + str(round(z,3)) + "\tyaw: " + str(round(degrees(yaw), 1)))


    def panelCallback(self, data):
        ## Sensor Coordinate frame: x = right, y = down, z = front
        ## Nav Coordinate frame:    x = front, y = left, z = up

        # Return if node is not enabled
        if (not self.is_node_enabled):
            return

        rospy.loginfo("Detected marker. Setting goal")


        # Read marker position and angle
        self.marker_position = data.pose.position
        roll, pitch, yaw = euler_from_quaternion([
                                data.pose.orientation.x,
                                data.pose.orientation.y,
                                data.pose.orientation.z,
                                data.pose.orientation.w
                                ])
        self.marker_angle = pitch


        # Calculate target position
        x_goal =   self.marker_position.z - self.desired_dist*cos(self.marker_angle)
        y_goal = -(self.marker_position.x - self.desired_dist*sin(self.marker_angle))
        print("x_d = " + str(round(x_goal,3)) + " y_d = " + str(round(y_goal,3)) + " a = " + str(round(degrees(self.marker_angle))) )

        # Generate goal
        pose = [x_goal, y_goal, 0, self.marker_angle]
        goal = self.generateRelativePositionGoal(pose)

        # Return target waypoint
        self.result_ = PanelPositionResult()
        self.result_.success = True
        self.result_.waypoints = goal

        # Disable the node since it found its target
        self.disableNode()


    def generateRelativePositionGoal(self, pose):
        # Compute pose
        x = pose[0]
        y = pose[1]
        z = pose[2]
        yaw = pose[3]
        quat = quaternion_from_euler(0.0, 0.0, yaw)

        # Intialize the waypoint goal
        goal = PoseArray()

        # Use the map frame to define goal poses
        goal.header.frame_id = 'base_link'
        goal.header.stamp = rospy.Time.now()
        goal.poses.append( Pose( Point(x, y, z) , Quaternion(*quat.tolist()) ) )
        return goal


if __name__=='__main__':
  try:
      mbzirc_panel_track()
      rospy.spin()
  except rospy.ROSInterruptException:
      rospy.loginfo("mbzirc_c2_auto finished.")
