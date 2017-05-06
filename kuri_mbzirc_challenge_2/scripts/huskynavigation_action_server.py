#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 10/04/2016
@author: Tarek Taha
Updated by: Dongming

"""

import rospy
import actionlib
import math
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from kuri_mbzirc_challenge_2_msgs.msg import HuskynavAction, HuskynavResult

class Huskynavigationserver:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('husky_navigate', HuskynavAction, self.execute, False)
        self.server.start()
        self.g_range_ahead = 0
        self.current_pose = [0,0,0,0]

        self.navigationActionServer = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("Connecting to the move Action Server")
        self.navigationActionServer.wait_for_server()
        rospy.loginfo("Ready ...")

    def execute(self, goal_msg):
        for w in goal_msg.waypoints.poses:
            rospy.loginfo("Executing waypoint")
            goal = self.generateGoal(goal_msg.waypoints.header.frame_id, w)
            self.navigationActionServer.send_goal(goal)

            # Check if we got preempted while navigating
            r = rospy.Rate(30)
            while (not rospy.is_shutdown() and not self.server.is_preempt_requested() and self.navigationActionServer.simple_state != actionlib.SimpleGoalState.DONE ):
                r.sleep()

            if (rospy.is_shutdown() or self.server.is_preempt_requested()):
                # End current movement
                rospy.loginfo("Stopping the robot...")
                self.navigationActionServer.cancel_goal()
                rospy.sleep(2)

                cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
                cmd_vel_pub.publish(Twist())
                rospy.sleep(1)


                # Tell the server that we got preempted
                self.server.set_preempted()

                return

            # Everything was okay, we reached our goal. Execute next waypoint

        result = HuskynavResult()
        result.success = True
        rospy.loginfo("Husky navigation Succeeded")
        self.server.set_succeeded(result)

    def generateGoal(self, frame, pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame
        goal.target_pose.pose = pose

        return goal

if __name__ == '__main__':
      rospy.init_node("husky_navigation_server")   
      server = Huskynavigationserver()
      rospy.spin()
