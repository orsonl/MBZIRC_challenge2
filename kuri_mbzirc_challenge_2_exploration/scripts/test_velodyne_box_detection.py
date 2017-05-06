#!/usr/bin/env python

""" autonomous.py - Version 1.0 2016-10-12

    General framework based on Patrick Goebel's nav_test.py
    Initial version based on ccam-navigation by Chris Mobley
    Autonomous movement added by Jonathan Hodges

    Define waypoint destinations for a robot to move autonomously within
    a map framework.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html

"""

import rospy
import rospkg
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from kuri_mbzirc_challenge_2_msgs.msg import BoxPositionAction, BoxPositionGoal
from tf.transformations import quaternion_from_euler
from decimal import *
import time
from math import radians, pi

class mbzirc_c2_auto():

  def __init__(self):
    rospy.init_node('test_box_detection', anonymous=True)

    # Enable shutdown in rospy (This is important so we cancel any move_base goals
    # when the node is killed)
    rospy.on_shutdown(self.shutdown)

    # Subscribe to the action server
    self.client = actionlib.SimpleActionClient("get_box_cluster", BoxPositionAction)
    rospy.loginfo("Waiting for action server...")

    # Wait 60 seconds for the action server to become available
    self.client.wait_for_server(rospy.Duration(60))
    rospy.loginfo("Connected to action server")

    # Send start command
    rospy.loginfo("Sending start command")
    goal = BoxPositionGoal()
    goal.request = goal.REQUEST_START
    goal.range_max = 30
    goal.range_max = 60
    goal.angle_min = -pi/2
    goal.angle_max = pi/2
    self.execute(goal)
    rospy.loginfo("Started")

    #time.sleep(30)

    #rospy.loginfo("Sending stop command")
    #goal = BoxPositionGoal()
    #goal.request = goal.REQUEST_STOP
    #self.execute(goal)
    #rospy.loginfo("Stopped")


    rospy.signal_shutdown("Complete")


  def execute(self, goal):
    # Send the goal pose to the MoveBaseAction server
    self.client.send_goal(goal)

    # Allow 1 minute to get there
    finished_within_time = self.client.wait_for_result(rospy.Duration(60))

    # If we don't get there in time, abort the goal
    if not finished_within_time:
      self.client.cancel_goal()
      rospy.loginfo("Timed out achieving goal")
    else:
      # We made it!
      state = self.client.get_state()
      if state == GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal succeeded!")


  def shutdown(self):
    rospy.loginfo("Stopping the robot...")
    self.client.cancel_goal()


if __name__ == '__main__':
  try:
    mbzirc_c2_auto()
    rospy.spin()
  except rospy.ROSInterruptException:
     rospy.loginfo("mbzirc_c2_auto finished.")
