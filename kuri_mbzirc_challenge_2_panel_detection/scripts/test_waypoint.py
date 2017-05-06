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
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from decimal import *
import time
from math import radians, pi

class mbzirc_c2_auto():

  def __init__(self):
    rospy.init_node('test_waypoint', anonymous=True)

    # Enable shutdown in rospy (This is important so we cancel any move_base goals
    # when the node is killed)
    rospy.on_shutdown(self.shutdown)

    # Subscribe to the move_base action server
    self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    rospy.loginfo("Waiting for move_base action server...")

    # Wait 60 seconds for the action server to become available
    self.move_base.wait_for_server(rospy.Duration(60))

    rospy.loginfo("Connected to move base server")
    rospy.loginfo("Starting navigation test")



    # Set up the goal locations. Poses are defined in the map frame.
    self.waypoints=list()
    quaternions = list()

    # 0, clock, ?, anticlock
    euler_angles = (0, -pi/2, pi, pi/2)
    for angle in euler_angles:
        q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
        q = Quaternion(*q_angle)
        quaternions.append(q)


    # Define waypoints
    self.waypoints.append(Pose(Point(1, 0, 0.0), quaternions[0]))

    # Cycle through the waypoints
    for i in range(0, len(self.waypoints) ):
      if rospy.is_shutdown():
        break;

      # Intialize the waypoint goal
      goal = MoveBaseGoal()

      # Use the map frame to define goal poses
      goal.target_pose.header.frame_id = 'base_link'
      goal.target_pose.header.stamp = rospy.Time.now()
      goal.target_pose.pose = self.waypoints[i]

      # Start the robot moving toward the goal
      self.move(goal)

    rospy.signal_shutdown("Complete")


  def move(self, goal):
    # Send the goal pose to the MoveBaseAction server
    self.move_base.send_goal(goal)

    # Allow 1 minute to get there
    finished_within_time = self.move_base.wait_for_result(rospy.Duration(60))

    # If we don't get there in time, abort the goal
    if not finished_within_time:
      self.move_base.cancel_goal()
      rospy.loginfo("Timed out achieving goal")
    else:
      # We made it!
      state = self.move_base.get_state()
      if state == GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal succeeded!")


  def shutdown(self):
    rospy.loginfo("Stopping the robot...")
    self.move_base.cancel_goal()


if __name__ == '__main__':
  try:
    mbzirc_c2_auto()
    rospy.spin()
  except rospy.ROSInterruptException:
     rospy.loginfo("mbzirc_c2_auto finished.")
