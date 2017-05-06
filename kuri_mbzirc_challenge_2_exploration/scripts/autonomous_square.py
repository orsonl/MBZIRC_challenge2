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
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy as np
from decimal import *
import time
from math import radians, pi
class mbzirc_c2_auto():
    # A few key tasks are achieved in the initializer function:
    #     1. We load the pre-defined search routine
    #     2. We connect to the move_base server in ROS
    #     3. We start the ROS subscriber callback function registering the object
    #     4. We initialize counters in the class to be shared by the various callback routines
    def __init__(self):
        # Name this node, it must be unique
	rospy.init_node('autonomous', anonymous=True)

        # Enable shutdown in rospy (This is important so we cancel any move_base goals
        # when the node is killed)
        rospy.on_shutdown(self.shutdown)
        self.rest_time = rospy.get_param("~rest_time", 0.1) # Minimum pause at each location
        self.stalled_threshold = rospy.get_param("~stalled_threshold", 100) # Loops before stall

        # Goal state return values
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']
        square_size=2;
        square_size_long=4;
        square_size_short=2;
        # Set up the goal locations. Poses are defined in the map frame.
        # An easy way to find the pose coordinates is to point-and-click
        # Nav Goals in RViz when running in the simulator.
        # Pose coordinates are then displayed in the terminal
        # that was used to launch RViz.
        self.waypoints=list()
        quaternions = list()
        #euler_angles = (pi/2, pi, 3*pi/2, 0)
        euler_angles = (0, 3*pi/2, pi, pi/2,0,3*pi/2)



        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            quaternions.append(q)






        #self.waypoints.append(Pose(Point(0.0, 0.0, 0.0), quaternions[0]))
        #self.waypoints.append(Pose(Point(square_size, 0.0, 0.0), quaternions[1]))
        #self.waypoints.append(Pose(Point(square_size, -square_size, 0.0), quaternions[2]))
        #self.waypoints.append(Pose(Point(0.0, -square_size, 0.0), quaternions[3]))
        #self.waypoints.append(Pose(Point(0.0, 0.0, 0.0), quaternions[0]))
        #self.waypoints.append(Pose(Point(square_size, 0.0, 0.0), quaternions[1]))
        #self.waypoints.append(Pose(Point(square_size, -square_size, 0.0), quaternions[2]))
        #self.waypoints.append(Pose(Point(0.0, -square_size, 0.0), quaternions[3]))
        #self.waypoints.append(Pose(Point(0.0, 0.0, 0.0), quaternions[0]))


        self.waypoints.append(Pose(Point(0.0, 0.0, 0.0), quaternions[0]))
        self.waypoints.append(Pose(Point(square_size_long, 0.0, 0.0), quaternions[1]))
        self.waypoints.append(Pose(Point(square_size_long, -square_size_short, 0.0), quaternions[2]))
        self.waypoints.append(Pose(Point(0.0, -square_size_short, 0.0), quaternions[3]))
        self.waypoints.append(Pose(Point(0.0, 0.0, 0.0), quaternions[0]))
        self.waypoints.append(Pose(Point(square_size_long, 0.0, 0.0), quaternions[1]))
        self.waypoints.append(Pose(Point(square_size_long, -square_size_short, 0.0), quaternions[2]))
        self.waypoints.append(Pose(Point(0.0, -square_size_short, 0.0), quaternions[3]))
        self.waypoints.append(Pose(Point(0.0, 0.0, 0.0), quaternions[0]))









      #          ct2 = ct2+1
        #self.wp = -1
        #self.ct4 = 0

        print "Static path has : "
        print len(self.waypoints)
        print " point(s)."          


        # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")
        
        # Initialize a counter to track waypoints
        i = 0
        
        # Cycle through the four waypoints
        while i < len(self.waypoints) and not rospy.is_shutdown():
            # Update the marker display
            #self.marker_pub.publish(self.markers)
            
            # Intialize the waypoint goal
            goal = MoveBaseGoal()
            
            # Use the map frame to define goal poses
            goal.target_pose.header.frame_id = 'odom'
            
            # Set the time stamp to "now"
            goal.target_pose.header.stamp = rospy.Time.now()
            
            # Set the goal pose to the i-th waypoint
            goal.target_pose.pose = self.waypoints[i]
            
            # Start the robot moving toward the goal
            self.move(goal)
            
            i += 1



    def move(self, goal):
            # Send the goal pose to the MoveBaseAction server
            self.move_base.send_goal(goal)
            
            # Allow 1 minute to get there
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(600)) 
            
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
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        mbzirc_c2_auto()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("mbzirc_c2_auto finished.")
