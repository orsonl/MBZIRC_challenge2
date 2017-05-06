#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 10/04/2016
@author: Tarek Taha

Naive code to perform static exploration

"""

import rospy
import math
from math import radians, degrees, cos, sin, tan, pi
import csv
import sys, os

import time
import cv2
import numpy as np

import actionlib

from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, PoseWithCovariance, Twist, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
from tf import TransformListener
from tf.transformations import quaternion_from_euler, euler_from_quaternion


# Variables
gotOdom = False
laser_min_range = 1   # Used to ignore the UR5 arm
laser_max_range = 5   # Used to ignore far points
wall_distance = 2

hough_angle_res = 5*(math.pi/180.0)#math.pi/32 # rad. Used for hough
hough_range_res = 0.1 # meters. Used for hough
hough_threshold = 10   # Minimum points to consider a line. Used for hough

def generateRelativePositionGoal(pose):
    global current_pose, past_pose;
    
    past_pose = current_pose;
    
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = '/odom'
    goal.target_pose.pose.position.x = current_pose[0] + pose[0]
    goal.target_pose.pose.position.y = current_pose[1] + pose[1]
    goal.target_pose.pose.position.z = current_pose[2] + pose[2]
   
    angle = radians(current_pose[3] + pose[3])
    quat = quaternion_from_euler(0.0, 0.0, angle)
    goal.target_pose.pose.orientation = Quaternion(*quat.tolist())
    
    return goal

def poseCallback(data):
  global current_pose, gotOdom
  
  if not hasattr(poseCallback, "start_time"):
     poseCallback.start_time = rospy.Time()  # it doesn't exist yet, so initialize it

  x = data.pose.pose.position.x
  y = data.pose.pose.position.y
  z = data.pose.pose.position.z
  roll, pitch, yaw = euler_from_quaternion([data.pose.pose.orientation.x,
                                           data.pose.pose.orientation.y,
                                           data.pose.pose.orientation.z,
                                           data.pose.pose.orientation.w])
  current_pose = [x,y,z,yaw]
  gotOdom = True
  
  # Show message once a second
  if (rospy.Time.now() - poseCallback.start_time > rospy.Duration(1)):
    poseCallback.start_time = rospy.Time.now()
    rospy.loginfo("Current Location x: " + str(x) + "y: " + str(y) + "z: " + str(z) + " yaw: " + str(degrees(yaw)))


# Generate range using floats
def frange(start, end, jump):
  out = [start]
  while True:
    start += jump
    out.append(start)
    
    if (start > end):
      break;
    
  return out
    
class HoughLine:
  def __init__(self, r, angle, votes):
    self.r = r
    self.angle = angle
    self.votes = votes

    self.m = -1/tan(self.angle)
    self.c = self.r / sin(self.angle)

  def __repr__(self):
    return repr((self.r, self.angle, self.votes))

# Hough transform designed for radial laser data
def houghTransform(r, alpha):
  global laser_min_range, hough_angle_res, hough_range_res, hough_threshold
  
  theta = frange(0, 2*math.pi, hough_angle_res)
  rho   = frange(0, max(r), hough_range_res)
  
  # Create 2D grid of theta and rho
  votes = np.zeros((len(theta), len(rho)), dtype=np.uint64)

  #votes = [[0 for i in range(len(rho))] for j in range(len(theta))]
  
  # analyze each point
  for p in range(len(r)):
    for i_t in range(len(theta)):
      r_temp = r[p]*cos(alpha[p] - theta[i_t])
      if (r_temp < 0):
	continue
      
      r_temp = hough_range_res*round(r_temp/hough_range_res) # round to nearest value of rho
      i_r = int(r_temp/hough_range_res) # find index of corresponding rho
      
      votes[i_t, i_r] += 1
  
  
  # Find max votes	    
    v_max = 0
    for i_r in range(len(rho)):
      for i_t in range(len(theta)):
        if (votes[i_t, i_r] > v_max):
          v_max = votes[i_t, i_r]
	

  """
  # Extract lines by thresholding
  line_out = [] #Output lines
  for i_r in range(len(rho)):
    for i_t in range(len(theta)):
      if (votes[i_t][i_r] >= hough_threshold): # and votes[i_t][i_r] >= v_max*0.5
	h = HoughLine(rho[i_r], theta[i_t], votes[i_t][i_r])
	line_out.append(h)
  """
  

  # Check for local maxima
  line_out = [] #Output lines
  for i_r in range(len(rho)):
      for i_t in range(len(theta)):

          if (votes[i_t, i_r] >= hough_threshold):
              isMaxima = True
              for j_r in range(i_r-2, i_r+2, 1):
                if (not isMaxima):
                    break

                k_r = j_r
                if (j_r < 0 or j_r >= len(rho)):
                    continue


                for j_t in range(i_t-2, i_t+2, 1):
                    if (i_t == j_t and i_r == j_r):
                        continue

                    # wrap around
                    k_t = j_t
                    if (j_t < 0 or j_t >= len(theta)):
                        k_t = abs( len(theta)-abs(j_t) )

                    if (votes[i_t, i_r] <= votes[k_t, k_r]):
                        isMaxima = False
                        break

              if (isMaxima):
                  h = HoughLine(rho[i_r], theta[i_t], votes[i_t, i_r])
                  line_out.append(h)


  # Sort them in descending order
  line_out = sorted(line_out, key=lambda l: l.votes, reverse=True)

  return line_out

# NextMove
def nextMove(ranges, angles):
  # Find min range
  
  r_min = ranges[0]
  r_min_index = 0
  for i in range(len(ranges)):
      if ( ranges[i] < r_min ):
          r_min = ranges[i]
          r_min_index = i

# Laser scanner callback
def scan_callback(msg):
  frame_id = msg.header.frame_id

  ## Get TF
  """
  tf = TransformListener()
  if tf.frameExists("/base_link") and tf.frameExists("/base_laser"):
      t = tf.getLatestCommonTime("/base_link", "/base_laser")
      position, quaternion = tf.lookupTransform("/base_link", "/base_laser", t)
      print position, quaternion
  """

  ranges = []
  angles = []
  
  ## Create points from laser data
  for i in range(len(msg.ranges)):
    # Skip points at infinity
    if ( math.isinf(msg.ranges[i]) ):
      continue

    # Skip points outside laser range
    if (msg.ranges[i] < laser_min_range or msg.ranges[i] > laser_max_range):
      continue
    
    # Record range and compute angle
    ranges.append(msg.ranges[i])
    angles.append(msg.angle_max - 2 * i * msg.angle_max / len(msg.ranges))

  # Extract lines with hough transform
  lines = houghTransform(ranges, angles)
  drawHoughLine(lines, "/base_laser_mount")

  line_pairs = findLinePairs(lines)
  findCorner(line_pairs)
  nextMove(ranges, angles)

def findLinePairs(lines):
  pairs = []

  for i in range(len(lines)-1):
    for j in range(i+1,len(lines)):
      t1 = lines[i].angle
      t2 = lines[j].angle

      angle = np.degrees(t1-t2)

      if (angle > 180):
        angle = angle-180
      if (angle < -180):
        angle = angle+180
      angle = abs(angle)

      if (80 <= angle <= 100):
        pairs.append([lines[i], lines[j]])

  return pairs

def findCorner(line_pairs):
  points = []

  for i in range(len(line_pairs)):
    m1 = line_pairs[i][0].m
    m2 = line_pairs[i][1].m
    c1 = line_pairs[i][0].c
    c2 = line_pairs[i][1].c

    x = (c2-c1)/(m1-m2)
    y = m1*x + c1

    points.append([x,y])
    point = [[x,y], [0,0]]
    #print(point[0])

  drawPoints(points, "/base_laser_mount")



def drawHoughLine(lines, frame_id):
  # Create markers for each
  marker = Marker()
  marker.header.frame_id = frame_id

  marker.type = marker.LINE_LIST
  marker.pose.orientation.w = 1

  marker.scale.x = 0.01
  marker.color.a = 1.0
  marker.color.b = 1.0

  # Compute two points along the line and draw a line between them
  arc = np.radians(75)
  for i in range(len(lines)):
    r = lines[i].r
    t = lines[i].angle
    r_new = r/cos(arc)

    x1, y1 = [r_new*cos(t+arc), r_new*sin(t+arc)]
    x2, y2 = [r_new*cos(t-arc), r_new*sin(t-arc)]

    p1 = Point(x1, y1, 0)
    marker.points.append(p1)

    p2 = Point(x2, y2, 0)
    marker.points.append(p2)

  # Publish markers
  line_pub.publish(marker)


def drawPoints(points, frame_id):
  # Create markers for each
  marker = Marker()
  marker.header.frame_id = frame_id

  marker.type = marker.POINTS
  marker.scale.x = 0.05
  marker.scale.y = 0.05
  marker.color.a = 1.0
  marker.color.g = 1.0

  for i in range(len(points)):
    p = Point()
    p.x = points[i][0]
    p.y = points[i][1]

    marker.points.append(p)

  # Publish markers
  point_pub.publish(marker)





if __name__=='__main__':
  rospy.init_node("find_panel")   

  # Set up subscribers and navigation stack
  current_pose = [0,0,0,0]
  scan_sub   = rospy.Subscriber('/scan', LaserScan, scan_callback, queue_size = 2)
  pose_sub   = rospy.Subscriber("/odometry/filtered", Odometry, poseCallback)
  line_pub = rospy.Publisher ("/explore/HoughLines", Marker, queue_size = 100)
  point_pub = rospy.Publisher ("/explore/Points", Marker, queue_size = 100)

  navigationActionServer = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
  rospy.loginfo("Connecting to the move Action Server")
  navigationActionServer.wait_for_server()
  rospy.loginfo("Ready ...")

  # Wait for initial odometry
  while (not gotOdom):
    time.sleep(0.01)

  starting_pose = current_pose
  starting_angle = 0
  time.sleep(200)


  # Case 1: No object detected, move around
  # Case 2: Object detected, go in front of it
    # Case 2.1: Oops, false alarm. Go back to moving
    # Case 2.2: Done positioning. Terminate
  # Case 3: Went 360, couldn't find it. Go back to exploration
  
  # 1: Move at slight angle perp to wall
  
  rospy.loginfo("Generate the desired configuration in front of panel")
  #pose = [(64, -25, 0.0, 0.0)]
  pose = [0, 0, 0, 0]
  goal = generateRelativePositionGoal(pose)
  rospy.loginfo("Moving Robot to the desired configuration in front of panel")
  navigationActionServer.send_goal(goal)
  rospy.loginfo("Waiting for Robot to reach the desired configuration in front of panel")
  navigationActionServer.wait_for_result()  

  navResult  = navigationActionServer.get_result()
  navState   = navigationActionServer.get_state()

  rospy.loginfo("Finished Navigating")
  print "Result: ", str(navResult)
  # Outcome 3 : SUCCESS, 4 : ABORTED , 5: REJECTED
  
  
  if (navState == 3):
    status = "SUCCESS"
  elif (navState == 4):
    status = "ABORTED"
  elif (navState == 5):
    status = "REJECTED"
  else:
    status = "UNKNOWN (code " + str(navState) + ")"
  
  print ("Navigation status: " + status)

    
    
 
  '''
  #while True:
  for pose in waypoints:
    rospy.loginfo("Creating navigation goal...")
    goal = generateGoal(pose)
    
    rospy.loginfo("Moving Robot desired goal")
    navigationActionServer.send_goal(goal)
	
    #to stop if obstacle is sensed in the range of laser
    while (navigationActionServer.get_state()==0 or navigationActionServer.get_state()==1):
      rospy.sleep(0.1)
      if (g_range_ahead < 29):
        navigationActionServer.cancel_goal()
        
        rospy.loginfo("Obstacle in front")
        break

    #to break out from the waypoints loop
    if (g_range_ahead < 29):
      break

  rospy.loginfo("Waiting for Robot to reach goal")
  navigationActionServer.wait_for_result()

  rospy.sleep(10.)
    
  #define the obstacle moving goal
  rospy.loginfo("Creating obstacle goal...")
  print "current_pose: ", str(current_pose)
  print "obstacle_angle", str(obstacle_angle)
  print "angle_discrete", str(angle_discrete)
  print "angle_id", str(angle_id)
  print "angle_number", str(angle_number)

  obst_goal_local_position = [(g_range_ahead - 2) * cos(obstacle_angle),(g_range_ahead - 2) * sin(obstacle_angle)]
  obst_goal_global_position = [current_pose[0] + cos(current_pose[3]) * obst_goal_local_position[0] - sin(current_pose[3]) * obst_goal_local_position[1], current_pose[1] + sin(current_pose[3]) * obst_goal_local_position[0] + cos(current_pose[3]) * obst_goal_local_position[1]]
  pose = [(obst_goal_global_position[0],obst_goal_global_position[1], 0, obstacle_angle + current_pose[3])]
  goal = generateGoal(pose)

  print "cos(current_pose[3])", str(cos(current_pose[3]))
  print "local postion: ", str(obst_goal_local_position)
  print "global postion: ", str(obst_goal_global_position)
  print "goal: ", str(pose)

  rospy.loginfo("Moving Robot to the obstacle")
  navigationActionServer.send_goal(goal)

  #to stop 3 meters away of the obstacle
  while (navigationActionServer.get_state()==0 or navigationActionServer.get_state()==1):
    rospy.sleep(0.1)
    if (g_range_ahead < 3):
      navigationActionServer.cancel_goal()

      rospy.loginfo("3 meter in front of Obstacle")
      break

  rospy.loginfo("Waiting for Robot to reach obstacle goal")
  navigationActionServer.wait_for_result()  
  '''
