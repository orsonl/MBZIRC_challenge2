#!/usr/bin/env python

import rosbag

import datetime
from tf.msg import tfMessage
from argparse import ArgumentParser
from geometry_msgs.msg import Quaternion
from numpy import mean, array, hypot, diff, convolve, arange, sin, cos, ones, pi, matrix
from tf.transformations import euler_from_quaternion,quaternion_from_euler,quaternion_multiply,quaternion_matrix
import tf

from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D

parser = ArgumentParser(description='Plot GPS and odometry data.')
parser.add_argument('bag', type=str, help='input bag file')
args = parser.parse_args()

bag = rosbag.Bag(args.bag)

# Read odometry/filtered points
odometry_tuples_x = []
odometry_tuples_y = []
odometry_tuples_z = []
for topic, msg, time in bag.read_messages(topics=("/odometry/filtered", "/odometry/filtered")):
        odometry_tuples_x.append(msg.pose.pose.position.x)
        odometry_tuples_y.append(msg.pose.pose.position.y)
        odometry_tuples_z.append(msg.pose.pose.position.z)

# Read odometry/gps points
gps_tuples_x = []
gps_tuples_y = []
gps_tuples_z = []
for topic, msg, time in bag.read_messages(topics=("/odometry/gps", "/odometry/gps")):
        gps_tuples_x.append(msg.pose.pose.position.x)
        gps_tuples_y.append(msg.pose.pose.position.y)
        gps_tuples_z.append(msg.pose.pose.position.z)


def getPlotSquareBounds(x_list, y_list):
    x_max = max(x_list)
    x_min = min(x_list)
    y_max = max(y_list)
    y_min = min(y_list)

    x_avg = (x_max + x_min)/ 2
    y_avg = (y_max + y_min)/ 2

    x_diff = round(x_max - x_avg, 2)
    y_diff = round(y_max - y_avg, 2)

    diff = x_diff
    if (y_diff > x_diff):
        diff = y_diff

    diff = diff*1.2

    return [x_avg + diff, x_avg - diff, y_avg + diff, y_avg - diff], x_diff*2, y_diff*2


# Plot
f, axarr = pyplot.subplots(1, 2)

axarr[0].scatter(gps_tuples_x, gps_tuples_y)
axarr[0].set_title('GPS + Odom')
axarr[0].set_aspect('equal')
axes0, width0, height0 = getPlotSquareBounds(gps_tuples_x, gps_tuples_y)
axarr[0].axis(axes0)

axarr[1].scatter(odometry_tuples_x, odometry_tuples_y)
axarr[1].set_title('Odometry')
axarr[1].set_aspect('equal')
axes1, width1, height1 = getPlotSquareBounds(odometry_tuples_x, odometry_tuples_y)
axarr[1].axis(axes1)


f.text(0.125, 0.1, "Points: " + str(len(gps_tuples_x)) )
f.text(0.125, 0.05, "Width, Height (simple): " + str(width0) + ", " + str(height0) )

f.text(0.55, 0.1, "Points: " + str(len(odometry_tuples_x)) )
f.text(0.55, 0.05, "Width, Height (simple): " + str(width1) + ", " + str(height1) )
pyplot.show()

# Fine-tune figure; hide x ticks for top plots and y ticks for right plots
#plt.setp([a.get_xticklabels() for a in axarr[0, :]], visible=False)
#plt.setp([a.get_yticklabels() for a in axarr[:, 1]], visible=False)
