#!/usr/bin/env python
import roslib
import rospy

from nav_msgs.msg import Odometry

import tf

is_gps_enabled = False;

def odom_filtered_cb(msg):
    global is_gps_enabled
    if (is_gps_enabled):
        return

    pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
    ori = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

    br = tf.TransformBroadcaster()
    br.sendTransform(pose,
                     ori,
                     rospy.Time.now(),
                     "base_link",
                     "odom_gps")

def odom_gps_cb(msg):
    global is_gps_enabled
    is_gps_enabled = True

    """
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, 0),
                  tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                  rospy.Time.now(),
                  "odom_gps",
                  "world")
    """

if __name__ == '__main__':
    rospy.init_node('publish_odom_gps_tf')

    rospy.Subscriber('/odometry/filtered', Odometry, odom_filtered_cb)
    rospy.Subscriber('/odometry/gps', Odometry, odom_gps_cb)


    rospy.spin()
