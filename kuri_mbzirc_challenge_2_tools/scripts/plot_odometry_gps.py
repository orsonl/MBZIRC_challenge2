#!/usr/bin/env python


from argparse import ArgumentParser
import os
import rospkg
import subprocess
import webbrowser


parser = ArgumentParser(description='Plot GPS and odometry data.')
parser.add_argument('bag', metavar='FILE', type=str, help='input bag file')
args = parser.parse_args()

# Find package location
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('kuri_mbzirc_challenge_2_tools')


xml_file = os.path.join(pkg_path, 'gps/gpxviewer/pocotrail.xml')
gps_arg = "-o " + xml_file + " " + args.bag

prc = subprocess.Popen("rosrun kuri_mbzirc_challenge_2_tools convert_rosbag_to_gpxviewer_points.py " + gps_arg, shell=True)
prc.wait()


url = "file://" + pkg_path + "/gps/gpxviewer/gpxviewer-drag.html"
webbrowser.open(url,new=2)# open in a new tab, if possible


prc = subprocess.Popen("rosrun kuri_mbzirc_challenge_2_tools odometry_plot.py " + args.bag, shell=True)
prc.wait()
