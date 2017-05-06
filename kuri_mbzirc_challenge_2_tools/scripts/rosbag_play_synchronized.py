#!/usr/bin/env python

import datetime
from argparse import ArgumentParser
import time
import subprocess
import os, fnmatch
import re #regex
import sys

import rosbag

fileprefix = ['diagnostics_', 'kinect_', 'velodyne_', 'scan_']

parser = ArgumentParser(description='Plot GPS and odometry data.')
parser.add_argument('folder', type=str, help='Directory to search for rosbag files')
parser.add_argument('id', type=int, help='Id of bags to play', nargs='*', default=-1)
#args = parser.parse_args()

input_folder = ""
input_id = -1
input_args = ""

if ( len(sys.argv)==1):
    print("ERROR: Incorrect syntax. Format is:\n\t" + os.path.basename(sys.argv[0]) + ' dir [id] [args]')
    sys.exit()
if ( len(sys.argv)>1):
    input_folder = sys.argv[1]

if ( len(sys.argv)>2):
    input_id = int(sys.argv[2])

for i in range (3, len(sys.argv)):
    input_args += sys.argv[i] + " "

class RosbagInfo():
    def __init__(self):
        self.str = '[ ]'
        self.exists = False
        self.duration = 0
        self.error = False
        self.error_msg = ""


def find(pattern, path, fullpath = False):
    result = []
    for root, dirs, files in os.walk(path):
        for name in files:
            if fnmatch.fnmatch(name, pattern):
                location = name
                if (fullpath):
                    location = os.path.join(root, name)
                result.append(location)
    return result


def getRosbagInfo(path):
    r = RosbagInfo()

    exists = os.path.exists( path )
    r.exists = exists

    if (exists):
        r.str = '[x]'
        try:
            with rosbag.Bag(path, 'a') as bag:
                try:
                    r.duration = round(bag.get_end_time() - bag.get_start_time(), 2)
                except:
                    r.error = True
                    r.error_msg = 'Cannot get rosbag time from ' + path
        except Exception as err:
            r.error = True
            r.error_msg = "Error opening " + path + ": " + str(err)

    return r




# Get bag files in directory
filenames = find('*.bag', input_folder)
if (len(filenames) == 0):
    print("No rosbag files found in " + input_folder)
    sys.exit()

# Determine unique timestamps on bag files
timestamps = []

pattern = re.compile("(.*?)_(.*?).bag")
for f in filenames:
    result = pattern.match(f)
    if (result != None):
        t = result.group(2)

        if t in timestamps:
            continue

        timestamps.append( t )

if (len(filenames) == 0):
    print("No rosbag file found in " + input_folder)
    sys.exit()


# Sort in order
timestamps.sort()

# Create empty array to store bools
w, h = len(fileprefix), len(timestamps)
files_timestamped = [[False for x in range(w)] for y in range(h)]



# NO ID SPECIFIED
if (input_id < 0):
    # Print all files
    print('ID\tTimestamp\t\tDiag\tKinect\tVelo\tScan\tDuration (sec)')

    for i in range(0, len(timestamps)):
        t = timestamps[i]
        duration = 0

        output_msg = str(i) + '\t'
        output_msg += t + '\t'

        for j in range(0, len(fileprefix)):
            path = os.path.join(input_folder, fileprefix[j] + t + '.bag')
            baginfo = getRosbagInfo(path)

            output_msg += baginfo.str  + '\t'
            if baginfo.duration > 0:
                duration = baginfo.duration
            if (baginfo.error):
                print(baginfo.error_msg)

        output_msg += str(duration)
        print(output_msg)
    sys.exit()





# Play the specified files
t = timestamps[input_id]
bag_str = ""

for j in range(0, len(fileprefix)):
    path = os.path.join(input_folder, fileprefix[j] + t + '.bag')
    baginfo = getRosbagInfo(path)

    if (baginfo.exists):
        bag_str += path + " "

bag_str += input_args

try:
    rosplay = subprocess.Popen("rosbag play " + bag_str, shell=True).communicate()
except KeyboardInterrupt:
    sys.exit()
