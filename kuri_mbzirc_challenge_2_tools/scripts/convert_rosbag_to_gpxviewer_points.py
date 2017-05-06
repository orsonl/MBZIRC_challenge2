#!/usr/bin/env python

import rosbag

import datetime
from tf.msg import tfMessage
from argparse import ArgumentParser
from geometry_msgs.msg import Quaternion
import tf
import time



# Prompt user if yattag is missing.
try:
  from yattag import Doc, indent
except ImportError:
  print("This script requires yattag.")
  print("On Ubuntu: sudo pip install yattag")
  exit(1)

parser = ArgumentParser(description='Plot GPS and odometry data.', prefix_chars='-')
parser.add_argument('bag', metavar='FILE', type=str, help='input bag file')
parser.add_argument('-o', '--output', metavar='output', help='Output file. By default is named to "pocotrail.xml"')
args = parser.parse_args()

bag = rosbag.Bag(args.bag)

## Create XML file
prev_nan = True

doc, tag, text = Doc().tagtext()

doc.asis('<?xml version="1.0" encoding="utf-8"?>')
with tag('gpx', version="1.0"):
  with tag('trk'):

    # Create a new <trkseg> when we encounter NaN (lost fix)
    for topic, msg, t in bag.read_messages(topics=("/gps/fix", "/gps/fix")):
      if ( str(msg.latitude) != "nan" and prev_nan):
        doc.asis('<trkseg>')

      if ( str(msg.latitude) == "nan" and not prev_nan):
        prev_nan = True
        doc.asis('</trkseg>')

      if ( str(msg.latitude) != "nan"):
        prev_nan = False

        ts = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.localtime(msg.header.stamp.secs))
        with tag('trkpt', lat=msg.latitude, lon=msg.longitude):
          with tag('ele'):
            text(msg.altitude)
          with tag('time'):
            text( ts )

    # Create final closing tag
    if (not prev_nan):
      doc.asis('</trkseg>')


# Format XML
result = indent(
    doc.getvalue(),
    indentation = ' '*2,
    newline = '\r\n'
)

# Save result
filename = 'pocotrail.xml'
if args.output:
    filename = args.output

with open(filename, 'w') as file_:
    file_.write( result )

print("Saved to " + filename)
