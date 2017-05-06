#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rosbag
from argparse import ArgumentParser
from matplotlib import pyplot
from numpy import nan

parser = ArgumentParser(description='Plot GPS and odometry data.')
parser.add_argument('bag', type=str, help='input bag file')
args = parser.parse_args()

bag = rosbag.Bag(args.bag)

a_time = []
a_lat = []
a_lon = []
a_fix = []
a_sat = []
a_alt = []

for topic, msg, t in bag.read_messages(topics=("/gps/nmea_sentence", "/gps/nmea_sentence")):
    # Specs: http://aprs.gids.nl/nmea/#gga

    entry = msg.sentence.split(',')

    if (entry[0].upper() != "$GPGGA"):
        continue

    time = float(entry[1])

    try:
        lat = float(entry[2])
    except ValueError:
        lat = nan

    try:
        lon = float(entry[4])
    except ValueError:
        lon = nan

    fix = int(entry[6])
    sat = int(entry[7])
    hdop = float(entry[8])

    try:
        alt = float(entry[9])
    except ValueError:
        alt = nan

    try:
        hgeoid = float(entry[11])
    except ValueError:
        hgeoid = nan

    try:
        dt_dgps = float(entry[13])
    except ValueError:
        dt_dgps = -1

    a_time.append(t.to_sec())
    a_lat.append(lat)
    a_lon.append(lon)
    a_fix.append(fix)
    a_sat.append(sat)
    a_alt.append(alt)


# Plot
f, axarr = pyplot.subplots(4, 1)

axarr[0].scatter(a_time, a_lat)
axarr[0].set_ylabel('Latitude')
axarr[0].ticklabel_format(axis='y', style='plain', useOffset=False)

axarr[1].scatter(a_time, a_lon)
axarr[1].set_ylabel('Longitude')
axarr[1].ticklabel_format(axis='y', style='plain', useOffset=False)

axarr[2].scatter(a_time, a_alt)
axarr[2].set_ylabel('Altitude')
axarr[2].ticklabel_format(axis='y', style='plain', useOffset=False)

axarr[3].scatter(a_time, a_sat)
axarr[3].set_xlabel('Time')
axarr[3].set_ylabel('# of satellites')
axarr[3].set_ylim(0,13)
axarr[3].ticklabel_format(axis='y', style='plain', useOffset=False)

"""
axarr[0].scatter(a_time, a_fix)
axarr[0].set_ylabel('GPS Fix')
axarr[0].set_ylim(0,3)
axarr[0].ticklabel_format(axis='y', style='plain', useOffset=False)
"""

for a in axarr:
  a.old_xlim = a.get_xlim()  # store old values so changes can be detected

def re_zoom(event):
    zoom = 1.0
    for ax in event.canvas.figure.axes: # get the change in scale
        nx = ax.get_xlim()
        ox = ax.old_xlim
        if ox != nx:                    # of axes that have changed scale
            zoom = (nx[1]-nx[0])/(ox[1]-ox[0])

    for ax in event.canvas.figure.axes: # change the scale
        nx = ax.get_xlim()
        ox = ax.old_xlim
        if ox == nx:                    # of axes that need an update
            mid = (ox[0] + ox[1])/2.0
            dif = zoom*(ox[1] - ox[0])/2.0
            nx = (mid - dif, mid + dif)
            ax.set_xlim(*nx)
        ax.old_xlim = nx
    if zoom != 1.0:
        event.canvas.draw()             # re-draw the canvas (if required)

pyplot.connect('motion_notify_event', re_zoom)  # for right-click pan/zoom
pyplot.connect('button_release_event', re_zoom) # for rectangle-select zoom

pyplot.show()
