# MBZIRC_challenge2

## General Description
The package includes my work in Mohamed Bin Zayed International Robotics Challenge(MBZIRC) 2017.

The package is a collaboration work between Orson Lin and Kahlifa University, Abu Dhabi, UAE.

I created this repository only to provide easier access of my work to the public. I do not take the full credit. 

The original repository is: kuri-kustar/kuri_mbzirc_challenge_2. 

## Installing

Follow the steps below to install the simulation environment with all it's dependencies.

cd <catkin_ws>

wstool init src

wstool set -t src kuri_mbzirc_challenge_2 https://github.com/kuri-kustar/kuri_mbzirc_challenge_2.git --git

wstool merge -t src https://raw.githubusercontent.com/kuri-kustar/kuri_mbzirc_challenge_2/master/mbzirc.rosinstall

wstool update -t src

rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO

catkin build

## Overview

The folder structure is as follows:

kuri_mbzirc_challenge_2: Simulation and robot environmentkuri_mbzirc_challenge_2_locating_panel: Code for locating the panel in the arena
kuri_mbzirc_challenge_2_detecting_panel: Positioning after detecting the panel; nbv + vision panel detection
kuri_mbzirc_challenge_2_system_coordinator: State machine
kuri_mbzirc_challenge_2_wrench_detection: Wrench detection
