The whole panel localization process can be divided into two parts:
---------------------------------------------------------------------------------------------------
1. Husky Navigation
consists of:
husky_wp node: Navigate the Husky through fixed waypoints until the goal point for the panel is set.
---------------------------------------------------------------------------------------------------
2. Search for panel 
consists of:
detection_velo node: Process the raw data from the Velodyne and publish the possible goal points at each time stamp.
detect_goal node:    Subscribe the possible goal points at each stamp and set the actual goal point to stop the fixed waypoint navigation executed by husky_wp node.
---------------------------------------------------------------------------------------------------
To recreate the demo in videos:

==========================
== Connect to the Velodyne
==========================
In a new terminal window, SSH to pr2-head:

sudo ssh pr2-head@192.168.1.19
sudo ifconfig eth1 192.168.1.70
sudo route add 192.168.1.201 eth1
roslaunch velodyne_pointcloud VLP16_points.launch


==========================
== Start the "Search for panel" part
==========================
In a new terminal window, SSH to pr2-head:

sudo ssh pr2-head@192.168.1.19
roslaunch kuri_mbzirc_challenge_2_exploration velodyne_part.launch


==========================
== Start the other part of the "Search for panel" part
==========================
In a new terminal window, SSH to pr2-head:

sudo ssh pr2-head@192.168.1.19
rosrun kuri_mbzirc_challenge_2_exploration detect_goal_point


==========================
== Start the "Husky Navigation" part
==========================
In a new terminal window, SSH to the Husky onboard computer:

sudo ssh administrator@192.168.1.11
Two options:
1. Make Husky wait at the same place until the goal point is set
roslaunch kuri_mbzirc_challenge_2_exploration husky_nav_part_standby.launch
2. Let Husky navigate through fixed waypoints until the goal point is set
roslaunch kuri_mbzirc_challenge_2_exploration husky_nav_part_straight.launch
