#!/usr/bin/env python3.7
import rospy
from boat_server import msg

import numpy as np

if __name__ == '__main__':
	pub = rospy.Publisher('waypoint_cmd', msg.waypoint_cmd, queue_size=10)
	rospy.init_node('waypoint_sender', anonymous=True)

	waypoint_list = []

	# Ingress
	#waypoint_list.append(np.array([42.406008, -71.241836]))

	# # Egress
	# waypoint_list.append(np.array([42.405735, -71.241889]))

	

	# Bottom Right
	waypoint_list.append(np.array([42.406071, -71.242046]))

	# Top Right
	waypoint_list.append(np.array([42.406347, -71.242046]))

	# Transect 2 Top
	waypoint_list.append(np.array([42.406347, -71.242225]))

	# Transect 2 Bottom
	waypoint_list.append(np.array([42.406071, -71.242225]))

	# Transect 1 Bottom
	waypoint_list.append(np.array([42.406071, -71.242421]))

	# Transect 1 Top
	waypoint_list.append(np.array([42.406347, -71.242421]))

	# Top Left
	waypoint_list.append(np.array([42.406347, -71.242608]))

	# Bottom Left
	waypoint_list.append(np.array([42.406071, -71.242608]))


	# Ingress
	#waypoint_list.append(np.array([42.406008, -71.241836]))

	# Egress
	#waypoint_list.append(np.array([42.405735, -71.241889]))


	# 
	#waypoint_list.append(np.array([]))

	# Alpha
	#waypoint_list.append(np.array([42.405818,-71.241844]))
	#waypoint_list.append(np.array([42.4057352,-71.2418894]))

	# Bravo
	#waypoint_list.append(np.array([42.405859,-71.242497]))

	# Charlie
	#waypoint_list.append(np.array([42.405832, -71.241977]))
	#waypoint_list.append(np.array([42.4057416, -71.2419300]))

	# Return to Alpha
	#waypoint_list.append(np.array([42.405818,-71.241844]))
	#waypoint_list.append(np.array([42.484911, -71.221840]))

	waypoints = [msg.waypoint(coords=wp) for wp in waypoint_list]
	waypoint_cmd = msg.waypoint_cmd(waypoints=waypoints)

	pub.publish(waypoint_cmd)