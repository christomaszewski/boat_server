#!/usr/bin/env python3.7
import rospy
from boat_server import msg

import numpy as np

if __name__ == '__main__':
	pub = rospy.Publisher('waypoint_cmd', msg.waypoint_cmd, queue_size=10)
	rospy.init_node('waypoint_sender', anonymous=True)

	waypoint_list = []

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
	waypoint_list.append(np.array([42.4848, -71.2215166667]))

	waypoints = [msg.waypoint(coords=wp) for wp in waypoint_list]
	waypoint_cmd = msg.waypoint_cmd(waypoints=waypoints)

	pub.publish(waypoint_cmd)