#!/usr/bin/env python3.7
import rospy
import sys
import robot_primitives as rp


from boat_server import msg

import numpy as np

if __name__ == '__main__':
	if len(sys.argv) < 2:
		print('Please specify a path object to load')
		exit(1)

	pub = rospy.Publisher('waypoint_cmd', msg.waypoint_cmd, queue_size=10)
	rospy.init_node('waypoint_sender', anonymous=True)

	filename = sys.argv[1]
	path = rp.paths.ConstrainedPath.from_file(filename)

	waypoint_list = [np.array(pt) for pt in path.coord_list]

	waypoints = [msg.waypoint(coords=wp) for wp in waypoint_list]
	waypoint_cmd = msg.waypoint_cmd(waypoints=waypoints)

	pub.publish(waypoint_cmd)