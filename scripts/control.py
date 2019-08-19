#!/usr/bin/env python3.7

import rospy
import utm
import queue
import numpy as np

#from boat_server.msg import motor_cmd as motor_cmd_msg, imu as imu_msg, gps as gps_msg
from boat_server import msg

class BoatController():

	def __init__(self):
		rospy.Subscriber('gps', msg.gps, self._gps_callback)
		rospy.Subscriber('imu', msg.imu, self._imu_callback)
		rospy.Subscriber('waypoint', msg.imu, self._waypoint_callback)
		rospy.Subscriber('autonomy', msg.imu, self._autonomy_callback)

		self._r = rospy.Rate(1)

		self._motor_pub = rospy.Publisher('motor_cmd', msg.motor_cmd, queue_size=10)
		self._zero_cmd = msg.motor_cmd(m0=0.,m1=0.)

		self._magnetic_declination = -(14. + 32./60.)
		self._board_orientation_offset = 90

		self._origin = None
		self._curr_loc = None
		self._curr_heading = None
		self._autonomy_enabled = False
		
		self._waypoints = queue.Queue()
		self._curr_target = None
		self._curr_src = None

	def start(self):
		while not rospy.is_shutdown():
			if not self._autonomy_enabled:
				self._motor_pub.publish(self._zero_cmd)
				self._r.sleep()
			else:
				# PID control here
				pass

	def _gps_callback(self, gps_data):
		easting, northing, _, _ = utm.from_latlon(gps_data.lat, gps_data.long)
		utm_coords = np.array([easting, northing])

		# If this is the first gps reading received, assign origin
		if self._origin is None:
			self._origin = utm_coords

		self._curr_loc = utm_coords - self._origin

	def _imu_callback(self, imu_data):
		self._curr_heading = imu_data.x + self._magnetic_declination + self._board_orientation_offset

	def _waypoint_callback(self, waypoint_data):
		pass

	def _autonomy_callback(self, autonomy_data):
		pass

	@property
	def initialized(self):
		required_init = [self._origin, self._curr_loc, self._curr_heading]
		if any(val is None for val in required_init):
			return False
		else:
			return True


if __name__ == '__main__':
	rospy.init_node('boat_controller', anonymous=True)
	boat_ctrl = BoatController()

	while not boat_ctrl.initialized:
		rospy.loginfo(f"Waiting for boat controller to initialize")

	rospy.loginfo(f"Boat controller initialized, starting up")

	boat_ctrl.start()
