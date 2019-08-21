#!/usr/bin/env python3.7

import rospy
import utm
import numpy as np
from threading import Lock

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

		self._initialized = False
		self._magnetic_declination = -(14. + 32./60.)
		self._board_orientation_offset = 90
		self._sufficient_proximity = 3.0

		self._origin = None
		self._curr_pos = None
		self._curr_heading = None
		self._autonomy_enabled = False
		
		self._waypoint_lock = Lock()
		self._waypoints = []
		self._curr_waypoint_index = -1
		self._prev_waypoint_index = -1
		self._curr_target = None
		self._curr_src = None

	def start(self):
		if not self.initialized:
			rospy.loginfo("Cannot start controller until it is initialized.")
			return

		while not rospy.is_shutdown():
			if self._autonomy_enabled:
				diff = self._curr_target - self._curr_pos
				dist = np.linalg.norm(diff)
				if dist < self._sufficient_proximity:
					# At current waypoint
					self._motor_pub.publish(msg.motor_cmd(0., 0.))

			else:
				# Autonomy disabled, do nothing
				pass

	def _gps_callback(self, gps_data):
		easting, northing, _, _ = utm.from_latlon(gps_data.lat, gps_data.long)
		utm_coords = np.array([easting, northing])

		# If this is the first gps reading received, initialize controller
		if not self.initialized:
			self._initialize(utm_coords)

		self._curr_pos = utm_coords - self._origin

	def _imu_callback(self, imu_data):
		self._curr_heading = imu_data.x + self._magnetic_declination + self._board_orientation_offset

	def _waypoint_callback(self, waypoint_data):
		pass

	def _autonomy_callback(self, autonomy_data):
		pass

	def _waypoint_available(self):
		return len(self._waypoints) > 0

	def _initialize(self, location):
		self._origin = location


	@property
	def initialized(self):
		return self._initialized


if __name__ == '__main__':
	rospy.init_node('boat_controller', anonymous=True)
	boat_ctrl = BoatController()

	while not boat_ctrl.initialized:
		rospy.loginfo(f"Waiting for boat controller to initialize")

	rospy.loginfo(f"Boat controller initialized, starting up")

	boat_ctrl.start()
