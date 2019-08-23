#!/usr/bin/env python3.7

import rospy
import utm
import numpy as np
from threading import Lock

from boat_server import msg

class PIDController():

	def __init__(self, kP=1., kI=0., kD=0., window_size=100):
		self._kP = kP
		self._kI = kI
		self._kD = kD

		self._prev_error = None

class BoatController():

	def __init__(self):
		rospy.Subscriber('gps', msg.gps, self._gps_callback)
		rospy.Subscriber('imu', msg.imu, self._imu_callback)
		rospy.Subscriber('waypoint_cmd', msg.waypoint_cmd, self._waypoint_callback)
		rospy.Subscriber('autonomy_cmd', msg.autonomy_cmd, self._autonomy_callback)

		self._r = rospy.Rate(1)

		self._motor_pub = rospy.Publisher('motor_cmd', msg.motor_cmd, queue_size=10)
		self._eboard_pub = rospy.Publisher('eboard_cmd', msg.eboard_cmd, queue_size=10)

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
		self._curr_target = np.zeros(2)
		self._curr_src = np.zeros(2)

	def start(self):
		if not self.initialized:
			rospy.loginfo("Cannot start controller until it is initialized.")
			return

		while not rospy.is_shutdown():
			if self._autonomy_enabled:

				with self._waypoint_lock:
					if self._prev_waypoint_index > self._curr_waypoint_index:
						# waypoint indices messed up. stop motors and loginfo
						self._motor_pub.publish(msg.motor_cmd(m0=0.,m1=0.))
						self._curr_waypoint_index = self._prev_waypoint_index = -1
						rospy.loginfo("Error: prev wp index > curr wp index")

					elif self._curr_waypoint_index >= len(self._waypoints):
					 	# All waypoints have been reached, do nothing
					 	rospy.loginfo("Warning: curr wp index exceeds # of wps")
					 	pass
					elif self._curr_waypoint_index < 0:
						# No waypoints available so just send 0 command to motors
						self._motor_pub.publish(msg.motor_cmd(m0=0.,m1=0.))

					elif self._curr_waypoint_index != self._prev_waypoint_index:
						# new waypoints received or just completed a waypoint
						# reset pid stuff, update source, set prev_wp = curr_wp, update target
						if self._curr_waypoint_index == 0:
							self._curr_src = np.copy(self._curr_pos)
						elif self._prev_waypoint_index > -1:
							self._curr_src = np.copy(self._waypoints[self._prev_waypoint_index]) - self._origin

						self._curr_target = np.copy(self._waypoints[self._curr_waypoint_index]) - self._origin
						self._prev_waypoint_index = self._curr_waypoint_index

				# Compute distance to current waypoint
				diff = self._curr_target - self._curr_pos
				dist = np.linalg.norm(diff)
				if dist < self._sufficient_proximity:
					# At current waypoint
					self._motor_pub.publish(msg.motor_cmd(0., 0.))
					with self._waypoint_lock:
						if self._prev_waypoint_index == self._curr_waypoint_index and len(self._waypoints) > self._curr_waypoint_index + 1:
							self._curr_waypoint_index += 1
				else:
					# compute motor signals using line following logic and pid heading controller
					self._motor_pub.publish(msg.motor_cmd(0., 0.))
					pass

			else:
				# Autonomy disabled, do nothing
				pass

	def _gps_callback(self, gps_data):
		utm_coords = np.array(utm.from_latlon(gps_data.lat, gps_data.long)[:2])

		# If this is the first gps reading received, initialize controller
		if not self.initialized:
			self._origin = utm_coords
		
		self._curr_pos = utm_coords - self._origin

	def _imu_callback(self, imu_data):
		self._curr_heading = imu_data.x + self._magnetic_declination + self._board_orientation_offset

	def _waypoint_callback(self, waypoint_cmd):
		new_waypoint_list = [np.array(utm.from_latlon(*w.coords)[:2]) for w in waypoint_cmd.waypoints]

		with self._waypoint_lock:
			self._waypoints = new_waypoint_list
			self._curr_waypoint_index = 0
			self._prev_waypoint_index = -1

	def _autonomy_callback(self, autonomy_cmd):
		if autonomy_cmd.enabled:
			# Send Arming message to eboard
			self._eboard_pub.publish(msg.eboard_cmd(cmd='arm'))

		self._autonomy_enabled = autonomy_cmd.enabled

	@property
	def initialized(self):
		self._initialized = self._curr_heading is not None and self._origin is not None
		return self._initialized


if __name__ == '__main__':
	rospy.init_node('boat_controller', anonymous=True)
	boat_ctrl = BoatController()

	while not boat_ctrl.initialized:
		rospy.loginfo(f"Waiting for boat controller to initialize")

	rospy.loginfo(f"Boat controller initialized, starting up")

	boat_ctrl.start()
