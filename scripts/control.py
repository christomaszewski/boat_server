#!/usr/bin/env python3.7

import rospy
import utm
import numpy as np
from threading import Lock
from collections import deque

from boat_server import msg
from std_msgs.msg import Float64


class PIDController():

	def __init__(self, kP=0.35, kI=0., kD=0.):
		""" Initializes PID controller. Max error magnitude is pi in our case """
		self._kP = kP
		self._kI = kI
		self._kD = kD

		self._prev_error = None
		self._prev_time = None
		self._error_integral = 0.
		self._error_derivative = 0.

		self._signal_limit = 1.0

	def reset(self):
		self._error_integral = 0.
		self._error_derivative = 0.
		self._prev_time = None
		self._prev_error = None

	def update(self, error, t):
		if self._prev_error is not None and self._prev_time is not None:
			dt = t - self._prev_time
			self._error_derivative = (error - self._prev_error)/dt
			self._error_integral += dt*(self._prev_error + error)/2

		signal = self._kP*error + self._kI*self._error_integral + self._kD*self._error_derivative

		output_signal = np.clip(signal, -self._signal_limit, self._signal_limit)

		self._prev_error = error
		self._prev_time = t

		return output_signal

class BoatController():

	def __init__(self):
		rospy.Subscriber('gps', msg.gps, self._gps_callback)
		rospy.Subscriber('imu', msg.imu, self._imu_callback)
		rospy.Subscriber('waypoint_cmd', msg.waypoint_cmd, self._waypoint_callback)
		rospy.Subscriber('autonomy_cmd', msg.autonomy_cmd, self._autonomy_callback)

		self._r = rospy.Rate(10)

		self._motor_pub = rospy.Publisher('motor_cmd', msg.motor_cmd, queue_size=10)
		self._eboard_pub = rospy.Publisher('eboard_cmd', msg.eboard_cmd, queue_size=10)

		self._initialized = False

		# North should be 0
		self._magnetic_declination = 14. + 25./60.
		self._imu_transform = lambda x: np.radians(360-((x-270-self._magnetic_declination)%360))
		self._sufficient_proximity = 1.5
		self._max_lookahead = 3.0

		self._base_thrust = 0.2
		self._min_thrust = 0.0

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
		self._target_seg = self._curr_target - self._curr_src
		self._target_seg_length = 0.
		self._target_seg_angle = 0.

		self._pid = PIDController()

		# Debugging publishers
		self._pos_pub = rospy.Publisher('debug_pos', msg.debug_pos, queue_size=10)
		self._target_pub = rospy.Publisher('debug_target', msg.debug_pos, queue_size=10)
		self._src_pub = rospy.Publisher('debug_src', msg.debug_pos, queue_size=10)
		self._lookahead_pub = rospy.Publisher('debug_lookahead', msg.debug_pos, queue_size=10)
		self._heading_pub = rospy.Publisher('debug_heading', Float64, queue_size=10)
		self._desired_heading_pub = rospy.Publisher('debug_desired', Float64, queue_size=10)

	def start(self):
		if not self.initialized:
			rospy.loginfo("Cannot start controller until it is initialized.")
			return

		while not rospy.is_shutdown():
			self._r.sleep()
			if self._autonomy_enabled:

				with self._waypoint_lock:
					if self._prev_waypoint_index > self._curr_waypoint_index:
						# waypoint indices messed up. stop motors and loginfo
						self._motor_pub.publish(msg.motor_cmd(m0=0.,m1=0.))
						self._curr_waypoint_index = self._prev_waypoint_index = -1
						rospy.logwarn("Error: prev wp index > curr wp index")

					elif self._curr_waypoint_index >= len(self._waypoints):
					 	# All waypoints have been reached, do nothing
					 	rospy.logwarn("Warning: curr wp index exceeds # of wps")
					 	pass
					elif self._curr_waypoint_index < 0:
						# No waypoints available so just send 0 command to motors
						rospy.loginfo("No waypoints available, sending 0 motor signal")
						self._motor_pub.publish(msg.motor_cmd(m0=0.,m1=0.))

					elif self._curr_waypoint_index != self._prev_waypoint_index:
						# new waypoints received or just completed a waypoint
						# reset pid stuff, update source, set prev_wp = curr_wp, update target
						if self._curr_waypoint_index == 0:
							self._curr_src = np.copy(self._curr_pos)
						elif self._prev_waypoint_index > -1:
							self._curr_src = np.copy(self._waypoints[self._prev_waypoint_index]) - self._origin

						self._curr_target = np.copy(self._waypoints[self._curr_waypoint_index]) - self._origin

						self._src_pub.publish(msg.debug_pos(*self._curr_src))
						self._target_pub.publish(msg.debug_pos(*self._curr_target))
						
						rospy.loginfo("Starting new waypoint...")
						rospy.loginfo(f"origin:{self._origin}, src:{self._curr_src}, target:{self._curr_target}, boat position:{self._curr_pos}")

						self._prev_waypoint_index = self._curr_waypoint_index
						self._target_seg = self._curr_target - self._curr_src
						self._target_seg_length = np.linalg.norm(self._target_seg)
						self._target_seg_angle = np.arctan2(*self._target_seg[::-1])

						self._pid.reset()

				# Compute distance to current waypoint
				diff_from_target = self._curr_target - self._curr_pos
				dist_to_target = np.linalg.norm(diff_from_target)
				rospy.loginfo_throttle(5, f"distance to current waypoint: {dist_to_target}")
				if dist_to_target < self._sufficient_proximity:
					# At current waypoint
					rospy.loginfo(f"Boat at current target: {self._curr_target}")
					self._motor_pub.publish(msg.motor_cmd(0., 0.))
					with self._waypoint_lock:
						if self._prev_waypoint_index == self._curr_waypoint_index and len(self._waypoints) > self._curr_waypoint_index + 1:
							self._curr_waypoint_index += 1
							rospy.loginfo(f"Waypoint reached, incrementing waypoint index to {self._curr_waypoint_index}")
				else:
					# compute motor signals using line following logic and pid heading controller
					curr_seg = self._curr_pos - self._curr_src
					dist_from_src = np.linalg.norm(curr_seg)
					angle_from_src = np.arctan2(*curr_seg[::-1])

					rospy.loginfo_throttle(1,f"curr_seg: {curr_seg}, dist_from_src: {dist_from_src}, angle_from_src: {angle_from_src}")

					projected_length = np.dot(self._target_seg, curr_seg)/self._target_seg_length
					dist_from_ideal_line = abs(np.cross(self._target_seg, curr_seg))/self._target_seg_length
					lookahead_dist = self._max_lookahead*(1-np.tanh(0.2*abs(dist_from_ideal_line)))

					rospy.loginfo_throttle(1,f"projected_length: {projected_length}, dist_from_ideal_line: {dist_from_ideal_line}, lookahead_dist: {lookahead_dist}")

					# If lookahead point is beyond target just use target
					if projected_length + lookahead_dist > self._target_seg_length:
						lookahead_point = self._curr_target
					else:
						lookahead_point = (projected_length+lookahead_dist)*(self._target_seg/self._target_seg_length) + self._curr_src

					self._lookahead_pub.publish(msg.debug_pos(*lookahead_point))

					rospy.loginfo_throttle(1,f"current boat position: {self._curr_pos}, lookahead point: {lookahead_point}")

					diff_from_lookahead = lookahead_point - self._curr_pos

					desired_heading = np.arctan2(*diff_from_lookahead[::-1])
					heading_error = self._normalize_angle(desired_heading - self._curr_heading)

					self._desired_heading_pub.publish(desired_heading)

					heading_signal = self._pid.update(heading_error, rospy.get_time())
					rospy.loginfo_throttle(1,f"desired_heading:{desired_heading}, current_heading:{self._curr_heading}, heading_error:{heading_error}, heading_signal:{heading_signal}")	
					
					# Decay desired thrust as vehicle gets close to target
					desired_thrust = self._base_thrust
					if dist_to_target < 2*self._sufficient_proximity:
						desired_thrust *= (dist_to_target/self._sufficient_proximity - 1)

					# Limit desired thrust to >= 0
					desired_thrust = max(0., desired_thrust)

					m0 = np.clip(desired_thrust - heading_signal, -1., 1.)
					m1 = np.clip(desired_thrust + heading_signal, -1., 1.)

					rospy.loginfo_throttle(1,f"sending motor signals m0:{m0}, m1:{m1}")

					self._motor_pub.publish(msg.motor_cmd(m0, m1))

			else:
				# Autonomy disabled, do nothing
				rospy.loginfo_throttle(1,"Autonomy disabled.")
				
	def _gps_callback(self, gps_data):
		utm_coords = np.array(utm.from_latlon(gps_data.lat, gps_data.long)[:2])

		# If this is the first gps reading received, initialize controller
		if not self.initialized:
			self._origin = utm_coords
			rospy.loginfo(f"Got first gps position, setting origin to: {utm_coords}")
		
		self._curr_pos = utm_coords - self._origin

		self._pos_pub.publish(msg.debug_pos(*self._curr_pos))

	def _imu_callback(self, imu_data):
		self._curr_heading = self._imu_transform(imu_data.x)

		self._heading_pub.publish(self._curr_heading)

	def _waypoint_callback(self, waypoint_cmd):
		new_waypoint_list = [np.array(utm.from_latlon(*w.coords)[:2]) for w in waypoint_cmd.waypoints]

		with self._waypoint_lock:
			self._waypoints = new_waypoint_list
			self._curr_waypoint_index = 0
			self._prev_waypoint_index = -1

		rospy.loginfo(f"Got new waypoints: {new_waypoint_list}")

	def _autonomy_callback(self, autonomy_cmd):
		if autonomy_cmd.enabled:
			# Send Arming message to eboard
			self._eboard_pub.publish(msg.eboard_cmd(cmd='arm'))

		self._autonomy_enabled = autonomy_cmd.enabled

	def _normalize_angle(self, angle):
		if angle > np.pi:
			angle -= 2*np.pi
		elif angle < -np.pi:
			angle += 2*np.pi

		return angle

	@property
	def initialized(self):
		self._initialized = self._curr_heading is not None and self._origin is not None
		return self._initialized

if __name__ == '__main__':
	rospy.init_node('boat_controller', anonymous=True)
	boat_ctrl = BoatController()

	while not boat_ctrl.initialized:
		rospy.loginfo_throttle(5,f"Waiting for boat controller to initialize")

	rospy.loginfo(f"Boat controller initialized, starting up")

	boat_ctrl.start()
