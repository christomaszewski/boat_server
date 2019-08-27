#!/usr/bin/env python3.7
import rospy
from std_msgs.msg import String
from boat_server import msg

def motor_callback(motor_cmd):
	rospy.loginfo(f"Received motor cmd m0:{motor_cmd.m0}, m1:{motor_cmd.m0}")

def eboard_callback(eboard_cmd):
	rospy.loginfo(f"Received eboard cmd: {eboard_cmd.cmd}")

def waypoint_callback(waypoint_data):
	for w in waypoint_data.waypoints:
		rospy.loginfo(f"Received waypoint: {w.coords}")

def imu_callback(imu_data):
	rospy.loginfo(f"Received imu data: {imu_data.x}")

def gps_callback(gps_data):
	rospy.loginfo(f"Received gps data: {gps_data.lat}, {gps_data.long}")

def subscriber():
	rospy.init_node('test_subscriber', anonymous=True)
	rospy.Subscriber('imu', msg.imu, imu_callback)
	rospy.Subscriber('gps', msg.gps, gps_callback)
	rospy.Subscriber('motor_cmd', msg.motor_cmd, motor_callback)
	rospy.Subscriber('eboard_cmd', msg.eboard_cmd, eboard_callback)
	rospy.Subscriber('waypoint_cmd', msg.waypoint_cmd, waypoint_callback)
	rospy.spin() 

if __name__ == '__main__':
	subscriber()