#!/usr/bin/env python3.7
import rospy
from std_msgs.msg import String
from boat_server.msg import imu as imu_msg, gps as gps_msg

def callback(data):
	rospy.loginfo(f"Received following data over serial: {data.data}")

def imu_callback(imu_data):
	rospy.loginfo(f"Received imu data: {imu_data.x}")

def gps_callback(gps_data):
	rospy.loginfo(f"Received gps data: {gps_data.lat}, {gps_data.long}")

def subscriber():
	rospy.init_node('test_subscriber', anonymous=True)
	rospy.Subscriber('serial_in', String, callback)
	rospy.Subscriber('imu', imu_msg, imu_callback)
	rospy.Subscriber('gps', gps_msg, gps_callback)
	rospy.spin() 

if __name__ == '__main__':
	subscriber()