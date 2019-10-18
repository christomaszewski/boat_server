#!/usr/bin/env python3.7
import rospy
from boat_server import msg
import random

def publisher():
	gps_pub = rospy.Publisher('gps', msg.gps, queue_size=10)
	imu_pub = rospy.Publisher('imu', msg.imu, queue_size=10)

	rospy.init_node('test_publisher', anonymous=True)
	r = rospy.Rate(10)

	gps_pub_msg = msg.gps(42.405818,-71.241844)
	imu_pub_msg = msg.imu(180., 180., 180., 0, 0, 0, 0)

	gps_pub.publish(gps_pub_msg)
	imu_pub.publish(imu_pub_msg)

if __name__ == '__main__':
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
