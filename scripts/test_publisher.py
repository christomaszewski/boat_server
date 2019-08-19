#!/usr/bin/env python3.7
import rospy
from boat_server.msg import imu as imu_msg, gps as gps_msg
import random

def publisher():
	gps_pub = rospy.Publisher('gps', gps_msg, queue_size=10)
	imu_pub = rospy.Publisher('imu', imu_msg, queue_size=10)

	rospy.init_node('test_publisher', anonymous=True)
	r = rospy.Rate(10)

	gps_pub_msg = gps_msg(lat=42.4858533, long=-71.2230221)
	imu_pub_msg = imu_msg(x=0)

	while not rospy.is_shutdown():
		#rospy.loginfo(str)
		gps_pub.publish(gps_pub_msg)
		imu_pub.publish(imu_pub_msg)
		r.sleep()

		gps_pub_msg.lat += 0.000001*(random.random()-0.5)
		gps_pub_msg.long += 0.000001*(random.random()-0.5)
		imu_pub_msg.x += (random.random()-0.5)*5.
		imu_pub_msg.x %= 360
		#str = '{\"m0\":{\"v\":0.0}}\n'


if __name__ == '__main__':
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
