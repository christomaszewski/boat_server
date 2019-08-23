#!/usr/bin/env python3.7
import rospy
from boat_server.msg import motor_cmd as motor_cmd_msg, eboard_cmd as eboard_cmd_msg
import random
import numpy as np

def publisher():
	motor_cmd_pub = rospy.Publisher('motor_cmd', motor_cmd_msg, queue_size=10)
	eboard_cmd_pub = rospy.Publisher('eboard_cmd', eboard_cmd_msg, queue_size=10)

	rospy.init_node('motor_test', anonymous=True)
	r = rospy.Rate(2)

	eboard_msg = eboard_cmd_msg(cmd='arm')
	eboard_cmd_pub.publish(eboard_msg)

	r.sleep()

	motor_msg = motor_cmd_msg(m0=0.,m1=0.)
	motor_cmd_pub.publish(motor_msg)

	r.sleep()

	for vel in np.linspace(0.0, 0.1, 30):
		motor_cmd_pub.publish(motor_msg)
		r.sleep()


	eboard_msg = eboard_cmd_msg(cmd='disarm')
	eboard_cmd_pub.publish(eboard_msg)

if __name__ == '__main__':
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass