#!/usr/bin/env python3.7
import rospy
from boat_server import msg

if __name__ == '__main__':
	pub = rospy.Publisher('autonomy_cmd', msg.autonomy_cmd, queue_size=10)
	rospy.init_node('autonomy_tester', anonymous=True)
	pub.publish(msg.autonomy_cmd(True))