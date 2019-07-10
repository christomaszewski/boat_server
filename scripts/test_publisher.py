#!/usr/bin/env python3.7
import rospy
from std_msgs.msg import String

def publisher():
	pub  = rospy.Publisher('serial_out', String, queue_size=10)

	rospy.init_node('test_publisher', anonymous=True)
	r = rospy.Rate(10)
	
	str = '{\"e\":{\"cmd\":\"arm\"}}\n'
	while not rospy.is_shutdown():
		rospy.loginfo(str)
		pub.publish(str)
		r.sleep()
		str = '{\"m0\":{\"v\":0.0}}\n'


if __name__ == '__main__':
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
