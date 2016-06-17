#!/usr/bin/env python
import rospy
from explorer import Explorer
from std_msgs.msg import String

if __name__ == '__main__':
	try:
		rospy.init_node('move_forward', anonymous = False)

		randomExplorer = Explorer()

		rospy.Subscriber("/laboratorio3/exploration", String, randomExplorer.callback)

		randomExplorer.navigate()

		rospy.spin()

	except rospy.ROSInterruptException:
    		rospy.loginfo("Ctrl-C caught. Quitting")
