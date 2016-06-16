#!/usr/bin/env python
import rospy
from explorer import Explorer
from std_msgs.msg import String

def talker():
	pub = rospy.Publisher('/cmd_vel', Twist , queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	
	while not rospy.is_shutdown():
		twist = Twist(Vector3(0,0,0),Vector3(0,0,0))
		pub.publish(twist)
		rate.sleep()


if __name__ == '__main__':
	try:
		rospy.init_node('move_forward', anonymous = False)

		randomExplorer = Explorer()

		rospy.Subscriber("/laboratorio3/exploration", String, randomExplorer.callback)

		randomExplorer.navigate()

		rospy.spin()

	except rospy.ROSInterruptException:
    		rospy.loginfo("Ctrl-C caught. Quitting")
