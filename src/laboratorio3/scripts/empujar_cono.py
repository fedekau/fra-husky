#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from llevar_cono import LlevarCono 

if __name__ == '__main__':
	try:
		rospy.init_node('llevar_cono', anonymous = False)
		
		llevarCono = LlevarCono()

		rospy.Subscriber("/imu/data", Imu, llevarCono.stop_at_collision)

		rospy.Subscriber("/laboratorio3/empujar_cono", String, llevarCono.empujar_cono)
		
		rospy.spin()

	except rospy.ROSInterruptException:
    		rospy.loginfo("Ctrl-C caught. Quitting")
