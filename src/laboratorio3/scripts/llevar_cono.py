import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class LlevarCono():
	def __init__(self):
		self.hay_que_empujar = False

	def stop_at_collision(self, data):
		if(data.linear_acceleration.x > 5.0):
			self.hay_que_empujar = False
		
	def empujar_cono(self, data):
		if (data.data == "EMPUJAR"):
			self.hay_que_empujar = True

		pub = rospy.Publisher('/cmd_vel', Twist , queue_size=10)
		rate = rospy.Rate(10) # 10hz
		empujar = Twist(Vector3(10,0,0),Vector3(0,0,0))

		while (not rospy.is_shutdown()) and self.hay_que_empujar:
			pub.publish(empujar)
			rate.sleep()

		rospy.loginfo("PARE DE EMPUJAR")	
		retroceder = Twist(Vector3(-10,0,0),Vector3(0,0,0))
		i = 0
		while (not rospy.is_shutdown()) and i < 100:
			pub.publish(retroceder)
			rate.sleep()
			i +=1
