import rospy
import actionlib
import random
import math

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

class Explorer():
	def __init__(self):

		self.goal_sent = False
		self.explore = True	

		# Tell the action client that we want to spin a thread by default
		self.move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
		rospy.loginfo("Wait for the action server to come up")

		self.move_base.wait_for_server()

	def goto(self, pos, quat):

    		#Send a goal
		self.goal_sent = True
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'odom'
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000), Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

		# Start moving
		self.move_base.send_goal(goal)

		# Allow HuskyBot up to 60 seconds to complete task
		success = self.move_base.wait_for_result(rospy.Duration(60))

		state = self.move_base.get_state()
		result = False

		if success and state == GoalStatus.SUCCEEDED:
			result = True
		else:
			self.move_base.cancel_goal()

		self.goal_sent = False
		return result

	def callback(self, data):
    		if data.data == "STOP":
			self.move_base.cancel_goal()
			rospy.loginfo("Exploration has stopped")
			self.explore = False
		if data.data == "START":
			self.explore = True

	def is_exploring(self):
    		return self.explore

	def navigate(self):
		while not rospy.is_shutdown():
			if self.is_exploring():
				x = random.uniform(-10.0, 10.0)
				y = random.uniform(-10.0, 10.0)
				w = random.uniform(-5.0, 5.0)
				pose = {'x': x, 'y': y}
                		angle = {'r1': 0.000,'r2': 0.000,'r3': 0.000,'r4': w}
				
                		rospy.loginfo("Go to (x: %s, y: %s, w: %s) pose", pose['x'], pose['y'], angle)

				success = self.goto(pose, angle)

				if success:
    					rospy.loginfo("Reached the desired goal")
				else:
					rospy.loginfo("Failed to reach goal")
			rospy.sleep(1)
		return true
