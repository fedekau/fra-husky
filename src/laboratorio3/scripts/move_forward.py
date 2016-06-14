#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
import random
import math

class GoToPose():
    def __init__(self):

        self.goal_sent = False

	# Tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")

	self.move_base.wait_for_server()

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'odom'
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Start moving
        self.move_base.send_goal(goal)

	# Allow HuskyBot up to 60 seconds to complete task
	success = self.move_base.wait_for_result(rospy.Duration(75)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

if __name__ == '__main__':
    try:
        rospy.init_node('move_forward', anonymous=False)
        
	navigator = GoToPose()

	while not rospy.is_shutdown():	
		x = random.uniform(-10.0, 10.0)
		y = random.uniform(-10.0, 10.0)
		w = random.uniform(-5.0, 5.0)
        	pose = {'x': x, 'y' : y}
        	angle = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : w}
        	rospy.loginfo("Go to (x: %s, y: %s, w: %s) pose", pose['x'], pose['y'], angle)

        	success = navigator.goto(pose, angle)

        	if success:
           		rospy.loginfo("Hooray, reached the desired pose")
        	else:
            		rospy.loginfo("The base failed to reach the desired pose")

        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
