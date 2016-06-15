#!/usr/bin/env python

import rospy
import sensor_msgs.msg


def callback(data):
    bla = ""
    for i in range(0, len(data.ranges)):
	bla += " " + str(int(data.ranges[i]))

    rospy.loginfo(bla)

def listener():

    rospy.init_node('laser_scan', anonymous=True)

    rospy.Subscriber("/scan", sensor_msgs.msg.LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
