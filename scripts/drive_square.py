#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import Twist, Vector3

class DriveSquareNode(object):
    def __init__(self):
        rospy.init_node('drive_square')
	self.desired_velocity = 0.3
	self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def run(self):
	r = rospy.Rate(10)
	while not rospy.is_shutdown():
	    seconds = int(rospy.get_time())
	    i = 0
	    if seconds%3 == 0:
	        self.pub.publish(Twist(linear=Vector3(x=self.desired_velocity,y=i
		i+=1
		i=i%4
