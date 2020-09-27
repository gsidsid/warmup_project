#!/usr/bin/env python3

from janitor import Janitor
from time import sleep

import rospy

class DriveSquareNode(object):
	def __init__(self):
		rospy.init_node('drive_square')
		self.neato = Janitor()
		self.neato.connect()
		self.neato.drive(0,0)

	def run(self):
		while not rospy.is_shutdown():
			self.neato.go_right()
			sleep(3.12)
			self.neato.go_forward()
			sleep(3)

	def stop(self):
		self.neato._stop()

if __name__ == '__main__':
	driveSquareNode = DriveSquareNode()
	driveSquareNode.run()
	driveSquareNode.stop()