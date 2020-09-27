#!/usr/bin/env python3

from janitor import Janitor
from time import sleep

import rospy

class ObstacleAvoidanceNode(object):
	def __init__(self):
		rospy.init_node('drive_square')
		self.neato = Janitor()
		self.neato.connect()
		self.neato.drive(0,0)

	def radar_boundary(self, inside = [], outside = [], d = 3):
		radar = self.neato.radar
		sat = True
		for direction in inside:
			sat = sat and radar[direction] < d
		for direction in outside:
			sat = sat and radar[direction] > d
		return sat

	def states(self):
		if self.radar_boundary(inside=['nw'], outside=['n', 'ne']):
			self.neato.drive(0.4,-1)
		elif self.radar_boundary(inside=['n','nw'], outside=['ne']):
			self.neato.drive(0.4,-0.8)
		elif self.radar_boundary(inside=['ne'], outside=['n', 'nw']):
			self.neato.drive(0.4,1)
		elif self.radar_boundary(inside=['n','ne'], outside=['nw']):
			self.neato.drive(0.4,0.8)
		elif self.radar_boundary(inside=['n']):
			self.neato.drive(0.8,0)

	def run(self):
		while not rospy.is_shutdown():
			self.states()

	def stop(self):
		self.neato._stop()

if __name__ == '__main__':
	obstacleAvoidanceNode = ObstacleAvoidanceNode()
	obstacleAvoidanceNode.run()
	obstacleAvoidanceNode.stop()