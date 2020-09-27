#!/usr/bin/env python3

from janitor import Janitor
from visualization_msgs.msg import Marker

from time import sleep

import rospy

class WallFollowingNode(object):
	def __init__(self):
		rospy.init_node('wall_following')
		self.neato = Janitor()
		self.neato.connect()
		self.neato.drive(0,0)

	def radar_boundary(self, inside = [], outside = [], d = 1.2):
		radar = self.neato.radar
		sat = True
		for direction in inside:
			sat = sat and radar[direction] < d
		for direction in outside:
			sat = sat and radar[direction] > d
		return sat

	def states(self):
		if self.radar_boundary(outside=['n','nw','ne']):
			self.neato.drive(0.8,-0.5)
		elif self.radar_boundary(inside=['n'], outside=['nw','ne']):
			self.neato.go_right()
		elif self.radar_boundary(inside=['ne'], outside=['n','nw']):
			self.neato.go_forward()
		elif self.radar_boundary(inside=['nw'], outside=['n','ne']):
			self.neato.drive(0.8,-0.5)
		elif self.radar_boundary(inside=['n','ne'], outside=['nw']):
			self.neato.go_right()
		elif self.radar_boundary(inside=['n','nw'], outside=['ne']):
			self.neato.go_right()
		elif self.radar_boundary(inside=['n','nw','ne']):
			self.neato.go_right()
		elif self.radar_boundary(inside=['nw','ne'], outside=['n']):
			self.neato.drive(0.8,-0.5)

		if self.neato.position is not None:
			self.neato.visualize('wall', 'base_link', Marker.CUBE, 
				(1, 1.25, 0),
				(5,0.3,5))

	def run(self):
		while not rospy.is_shutdown():
			self.states()

	def stop(self):
		self.neato._stop()

if __name__ == '__main__':
	wallFollowingNode = WallFollowingNode()
	wallFollowingNode.run()
	wallFollowingNode.stop()