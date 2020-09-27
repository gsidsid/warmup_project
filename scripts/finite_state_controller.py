#!/usr/bin/env python3

from janitor import Janitor
from time import sleep

import rospy

class FSCNode(object):
	def __init__(self):
		rospy.init_node('finite_state_controller')
		self.neato = Janitor()
		self.neato.connect()
		self.neato.drive(0,0)
		self.wall_following_state = False

	def person_radar_boundary(self, inside = [], outside = [], i = 0.5, d = 1.2):
		radar = self.neato.radar
		sat = True
		for direction in inside:
			sat = sat and radar[direction] < d
			if radar[direction] < i:
				return False
		for direction in outside:
			sat = sat and radar[direction] > d
			if radar[direction] < i:
				return False
		return sat

	def wall_radar_boundary(self, inside = [], outside = [], d = 2):
		radar = self.neato.radar
		sat = True
		for direction in inside:
			sat = sat and radar[direction] < d
			if radar[direction] < 1.2:
				return False
		for direction in outside:
			sat = sat and radar[direction] > d
			if radar[direction] < 1.2:
				return False
		return sat

	def person_following_states(self):
		if self.person_radar_boundary(inside=['nw'], outside=['n', 'ne']):
			self.neato.drive(0.4,-1)
		elif self.person_radar_boundary(inside=['n','nw'], outside=['ne']):
			self.neato.drive(0.4,-0.8)
		elif self.person_radar_boundary(inside=['ne'], outside=['n', 'nw']):
			self.neato.drive(0.4,1)
		elif self.person_radar_boundary(inside=['n','ne'], outside=['nw']):
			self.neato.drive(0.4,0.8)
		elif self.person_radar_boundary(inside=['n']):
			self.neato.drive(0.8,0)
		else:
			self.neato.drive(0,0)
			self.wall_following_state = True

	def wall_following_states(self):
		if self.wall_radar_boundary(outside=['n','nw','ne']):
			self.neato.drive(0.8,-0.5)
		elif self.wall_radar_boundary(inside=['n'], outside=['nw','ne']):
			self.neato.go_right()
		elif self.wall_radar_boundary(inside=['ne'], outside=['n','nw']):
			self.neato.go_forward()
		elif self.wall_radar_boundary(inside=['nw'], outside=['n','ne']):
			self.neato.drive(0.8,-0.5)
		elif self.wall_radar_boundary(inside=['n','ne'], outside=['nw']):
			self.neato.go_right()
		elif self.wall_radar_boundary(inside=['n','nw'], outside=['ne']):
			self.neato.go_right()
		elif self.wall_radar_boundary(inside=['n','nw','ne']):
			self.neato.go_right()
		elif self.wall_radar_boundary(inside=['nw','ne'], outside=['n']):
			self.neato.drive(0.8,-0.5)
		else:
			self.neato.drive(0,0)
			self.wall_following_state = False

	def run(self):
		while not rospy.is_shutdown():
			if self.wall_following_state:
				self.wall_following_states()
			else:
				self.person_following_states()

	def stop(self):
		self.neato._stop()

if __name__ == '__main__':
	fscNode = FSCNode()
	fscNode.run()
	fscNode.stop()