#!/usr/bin/env python3

from janitor import Janitor
from visualization_msgs.msg import Marker
from math import sin, cos, radians

from time import sleep

import rospy

class PersonFollowingNode(object):
	def __init__(self):
		rospy.init_node('person_following')
		self.neato = Janitor()
		self.neato.connect()
		self.neato.drive(0,0)

	def radar_boundary(self, inside = [], outside = [], i = 1, d = 3):
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

	def states(self):
		if self.radar_boundary(inside=['nw'], outside=['n', 'ne']):
			self.neato.drive(0.4,-1)
			self.neato.visualize('person', 'base_link', Marker.SPHERE, 
					(self.neato.radar['nw']*cos(radians(135)), self.neato.radar['nw']*sin(radians(135)), 0),
					(1,1,1))
		elif self.radar_boundary(inside=['n','nw'], outside=['ne']):
			self.neato.drive(0.4,-0.8)
		elif self.radar_boundary(inside=['ne'], outside=['n', 'nw']):
			self.neato.drive(0.4,1)
			self.neato.visualize('person', 'base_link', Marker.SPHERE, 
					(self.neato.radar['ne']*cos(radians(315)), self.neato.radar['ne']*sin(radians(315)), 0),
					(1,1,1))
		elif self.radar_boundary(inside=['n','ne'], outside=['nw']):
			self.neato.drive(0.4,0.8)
		elif self.radar_boundary(inside=['n']):
			self.neato.visualize('person', 'base_link', Marker.SPHERE, 
					(self.neato.radar['n'], 0, 0),
					(1,1,1))
			self.neato.drive(0.8,0)
		else:
			self.neato.drive(0,0)

	def run(self):
		while not rospy.is_shutdown():
			self.states()

	def stop(self):
		self.neato._stop()

if __name__ == '__main__':
	personFollowingNode = PersonFollowingNode()
	personFollowingNode.run()
	personFollowingNode.stop()