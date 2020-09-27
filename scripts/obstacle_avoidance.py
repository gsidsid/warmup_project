#!/usr/bin/env python3

from janitor import Janitor
from time import sleep

import rospy

class ObstacleAvoidanceNode(object):
	def __init__(self):
		rospy.init_node('obstacle_avoidance')
		self.neato = Janitor()
		self.neato.connect()
		self.neato.drive(0,0)

	def states(self, theta):
		trim = 1.3*(theta-self.neato.position[2])
		trim -= 2/self.neato.radar['ne']
		trim += 2/self.neato.radar['nw']
		go = min(self.neato.radar['n']/2,0.7)
		self.neato.drive(go, trim)

	def run(self):
		if self.neato.position is not None:
			theta = self.neato.position[2]
			while not rospy.is_shutdown():
				self.states(theta)

	def stop(self):
		self.neato._stop()

if __name__ == '__main__':
	obstacleAvoidanceNode = ObstacleAvoidanceNode()
	obstacleAvoidanceNode.run()
	obstacleAvoidanceNode.stop() 