#!/usr/bin/env python3

from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

import rospy
import roslib

import sys, select, termios, tty
import threading

speeds = {
    'slow': 0.5,
    'default': 1,
    'fast': 1.5
}

class Janitor(threading.Thread):
    def __init__(self, rate=0):
        super(Janitor, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        rospy.Subscriber('/scan', LaserScan, self._process_scan)
        self.go = 0.0
        self.rotate = 0.0
        self.speed = 0.0
        self.condition = threading.Condition()
        self.done = False
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None
        self.start()

    def connect(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for neato to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before neato connected")

    def _process_scan(self, msg):
    	self.radar = {
    		'n': min(min(msg.ranges[0:46]),10),
    		'nw': min(min(msg.ranges[46:91]),10),
    		'w': min(min(msg.ranges[91:136]),10),
    		'sw': min(min(msg.ranges[136:181]),10),
    		's': min(min(msg.ranges[181:226]),10),
    		'se': min(min(msg.ranges[226:271]),10),
    		'e': min(min(msg.ranges[271:316]),10),
    		'ne': min(min(msg.ranges[316:361]),10)
    	}
    	self.scan = msg

    def _update(self, go, rotate, speed):
        self.condition.acquire()
        self.go = go
        self.rotate = rotate
        self.speed = speed
        self.condition.notify()
        self.condition.release()

    def _stop(self):
        self.done = True
        self._update(0, 0, 0)

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            self.condition.wait(self.timeout)
            twist.linear.x = self.go * self.speed
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.rotate * self.speed
            self.condition.release()
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)

    def go_left(self, speed=speeds['slow']):
        self._update(0,-1,speed)

    def go_right(self, speed=speeds['slow']):
        self._update(0,1,speed)

    def go_forward(self, speed=speeds['slow']):
        self._update(1,0,speed)

    def drive(self, _go, _rotate, _speed=speeds['slow']):
        self._update(_go, _rotate, _speed)

    def teleop(self, command):
        if command is 'w':
            self.go_forward()
        elif command is 'a':
            self.go_left()
        elif command is 'd':
            self.go_right()
        else:
            self.drive(0,0)