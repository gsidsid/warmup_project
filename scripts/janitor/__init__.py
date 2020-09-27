from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Vector3, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

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
        self.visualizer = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.position = None
        rospy.Subscriber('/scan', LaserScan, self._process_scan)
        rospy.Subscriber('/odom', Odometry, self._convert_pose_to_xy_and_theta)
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

    def _convert_pose_to_xy_and_theta(self, msg):
        pose = msg.pose.pose
        orientation_tuple = (pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
        angles = euler_from_quaternion(orientation_tuple)
        self.position = (pose.position.x, pose.position.y, angles[2])

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

    def visualize(self, name, frame, marker_type, position, scale):
        if position is not None:
            marker = Marker();
            marker.header.frame_id = frame;
            marker.header.stamp = rospy.Time.now();
            marker.ns = name;
            marker.id = 0;
            marker.type = marker_type;
            marker.action = Marker.ADD;
            marker.pose.position.x = position[0];
            marker.pose.position.y = position[1];
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = position[2];
            marker.pose.orientation.w = 1.0;
            marker.scale.x = scale[0];
            marker.scale.y = scale[1];
            marker.scale.z = scale[2];
            marker.color.a = 0.5; # Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            self.visualizer.publish(marker)