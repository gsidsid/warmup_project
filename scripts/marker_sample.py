#!/usr/bin/env python3

from visualization_msgs.msg import Marker
from time import sleep

import rospy

rospy.init_node('test_vis')
vis_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

marker = Marker();
marker.header.frame_id = "odom";
marker.header.stamp = rospy.Time.now();
marker.ns = "warmup_project";
marker.id = 0;
marker.type = Marker.SPHERE;
marker.action = Marker.ADD;
marker.pose.position.x = 1;
marker.pose.position.y = 2;
marker.pose.position.z = 0;
marker.pose.orientation.x = 0.0;
marker.pose.orientation.y = 0.0;
marker.pose.orientation.z = 0.0;
marker.pose.orientation.w = 1.0;
marker.scale.x = 1;
marker.scale.y = 1;
marker.scale.z = 1;
marker.color.a = 0.5; # Don't forget to set the alpha!
marker.color.r = 0.0;
marker.color.g = 1.0;
marker.color.b = 0.0;
# only if using a MESH_RESOURCE marker type:
# marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

while True:
    vis_pub.publish( marker )
    sleep(0.1)
