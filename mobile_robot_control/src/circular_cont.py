#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist


rospy.init_node('circular_cont')
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(10)
twist = Twist()
while not rospy.is_shutdown():
    twist.linear.x = 1
    twist.angular.z = 1
    pub.publish(twist)
    rate.sleep()
