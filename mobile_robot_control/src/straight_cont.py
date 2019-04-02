#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist


rospy.init_node('straightcont')
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(2)

twist = Twist()
while not rospy.is_shutdown():
    twist.linear.x = 1;

    pub.publish(twist)
    rate.sleep()
