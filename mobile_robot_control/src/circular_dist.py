#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist


rospy.init_node('circular_dist')
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
twist = Twist()

def get_countes_from_angle(x):
    return x/30


counter = 0
while counter < get_countes_from_angle(180):
    print(counter)
    twist.linear.x = 1
    twist.angular.z = 1
    pub.publish(twist)
    rospy.sleep(1)
    counter +=1
twist.linear.x = 0
twist.angular.z = 0
pub.publish(twist)
