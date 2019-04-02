#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist


rospy.init_node('straight_time')
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)


twist = Twist()
counter = 5

rate = rospy.Rate(1)
while not rospy.is_shutdown():
    if counter == 0 :
        twist.linear.x = 0

    else:
        twist.linear.x = 1
        counter = counter - 1
    pub.publish(twist)
    rate.sleep()

#rospy.spin()
