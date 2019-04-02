#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import *



x=0
y=0
theta=0

def newOdom(msg):
	global x
	x = msg.pose.pose.position.x
	global y
	y = msg.pose.pose.position.y
	rot = msg.pose.pose.orientation
	global theta
	(roll,pitch,theta) = euler_from_quaternion([rot.x,rot.y,rot.z,rot.w])


rospy.init_node('gotogoalcontroller')
sub = rospy.Subscriber('/odom', Odometry, newOdom)
#pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

xG = 5
yG = 5
thetaG = 90
Krho = 0.3
Kalpha = 2
Kbeta = 0
r = rospy.Rate(10)
done = 1


vel = Twist()

while (not rospy.is_shutdown()) and done==1:
	rho = sqrt((xG-x)**2 + (yG-y)**2)
	alpha = (atan2((yG-y),(xG-x))) - theta
	if abs(alpha) < 90:
		vel.linear.x = Krho * rho
		vel.angular.z = Kalpha*alpha
	else:
		vel.linear.x = -0.1*Krho * rho
		vel.angular.z = 0.5*Kalpha*alpha
	pub.publish(vel)
	if(abs(xG-x) < 0.1 and abs(yG-y) < 0.1):
		done=0
		vel.linear.x = 0
		vel.angular.z = 0
		pub.publish(vel)
	r.sleep()
