#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import hypot
dist=5
init_pose=0
flag=False
T =Twist()
T.linear.x=1
E=Twist()
def callback(msg):
    global flag
    global dist
    global init_pose
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    if(not flag):
        init_pose=hypot(x,y)
        dist = init_pose +5
        rospy.loginfo("Init_Pose: {}".format(init_pose))
        pub.publish(T)
        flag=True
        return
    if(flag):
        if((dist-hypot(x,y))>0.1):
            rospy.loginfo("traveled dist: {}".format(dist-hypot(x,y)))
            pub.publish(T)
        else:
            rospy.loginfo("Goal is reached with error: {}".format(dist-hypot(x,y)))
            pub.publish(E)
            rospy.signal_shutdown("BYE")




rospy.init_node('straight_dist')
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
sub = rospy.Subscriber("/odom",Odometry,callback)


rospy.spin()
