#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32,Empty
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
class wall_follower:
    def __init__(self):
        rospy.init_node('wall_follower_controller')
        self.pub=rospy.Publisher('cmd_vel',Twist,queue_size=1)
        self.heart_rate=rospy.Subscriber('/heart_rate',Empty,self.operate)
        self.heading_sub = rospy.Subscriber('/heading', Float32, self.heading_callback)
        self.left_sensor_sub = rospy.Subscriber('/ir_left', Range, self.left_sensor_callback)
        self.right_sensor_sub = rospy.Subscriber('/ir_right', Range, self.right_sensor_callback)
        self.front_sensor_sub = rospy.Subscriber('/ir_front', Range, self.front_sensor_callback)
        self.heading=0
        self.left_sensor=False
        self.right_sensor=False
        self.front_sensor=False
        self.forward_msg=Twist()
        self.turn_left_msg=Twist()
        self.turn_right_msg=Twist()
        self.stop_msg=Twist()
        self.forward_msg.linear.x=1
        self.turn_left_msg.angular.z=0.5
        self.turn_right_msg.angular.z=-0.5
        self.state="forward"

        #min values
        self.min_left=0.6
        self.min_right=0.6
        self.min_front=0.2

    def toright(self):
        return (not self.right_sensor and (self.left_sensor ^ self.front_sensor))or(self.left_sensor and self.front_sensor)
    def toleft(self):
        return (not self.left_sensor and self.front_sensor and self.right_sensor)
    def toforward(self):
        return (not self.front_sensor and self.right_sensor)
    def operate(self,msg):
        if(self.state=="forward"):
            self.forward_state()
        elif(self.state=="turn_left"):
            self.turn_left_state()
        elif(self.state=="turn_right"):
            self.turn_right_state()
    def heading_callback(self,msg):
        # if(msg.data<0):
        #     self.heading=msg.data+360
        # else:
        self.heading=msg.data


    def left_sensor_callback(self,msg):
        if(msg.range<self.min_left):
            self.left_sensor=True
        else:
            self.left_sensor=False



    def right_sensor_callback(self,msg):
        if(msg.range<self.min_right):
            self.right_sensor=True
        else:
            self.right_sensor=False


    def front_sensor_callback(self,msg):
        if(msg.range<self.min_front):
            self.front_sensor=True
        else:
            self.front_sensor=False



    def forward_state(self):
        rospy.logwarn("forward state")
        self.pub.publish(self.forward_msg)
        if(self.toright()):
            self.state="turn_right"
            rospy.logwarn("turning right state")
        elif(self.toleft()):
            self.state="turn_left"
            rospy.logwarn("turning left state")




    def turn_left_state(self):
        self.pub.publish(self.turn_left_msg)
        rospy.logwarn("turning left state")
        # current_imu=self.heading
        # offset=0
        # while(offset<=90):
        #     offset=self.heading-current_imu
        #     print(offset)
        # self.pub.publish(self.stop_msg)
        # rospy.logwarn("Stop !!!!!!!!")
        self.state="forward"
        # if(self.toforward()):
        #     self.state="forward"
        #     rospy.logwarn("forward state")
        # elif(self.toright()):
        #     self.state="turn_right"
        #     rospy.logwarn("turning right state")


    def turn_right_state(self):
        self.pub.publish(self.turn_right_msg)
        rospy.logwarn("turning right state")
        # current_imu=self.heading
        # offset=0
        # while(offset<=90):
        #     offset=current_imu-self.heading
        #     print(offset)
        # rospy.logwarn("Stop !!!!!!!!")
        # self.pub.publish(self.stop_msg)
        self.state="forward"
        # if(self.toforward()):
        #     self.state="forward"
        #     rospy.logwarn("forward state")
        # elif(self.toleft()):
        #     self.state="turn_left"
        #     rospy.logwarn("turning left state")

if __name__ == '__main__':

    dec_node = wall_follower()
    rospy.spin()
