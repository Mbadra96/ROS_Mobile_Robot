#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np



class imu2heading:
    def __init__(self):
        rospy.init_node('quaternion_to_euler')
        self.sub = rospy.Subscriber ('/imu', Imu, self.get_rotation)
        self.pub = rospy.Publisher('/heading',Float32,queue_size=1)
        self.counter=0
        self.last_yaw=10
        self.counter2=0
        self.last_yaw2=10

    def get_rotation (self,msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        yaw=np.rad2deg(yaw)
        # mapping from 180 -180 to 0 360
        if (self.last_yaw>0 and yaw<0):
            self.counter+=1
        elif(self.last_yaw<0 and yaw>0):
           self.counter-=1
        self.last_yaw=yaw
        yaw = yaw +(self.counter*360)
        # mapping from 0 360 to inf -inf
        if(self.last_yaw2>330 and yaw<20):
            self.counter2+=1
        elif(self.last_yaw2<20 and yaw>330):
            self.counter2-=1
        self.last_yaw2=yaw

        yaw= yaw +(self.counter2*360)
        self.pub.publish(yaw)

if __name__ == '__main__':

    dec_node = imu2heading()
    rospy.spin()
