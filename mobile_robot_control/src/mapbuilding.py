#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid,Odometry
from geometry_msgs.msg import Point
import numpy as np
import cv2
import tf

class BuldingMap():
    def __init__(self):
        self.listener = tf.TransformListener()
        self.start= Point()
        self.goal=Point()
        self.map = OccupancyGrid()
        self.sub_odom = rospy.Subscriber("/odom",Odometry,self.callback_odom)
        self.sub_grid = rospy.Subscriber("/map",OccupancyGrid,self.callback_grid)
        self.odom = Point()
        self.O=Odometry()
        self.flag=True
        self.trans=[0,0,0]
        while self.flag:
            rospy.logwarn("Waiting for map")
        self.printmap()
    def callback_odom(self,msg):
        try:
            (self.trans,rot) = self.listener.lookupTransform('/map','/base_footprint', rospy.Time(0))
        except:
            rospy.logwarn("Error in TF")
        self.odom.x=int(((self.trans[0])/0.05))
        self.odom.y=int(((self.trans[1])/0.05))
        self.O.header=msg.header
    def callback_grid(self,msg):
        self.map=msg
        self.flag=False
    def printmap(self):
        height = int(self.map.info.height)
        width = int(self.map.info.width)
        blank_image = np.ones((height,width,3), np.uint8)
        for i in range(height):
            for j in range(width):
                if self.map.data[int(j+(self.map.info.width*i))]<50:
                    blank_image[i][j]=255
        r=2
        rospy.sleep(0.2)
        print(self.odom.x,self.odom.y)
        cv2.rectangle(blank_image,(self.odom.x-r,self.odom.y-r),(self.odom.x+r,self.odom.y+r),(0,0,255),3)
        cv2.imshow("image", blank_image)
        cv2.waitKey(0)
        pass

rospy.init_node("BuldingMap")

s = BuldingMap()

rospy.spin()
