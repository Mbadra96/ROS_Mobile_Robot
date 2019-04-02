#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid,Path,Odometry
from geometry_msgs.msg import Point,PoseStamped
import numpy as np
import tf
class Asearch():
    def __init__(self):
        self.listener = tf.TransformListener()
        self.start= Point()
        self.goal=Point()
        self.map = OccupancyGrid()
        self.sub_odom = rospy.Subscriber("/odom",Odometry,self.callback_odom)
        self.sub_grid = rospy.Subscriber("/map",OccupancyGrid,self.callback_grid)
        self.pub = rospy.Publisher("/asearch_path",Path,queue_size=1)
        self.odom = Point()
        self.O=Odometry()
        self.Path = Path()
        self.flag=True
        self.visitedlist=[]
        self.Path=Path()
        self.trans=[0,0,0]
        while self.flag:
            rospy.logwarn("Waiting for map")
        self.startpath()
    def callback_odom(self,msg):
        try:
            (self.trans,rot) = self.listener.lookupTransform('/map','/odom', rospy.Time(0))
        except:
            rospy.logwarn("Error in TF")
        self.odom.x=int((msg.pose.pose.position.x+self.trans[0]/0.05))
        self.odom.y=int((msg.pose.pose.position.y+self.trans[1]/0.05))
        self.O.header=msg.header
    def callback_grid(self,msg):
        self.map=msg
        self.flag=False
    def startpath(self):
        self.start.x=self.odom.y
        self.start.y=self.odom.x
        self.goal.x=int(1/0.05)
        self.goal.y=int(0/0.05)
        dx = [0,1,1,1,0,-1,-1,-1]
        dy = [1,1,0,-1,-1,-1,0,1]
        openlist=[]
        closedlist=[]
        closedlist.append(self.start)
        while closedlist[len(closedlist)-1].x !=self.goal.x or closedlist[len(closedlist)-1].y !=self.goal.y:
            cord_x = closedlist[len(closedlist)-1].x
            cord_y = closedlist[len(closedlist)-1].y
            for i in range(8):
                if self.isvalid(cord_x+dx[i],cord_y+dy[i]):
                    if self.freepoint(cord_x+dx[i],cord_y+dy[i]):
                        if not self.visited(cord_x+dx[i],cord_y+dy[i]):
                            p=Point()
                            p.x=cord_x+dx[i]
                            p.y=cord_y+dy[i]
                            openlist.append(p)
                            self.visitedlist.append(p)
            sum = self.goal.x-openlist[0].x+self.goal.y-openlist[0].y
            index=0
            for i in range(len(openlist)):
                if (self.goal.x-openlist[i].x)+(self.goal.y-openlist[i].y) < sum:
                    sum = (self.goal.x-openlist[i].x)+(self.goal.y-openlist[i].y)
                    index=i
            closedlist.append(openlist.pop(i))
        rospy.loginfo("Adding to Path")
        for i in range(len(closedlist)):
            tmp = PoseStamped()
            tmp.pose.position.y=closedlist[i].x*0.05
            tmp.pose.position.x=closedlist[i].y*0.05
            tmp.header=self.O.header
            self.Path.poses.append(tmp)
        r = rospy.Rate(10)
        rospy.loginfo("Done")
        while(True):
            self.Path.header=self.O.header
            self.pub.publish(self.Path)
            r.sleep()



    def isvalid(self,x,y):
        if (x<0 or x>=self.map.info.width):
            return False
        if (y<0 or y>=self.map.info.height):
            return False
        return True

    def freepoint(self,x,y):
        try:
            if self.map.data[int(x+(self.map.info.width*y))] > 50:
                return False
        except:
            rospy.logwarn(int(x+self.map.info.width*y))
            return False
        return True
    def visited(self,x,y):
        for i in self.visitedlist:
            if (i.x==x and i.y==y):
                return True
        return False










if __name__ == "__main__":
    rospy.init_node("Asearch")
    s= Asearch()
    rospy.spin()
