#! /usr/bin/env python

import rospy
import actionlib
from mobile_robot_control.msg import goToGoalAction, goToGoalFeedback, goToGoalResult
from geometry_msgs.msg import Twist,Pose,Point
from nav_msgs.msg import Odometry
from math import atan2,pow,sqrt
from tf.transformations import euler_from_quaternion
class GoToGoalServer():
    def __init__(self):
        self._server= actionlib.SimpleActionServer("GoToGoalAction", goToGoalAction, execute_cb=self.execute_cb,
        auto_start=False)
        self._server.start()
        self.sub = rospy.Subscriber("/odom",Odometry,self.callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom =Point()
        self.theta=0
    def execute_cb(self,goal):
        success = True
        _feedback = goToGoalFeedback()
        _result = goToGoalResult()
        cmd_vel = Twist()
        c=0
        k=0
        while self.row(goal.goalpose.position) > 0.1:
            if self._server.is_preempt_requested():
                cmd_vel.linear.x=0
                cmd_vel.angular.z=0
                self.pub.publish(cmd_vel)
                success = False
                break
            _feedback.feedbackpose.position = self.odom
            alpha=self.alpha(goal.goalpose.position)
            if alpha<1.57 and alpha >-1.57:
                c = 0.3
                k=2
            else:
                c=-0.01
                k=1
            cmd_vel.linear.x=c*self.row(goal.goalpose.position)
            cmd_vel.angular.z = k*alpha
            self.pub.publish(cmd_vel)
            self._server.publish_feedback(_feedback)

        if success:
            cmd_vel.linear.x=0
            cmd_vel.angular.z=0
            self.pub.publish(cmd_vel)
            _result.resultpose.position = self.odom
            self._server.set_succeeded(_result)


    def callback(self,msg):
        self.odom.x=round(msg.pose.pose.position.x,4)
        self.odom.y=round(msg.pose.pose.position.y,4)
        rot_q = msg.pose.pose.orientation
        _, _, self.theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def row(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.odom.x), 2) +
                    pow((goal_pose.y - self.odom.y), 2))
    def alpha(self, goal_pose):
        deltax = goal_pose.x -self.odom.x
        deltay = goal_pose.y -self.odom.y
        return atan2(deltay, deltax)-self.theta




if __name__ == "__main__":
    rospy.init_node("GoToGoalServer")
    s= GoToGoalServer()
    rospy.spin()
