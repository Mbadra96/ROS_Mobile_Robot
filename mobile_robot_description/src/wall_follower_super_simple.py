#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import Float32,Empty
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

forward_msg=Twist()
turn_left_msg=Twist()
turn_right_msg=Twist()
stop_msg=Twist()
forward_msg.linear.x=0.3
turn_left_msg.angular.z=0.3
turn_right_msg.angular.z=-0.3

#min values
min_left=0.7
min_right=0.7
min_front=0.4
# Create callbacks
def left_sensor1_callback(msg,sm):
    if(msg.range<(0.4)):
        sm.userdata.left_sensor1=True
    else:
        sm.userdata.left_sensor1=False



def right_sensor1_callback(msg,sm):
    if(msg.range<(0.4)):
        sm.userdata.right_sensor1=True
    else:
        sm.userdata.right_sensor1=False

def left_sensor2_callback(msg,sm):
    if(msg.range<min_left):
        sm.userdata.left_sensor2=True
    else:
        sm.userdata.left_sensor2=False



def right_sensor2_callback(msg,sm):
    if(msg.range<min_right):
        sm.userdata.right_sensor2=True
    else:
        sm.userdata.right_sensor2=False


def front_sensor_callback(msg,sm):
    if(msg.range<min_front):
        sm.userdata.front_sensor=True
    else:
        sm.userdata.front_sensor=False


class Forward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['forward','turn_right','turn_left','stop']
        ,input_keys=['front_sensor','right_sensor1','left_sensor1','right_sensor2','left_sensor2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Forward')
        pub.publish(forward_msg)
        if(userdata.front_sensor or (not userdata.right_sensor2)):
            return 'stop'
        elif(userdata.left_sensor1):
            return 'turn_right'
        elif(userdata.right_sensor1):
            return 'turn_left'
        else:
            return 'forward'




class Turn_Right(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['forward','turn_right','stop']
        ,input_keys=['front_sensor','right_sensor1','left_sensor1','right_sensor2','left_sensor2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Turn_Right')
        pub.publish(turn_right_msg)
        if(userdata.front_sensor or (not userdata.right_sensor2)):
            return 'stop'
        elif(userdata.left_sensor1):
            return 'turn_right'
        else:
            return 'forward'


class Turn_Left(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['forward','turn_left','stop']
        ,input_keys=['front_sensor','right_sensor1','left_sensor1','right_sensor2','left_sensor2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Turn_Left')
        pub.publish(turn_left_msg)
        if(userdata.front_sensor or (not userdata.right_sensor2)):
            return 'stop'
        elif(userdata.right_sensor1):
            return 'turn_left'
        else:
            return 'forward'

class Super_Right(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['forward','super_right']
        ,input_keys=['front_sensor','right_sensor1','left_sensor1','right_sensor2','left_sensor2'])

    def execute(self, userdata):
        pub.publish(turn_right_msg)
        #rospy.sleep(1)
        if(userdata.front_sensor or (not userdata.right_sensor2)):
            return 'super_right'
        else:
            return 'forward'


class Super_Left(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['forward','super_left']
        ,input_keys=['front_sensor','right_sensor1','left_sensor1','right_sensor2','left_sensor2'])

    def execute(self, userdata):
        pub.publish(turn_left_msg)
        #rospy.sleep(1)
        if(userdata.front_sensor):
            return 'super_left'
        else:
            return 'forward'

class Stop(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['super_right','super_left']
        ,input_keys=['front_sensor','right_sensor1','left_sensor1','right_sensor2','left_sensor2'])

    def execute(self, userdata):
        pub.publish(stop_msg)
        if(userdata.right_sensor2 and userdata.front_sensor):
            return 'super_left'
        elif((not userdata.right_sensor2) or(userdata.left_sensor2 and userdata.front_sensor)):
            return 'super_right'
        else:
            return 'super_right'

# def main():
rospy.init_node('wall_follower_smach')

# Create a SMACH state machine
sm = smach.StateMachine(outcomes=[''])
sm.userdata.front_sensor=False
sm.userdata.right_sensor1=False
sm.userdata.left_sensor1=False
sm.userdata.right_sensor2=False
sm.userdata.left_sensor2=False

# Create Subscribers
left_sensor1_sub = rospy.Subscriber('/ir_left1', Range,left_sensor1_callback,(sm))
right_sensor1_sub = rospy.Subscriber('/ir_right1', Range, right_sensor1_callback,(sm))
left_sensor2_sub = rospy.Subscriber('/ir_left2', Range,left_sensor2_callback,(sm))
right_sensor2_sub = rospy.Subscriber('/ir_right2', Range, right_sensor2_callback,(sm))
front_sensor_sub = rospy.Subscriber('/ir_front', Range, front_sensor_callback,(sm))
# Create Publisher
pub=rospy.Publisher('cmd_vel',Twist,queue_size=1)

# Open the container
with sm:
    # Add states to the container
    smach.StateMachine.add('Forward', Forward(),
                           transitions={'forward':'Forward','turn_right':'Turn_Right', 'turn_left':'Turn_Left',
                           'stop':'Stop'},
                            remapping={'front_sensor':'front_sensor'
                            ,'right_sensor1':'right_sensor1','left_sensor1':'left_sensor1'
                            ,'right_sensor2':'right_sensor2','left_sensor2':'left_sensor2'})
    smach.StateMachine.add('Turn_Right', Turn_Right(),
                           transitions={'forward':'Forward','turn_right':'Turn_Right',
                           'stop':'Stop'},
                           remapping={'front_sensor':'front_sensor'
                           ,'right_sensor1':'right_sensor1','left_sensor1':'left_sensor1'
                           ,'right_sensor2':'right_sensor2','left_sensor2':'left_sensor2'})
    smach.StateMachine.add('Turn_Left', Turn_Left(),
                           transitions={'forward':'Forward','turn_left':'Turn_Left',
                           'stop':'Stop'},
                           remapping={'front_sensor':'front_sensor'
                           ,'right_sensor1':'right_sensor1','left_sensor1':'left_sensor1'
                           ,'right_sensor2':'right_sensor2','left_sensor2':'left_sensor2'})
    smach.StateMachine.add('Super_Right', Super_Right(),
                           transitions={'forward':'Forward','super_right':'Super_Right'},
                           remapping={'front_sensor':'front_sensor'
                           ,'right_sensor1':'right_sensor1','left_sensor1':'left_sensor1'
                           ,'right_sensor2':'right_sensor2','left_sensor2':'left_sensor2'})
    smach.StateMachine.add('Super_Left', Super_Left(),
                           transitions={'forward':'Forward','super_left':'Super_Left'},
                           remapping={'front_sensor':'front_sensor'
                           ,'right_sensor1':'right_sensor1','left_sensor1':'left_sensor1'
                           ,'right_sensor2':'right_sensor2','left_sensor2':'left_sensor2'})
    smach.StateMachine.add('Stop', Stop(),
                           transitions={'super_right':'Super_Right','super_left':'Super_Left'},
                           remapping={'front_sensor':'front_sensor'
                           ,'right_sensor1':'right_sensor1','left_sensor1':'left_sensor1'
                           ,'right_sensor2':'right_sensor2','left_sensor2':'left_sensor2'})


sis = smach_ros.IntrospectionServer('wall_follower', sm, '/SM_ROOT')
sis.start()

# Execute SMACH plan
outcome = sm.execute()
rospy.spin()
sis.stop()
