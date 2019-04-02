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
min_left=1.5
min_right=1.5
min_front=0.5
# Create callbacks
def heading_callback(msg,sm):
    sm.userdata.heading=msg.data


def left_sensor1_callback(msg,sm):
    if(msg.range<(min_left*1.4)):
        sm.userdata.left_sensor1=True
    else:
        sm.userdata.left_sensor1=False



def right_sensor1_callback(msg,sm):
    if(msg.range<(min_right*1.4)):
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




def toforward(A,B,C,D,E):
    #A,B,C,D,E=LS2,LS1,FS,RS1,RS2
    return ((not C)and(D)and(E))or((A)and(not C)and(D))or((not A)and(not B)and(not C)and(not D)and(not E))
def toright(A,B,C,D,E):
    #A,B,C,D,E=LS2,LS1,FS,RS1,RS2
    return ((not D)and(E))or((C)and(not E))or((B)and(not D))or((A)and(not D))or(A and B and C)
def toleft(A,B,C,D,E):
    #A,B,C,D,E=LS2,LS1,FS,RS1,RS2
    return ((not A)and(not B)and(not C)and(D))or((not A)and(not C)and(D)and(not E))or((not A)and( C)and(D)and(E))or((not B)and( C)and(D)and(E))

class Forward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['forward','turn_right','turn_left']
        ,input_keys=['heading','front_sensor','right_sensor1','left_sensor1','right_sensor2','left_sensor2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Forward')
        pub.publish(forward_msg)

        if toright(userdata.left_sensor2,userdata.left_sensor1,userdata.front_sensor,userdata.right_sensor1,userdata.right_sensor2):
            pub.publish(stop_msg)
            return 'turn_right'
        elif toleft(userdata.left_sensor2,userdata.left_sensor1,userdata.front_sensor,userdata.right_sensor1,userdata.right_sensor2):
            pub.publish(stop_msg)
            return 'turn_left'
        elif toforward(userdata.left_sensor2,userdata.left_sensor1,userdata.front_sensor,userdata.right_sensor1,userdata.right_sensor2):
            return 'forward'
        else:
            return 'forward'



class Turn_Right(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['forward','turn_right']
        ,input_keys=['heading','front_sensor','right_sensor1','left_sensor1','right_sensor2','left_sensor2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Turn_Right')
        pub.publish(turn_right_msg)
        if toright(userdata.left_sensor2,userdata.left_sensor1,userdata.front_sensor,userdata.right_sensor1,userdata.right_sensor2):
            return 'turn_right'
        elif toforward(userdata.left_sensor2,userdata.left_sensor1,userdata.front_sensor,userdata.right_sensor1,userdata.right_sensor2):
            pub.publish(stop_msg)
            return 'forward'
        else:
            return 'forward'

class Turn_Left(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['forward','turn_left']
        ,input_keys=['heading','front_sensor','right_sensor1','left_sensor1','right_sensor2','left_sensor2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Turn_Left')
        pub.publish(turn_left_msg)
        if toleft(userdata.left_sensor2,userdata.left_sensor1,userdata.front_sensor,userdata.right_sensor1,userdata.right_sensor2):
            return 'turn_left'
        elif toforward(userdata.left_sensor2,userdata.left_sensor1,userdata.front_sensor,userdata.right_sensor1,userdata.right_sensor2):
            pub.publish(stop_msg)
            return 'forward'
        else:
            return 'forward'




# def main():
rospy.init_node('wall_follower_smach')

# Create a SMACH state machine
sm = smach.StateMachine(outcomes=[''])
sm.userdata.heading=0
sm.userdata.front_sensor=False
sm.userdata.right_sensor1=False
sm.userdata.left_sensor1=False
sm.userdata.right_sensor2=False
sm.userdata.left_sensor2=False
sm.userdata.heading=0

# Create Subscribers
left_sensor1_sub = rospy.Subscriber('/ir_left1', Range,left_sensor1_callback,(sm))
right_sensor1_sub = rospy.Subscriber('/ir_right1', Range, right_sensor1_callback,(sm))
left_sensor2_sub = rospy.Subscriber('/ir_left2', Range,left_sensor2_callback,(sm))
right_sensor2_sub = rospy.Subscriber('/ir_right2', Range, right_sensor2_callback,(sm))
front_sensor_sub = rospy.Subscriber('/ir_front', Range, front_sensor_callback,(sm))
heading_sub = rospy.Subscriber('/heading', Float32, heading_callback,(sm))
# Create Publisher
pub=rospy.Publisher('cmd_vel',Twist,queue_size=1)

# Open the container
with sm:
    # Add states to the container
    smach.StateMachine.add('Forward', Forward(),
                           transitions={'forward':'Forward','turn_right':'Turn_Right', 'turn_left':'Turn_Left'},
                            remapping={'front_sensor':'front_sensor'
                            ,'right_sensor1':'right_sensor1','left_sensor1':'left_sensor1'
                            ,'right_sensor2':'right_sensor2','left_sensor2':'left_sensor2'})
    smach.StateMachine.add('Turn_Right', Turn_Right(),
                           transitions={'forward':'Forward','turn_right':'Turn_Right'},
                           remapping={'front_sensor':'front_sensor'
                           ,'right_sensor1':'right_sensor1','left_sensor1':'left_sensor1'
                           ,'right_sensor2':'right_sensor2','left_sensor2':'left_sensor2'})
    smach.StateMachine.add('Turn_Left', Turn_Left(),
                           transitions={'forward':'Forward','turn_left':'Turn_Left'},
                           remapping={'front_sensor':'front_sensor'
                           ,'right_sensor1':'right_sensor1','left_sensor1':'left_sensor1'
                           ,'right_sensor2':'right_sensor2','left_sensor2':'left_sensor2'})

sis = smach_ros.IntrospectionServer('wall_follower', sm, '/SM_ROOT')
sis.start()

# Execute SMACH plan
outcome = sm.execute()
rospy.spin()
sis.stop()



# if __name__ == '__main__':
#     main()
