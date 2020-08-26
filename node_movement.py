#!/usr/bin/env python
import rospy
from std_msgs.msg import String,Int16MultiArray
from geometry_msgs.msg import Twist

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

ANG_VEL_STEP_SIZE = 0.1

control_angular_vel = 0

def callback(data):

    global control_angular_vel

    x_ball,y_ball = data.data

    # print(x_ball)
    # print(y_ball)

    twist = Twist()

    if(x_ball == 0 and y_ball == 0):
        # No ball detection
        
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0

        target_angular_vel = 2

        control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
        
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

        pub.publish(twist)
    else:
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

        pub.publish(twist)
    


    


def listener():


    rospy.init_node('node_movement')

    rospy.Subscriber("/Ball_Info", Int16MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=1)
    listener()