#!/usr/bin/env python
import rospy
from std_msgs.msg import String,Int16MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import numpy as np

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

ANG_VEL_STEP_SIZE = 0.1
LIN_VEL_STEP_SIZE = 0.01

control_angular_vel = 0
control_linear_vel = 0

def perform_movement(target_linear_vel = 0.0,target_angular_vel = 0.0):

    global control_angular_vel,control_linear_vel

    twist = Twist()

    control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))

    twist.linear.x = target_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

    control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
        
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

    return twist


fps_no_det = 0
x_ball_old = 0
y_ball_old = 0
x_robot = 0
y_robot = 0
yaw_robot = 0
target_yaw = 0
ball_is_close = False

def callback(data):

    global fps_no_det, x_ball_old, y_ball_old,ball_is_close,x_robot

    x_ball,y_ball = data.data

    # print(x_ball)
    # print(y_ball)

    if not ball_is_close:
        if(x_ball == 0 and y_ball == 0):
            # No ball detection
            fps_no_det += 1
            if (fps_no_det > 30):
                print("giro a cercare la palla")
                twist = perform_movement(0.0,2)
            else:
                print("uso x,y vecchi")
                if (x_ball_old < 280 or x_ball_old > 360):
                    if(x_ball_old > 360):
                        print("giro a destra")
                        twist = perform_movement(0.0,-1)
                    else:
                        print("giro a sinistra")
                        twist = perform_movement(0.0,1)
                else:
                    twist = perform_movement(0.1,0)
                    print("vado avanti")
                if y_ball_old > 320:
                    ball_is_close = True
                else:
                    ball_is_close = False

        else:
            fps_no_det = 0
            x_ball_old = x_ball
            y_ball_old = y_ball
            if (x_ball < 280 or x_ball > 360):
                if(x_ball > 360):
                    print("giro a destra")
                    twist = perform_movement(0.0,-1)
                else:
                    print("giro a sinistra")
                    twist = perform_movement(0.0,1)
            else:
                twist = perform_movement(0.1,0)
                print("vado avanti")
            if y_ball > 320:
                ball_is_close = True
            else:
                ball_is_close = False
    else:
        yaw_diff = abs(yaw_robot-target_yaw)
        if yaw_robot > target_yaw and yaw_diff > 0.1:
            print("palla vicina - gira a destra")
            twist = perform_movement(0.1,-1)
        elif  yaw_robot < target_yaw and yaw_diff > 0.1:
            print("palla vicina - gira a sinistra")
            twist = perform_movement(0.1,1)
        else:
            print("palla vicina - vai a avanti")
            twist = perform_movement(0.1,0)
     
    pub.publish(twist)  
    
def odometry_callback(msg):
    global x_robot,y_robot,yaw_robot,target_yaw

    x_robot = msg.pose.pose.position.x
    y_robot = msg.pose.pose.position.y

    orientation_values = msg.pose.pose.orientation
    orientation_list = [orientation_values.x, orientation_values.y, orientation_values.z, orientation_values.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    # yaw_robot in radians
    # Positive values until 180 degrees (pi rad) in the trigonometric direction (anticlockwise)
    # Negative values until 180 degrees (-pi rad) in the anti-trigonometric direction (clockwise)
    # ! Here, pi rad = -pi rad
    yaw_robot = yaw

    # The cuurdinates 0, 0 are at the center of the field
    x_goal = 6.0
    y_goal = 0.0

    x_distance_to_goal = x_goal - x_robot  # Always the positive distance, given the geometry
    y_distance_to_goal = y_goal - y_robot  # Negative if the robot has to travel in the direction of -y,
                                           # positive if he robot has to travel in the direction of y
    euclidian_distance_to_goal = math.sqrt(x_distance_to_goal**2 + y_distance_to_goal**2)

    # The yaw to be achieved by the robot, follows the same convention as yaw_robot
    target_yaw = np.arcsin(y_distance_to_goal / euclidian_distance_to_goal)

    #print(target_yaw, yaw_robot)


    


def listener():


    rospy.init_node('node_movement')

    rospy.Subscriber("/Ball_Info", Int16MultiArray, callback)

    rospy.Subscriber("/robot1/odom", Odometry, odometry_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=1)
    listener()