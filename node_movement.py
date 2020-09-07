#!/usr/bin/env python
import rospy
from std_msgs.msg import String,Int32MultiArray,Int16MultiArray
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


no_ball_detected_counter = 0  # Number of frames since the ball was last detected
x_robot = 0
y_robot = 0
yaw_robot = 0
target_yaw = 0

x_ball_old = 0
y_ball_old = 0

close_ball_counter = 0  # If close_ball_counter > 0, then the ball_is_close was true less than 3 frames ago.

info_obstacle = [0,0,0]

def ball_callback(data):

    global no_ball_detected_counter, x_robot, x_ball_old, y_ball_old, close_ball_counter,info_obstacle

    ball_is_close = False
    ball_position = 0  # Used only when ball_is_close is True

    x_ball,y_ball = data.data

    # print(x_ball)
    # print(y_ball)

    # # Constants
    # BALL_CENTERED = 0
    # BALL_IN_LEFT_CORNER = 1
    # BALL_IN_RIGHT_CORNER = 2
    # WINDOW_WIDTH = 640
    # MARGIN = 50

    if(x_ball == 0 and y_ball == 0):
        
        # No ball detection
        ball_is_close = False
        no_ball_detected_counter += 1



    else:

        no_ball_detected_counter = 0

        x_ball_old = x_ball
        y_ball_old = y_ball
        
        if y_ball > 300:
            ball_is_close = True
            close_ball_counter = 3
        else:
            ball_is_close = False
            close_ball_counter = 0

    # Override the ball_is_close boolean if it was detected as close less than 3 frames ago
    if close_ball_counter > 0 and not ball_is_close:
        ball_is_close = True
        x_ball = x_ball_old
        y_ball = y_ball_old
        close_ball_counter -= 1

    if ball_is_close:  # Movement close to the ball

        # if x_ball < MARGIN:
        #     ball_position = BALL_IN_LEFT_CORNER
        # elif x_ball > WINDOW_WIDTH - MARGIN:
        #     ball_position = BALL_IN_RIGHT_CORNER
        # else:
        #     ball_position = BALL_CENTERED

        # if ball_position == BALL_IN_LEFT_CORNER:
        #     print("palla in angolo sinistra - repositioning")
        #     twist = perform_movement(-0.2, -0.5)

        # elif ball_position == BALL_IN_RIGHT_CORNER:
        #     print("palla in angolo desstra - repositioning")
        #     twist = perform_movement(-0.2, 0.5)

        # else:
        yaw_diff = abs(yaw_robot-target_yaw)
        if info_obstacle[1] == 0:
            
            if yaw_robot > target_yaw and yaw_diff > 0.1:
                print("palla vicina - gira a destra")
                twist = perform_movement(0.1,-0.5)
            elif  yaw_robot < target_yaw and yaw_diff > 0.1:
                print("palla vicina - gira a sinistra")
                twist = perform_movement(0.1,0.5)
            else:
                print("palla vicina - vai a avanti")
                twist = perform_movement(0.1,0)
        else:
            if info_obstacle[0] != 0:
                # ho l'ostacolo al centro e a sinistra ([x,x,0]) quindi giro a destra
                print("ostacolo a sinistra - gira a destra")
                twist = perform_movement(0.1,-0.5)

            elif info_obstacle[2] != 0: 
                # ho l'ostacolo al centro e a destra ([0,x,x]) quindi giro a sinistra
                print("ostacolo a destra - gira a sinistra")
                twist = perform_movement(0.1,0.5)
            else:
                # ho l'ostacolo solo a centro o in tutte e tre
                if yaw_robot > target_yaw and yaw_diff > 0.1:
                    print("ostacolo al centro - gira a destra")
                    twist = perform_movement(0.1,-0.5)
                elif  yaw_robot < target_yaw and yaw_diff > 0.1:
                    print("ostacolo al centro - gira a sinistra")
                    twist = perform_movement(0.1,0.5)
                else:
                    # default gira a sinistra se robot e gia allineato e ha l'ostacolo al centro
                    print("ostacolo al centro - gira a sinistra default")
                    twist = perform_movement(0.1,0.5)
       

    elif no_ball_detected_counter == 0:  # Movement when the ball is far away, but detected

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

    else:

        if (no_ball_detected_counter > 30):
            # devo cercare la palla
            yaw_diff = abs(yaw_robot-target_yaw)
            if (info_obstacle[0] == 0 and info_obstacle[1] == 0 and info_obstacle[2] == 0):
                # allineati alla porta
                if yaw_robot > target_yaw and yaw_diff > 0.1:
                    print("palla vicina - gira a destra")
                    twist = perform_movement(0.1,-0.5)
                elif  yaw_robot < target_yaw and yaw_diff > 0.1:
                    print("palla vicina - gira a sinistra")
                    twist = perform_movement(0.1,0.5)
                else:
                    pass
                    #dovremmo farla girare di 180 deg. o 360 deg.
                
            

            

        
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
                twist = perform_movement(0.5,0)
                print("vado avanti")
     
    pub.publish(twist)  

def obstacle_callback(data):
    global info_obstacle
    info_obstacle=data.data
    info_obstacle = [x/4000000 for x in info_obstacle]
    print(info_obstacle)

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

    rospy.Subscriber("/Ball_Info", Int16MultiArray, ball_callback)

    rospy.Subscriber("/Obstacle_Info", Int32MultiArray, obstacle_callback)

    rospy.Subscriber("/robot1/odom", Odometry, odometry_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=1)
    listener()