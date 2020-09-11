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

phase_counter = 0  # Used if the ball is not detected
is_in_phase_1 = True
save_yaw_robot = True

x_robot_saved = 0
yaw_robot_saved = 0

info_obstacle = [0,0,0]

go_forward_counter = 0

def ball_callback(data):

    global no_ball_detected_counter, x_robot, x_ball_old, y_ball_old, close_ball_counter, info_obstacle, phase_counter, is_in_phase_1, save_yaw_robot, x_robot_saved, yaw_robot_saved, go_forward_counter

    if x_robot_saved == 0:
        x_robot_saved = x_robot
    if yaw_robot_saved == 0:
        yaw_robot_saved = yaw_robot

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

    print("-----------------------------------")

    if ball_is_close:  # Movement close to the ball

        print("The ball is close")

        yaw_diff = abs(yaw_robot-target_yaw)
        if info_obstacle[1] == 0:

            print("No obstacle forwards")
            
            if yaw_robot > target_yaw and yaw_diff > 0.1:
                print("Turnung right with the ball")
                twist = perform_movement(0.1,-0.5)
            elif  yaw_robot < target_yaw and yaw_diff > 0.1:
                print("Turning left with the ball")
                twist = perform_movement(0.1,0.5)
            else:
                print("Going forward with the ball")
                twist = perform_movement(0.1,0)
        else:
            if info_obstacle[0] != 0:
                # ho l'ostacolo al centro e a sinistra ([x,x,0]) quindi giro a destra
                print("Obstacle forward, but not on the right")
                print("Turning right with the ball")
                twist = perform_movement(0.1,-0.5)

            elif info_obstacle[2] != 0: 
                # ho l'ostacolo al centro e a destra ([0,x,x]) quindi giro a sinistra
                print("Obstacle forward, but not on the left")
                print("Turning left with the ball")
                twist = perform_movement(0.1,0.5)
            else:
                # ho l'ostacolo solo a centro o in tutte e tre
                print("Obstacle in all directions")
                if yaw_robot > target_yaw and yaw_diff > 0.1:
                    print("Aligning with the goal")
                    print("Trying to go around on the right")
                    twist = perform_movement(0.1,-0.5)
                elif  yaw_robot < target_yaw and yaw_diff > 0.1:
                    print("Aligning with the goal")
                    print("Trying to go around on the left")
                    twist = perform_movement(0.1,0.5)
                else:
                    # default gira a sinistra se robot e gia allineato e ha l'ostacolo al centro
                    print("Aligned with the goal")
                    print("Trying to go around on the left")
                    twist = perform_movement(0.1,0.5)
       

    elif no_ball_detected_counter == 0 and info_obstacle[0] == 0 and info_obstacle[1] == 0 and info_obstacle[2] == 0:  # Movement when the ball is far away, but detected

        print("The ball is detected, but far away")

        if (x_ball < 280 or x_ball > 360):
            print("Aligning with the ball")
            if(x_ball > 360):
                print("Turning right")
                twist = perform_movement(0.0,-1)
            else:
                print("Turning left")
                twist = perform_movement(0.0,1)
        
        else:
            print("Aligned with the ball")
            print("Going forward")
            twist = perform_movement(0.1,0)

    else:

        print("The ball is not detected")
        # Alternating between 2 phases:
        # Phase 1: Turn 360 deg. to look around for the ball
        # Phase 2: Move towards the goal

        #FRAMES_BETWEEN_PHASES = 500

        # To switch between phases
        if x_robot_saved + 1.4 <= x_robot <= x_robot_saved + 1.6:
            is_in_phase_1 = True  # Toggle the boolean
            x_robot_saved = x_robot
            

        # Phase 1
        print(yaw_robot,yaw_robot_saved)
        if is_in_phase_1:
            print("Looking around for the ball")
            print("Turning left")
            if save_yaw_robot:
                yaw_robot_saved = yaw_robot
                save_yaw_robot = False
            twist = perform_movement(0.0,1)

            phase_counter+=1
        
            if abs(yaw_robot - yaw_robot_saved) <= 0.1 and phase_counter>50:
                is_in_phase_1 = False
                save_yaw_robot = True
                twist = perform_movement(0.0,0)  # no movement? 
                phase_counter = 0

        # Phase 2
        else:
            
            print("Moving towards the goal in search of the ball")

            yaw_diff = abs(yaw_robot - target_yaw)

            if info_obstacle[1] == 0 and info_obstacle[0] == 0 and info_obstacle[2] == 0:
                print("No obstacle")

                if go_forward_counter != 0:
                    print("Ignoring the goal")
                    print("Moving forward")
                    twist = perform_movement(0.1,0)
                    go_forward_counter -= 1

                else:
                    if yaw_robot > target_yaw and yaw_diff > 0.1:
                        print("Aligning with the goal")
                        print("Turning right")
                        twist = perform_movement(0.0, -1)
                    elif  yaw_robot < target_yaw and yaw_diff > 0.1:
                        print("Aligning with the goal")
                        print("Turning left")
                        twist = perform_movement(0.0, 1)
                    else:
                        # Move towards the goal
                        print("Aligned with the goal")
                        print("Moving forward")
                        twist = perform_movement(0.1,0)
            
            else:
                if info_obstacle[0] != 0:
                    print("Obstacle not on the right")
                    print("Bypassing to the right")
                    twist = perform_movement(0.05,-1)

                elif info_obstacle[2] != 0:
                    print("Obstacle not on the left")
                    print("Bypassing to the left")
                    twist = perform_movement(0.05,1)
                else:
                    print("Obstacle in all directions")
                    # if yaw_robot > target_yaw and yaw_diff > 0.1:
                    #     print("Aligning with the goal")
                    #     print("Trying to bypass to the right")
                    #     twist = perform_movement(0.05,-1)
                    # elif  yaw_robot < target_yaw and yaw_diff > 0.1:
                    #     print("Aligning with the goal")
                    #     print("Trying to bypass to the left")
                    #     twist = perform_movement(0.05,1)
                    # else:
                    # default gira a sinistra se robot e gia allineato e ha l'ostacolo al centro
                    # print("Aligned with the goal")
                    print("Trying to bypass to the left")
                    twist = perform_movement(0.05,1)
                
                go_forward_counter = 100
     
    pub.publish(twist)  

def obstacle_callback(data):
    global info_obstacle
    info_obstacle=data.data
    info_obstacle = [x/4000000 for x in info_obstacle]
    # print(info_obstacle)

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