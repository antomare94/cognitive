#!/usr/bin/env python
import rospy
from std_msgs.msg import String,Int16MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

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
orientation_z = 0

def callback(data):

    global fps_no_det,x_ball_old,y_ball_old

    x_ball,y_ball = data.data

    ball_vicina = False

    print(x_ball)
    print(y_ball)


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
                    twist = perform_movement(0.0,-2)
                else:
                    print("giro a sinistra")
                    twist = perform_movement(0.0,2)
            else:
                twist = perform_movement(1,0)
                print("vado avanti")

    else:
        fps_no_det = 0
        x_ball_old = x_ball
        y_ball_old = y_ball
        if (x_ball < 280 or x_ball > 360):
            if(x_ball > 360):
                print("giro a destra")
                twist = perform_movement(0.0,-2)
            else:
                print("giro a sinistra")
                twist = perform_movement(0.0,2)
        else:
            twist = perform_movement(1,0)
            print("vado avanti")
        if y_ball < 50:
            ball_vicina = True



    pub.publish(twist)  
    
def odomCallback(msg):
    global x_robot,y_robot,orientation_z

    x_robot = msg.pose.pose.position.x
    y_robot = msg.pose.pose.position.y

    orientation_z = msg.pose.pose.orientation.z


    


def listener():


    rospy.init_node('node_movement')

    #rospy.Subscriber("/Ball_Info", Int16MultiArray, callback)

    rospy.Subscriber("/robot1/odom", Odometry, odomCallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=1)
    listener()