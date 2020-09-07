#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Int16MultiArray, Int32MultiArray
import cv2 
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from datetime import datetime
from imutils.video import FPS
import threading

script_path = os.path.dirname(os.path.realpath(__file__))
frozen_inference_graph_path = os.path.join(script_path, 'output_inference_graph_v1/frozen_inference_graph.pb')
graph_path = os.path.join(script_path, 'graph.pbtxt')

tensorflowNet = cv2.dnn.readNetFromTensorflow(frozen_inference_graph_path, graph_path)

bridge = CvBridge()

def initialize_tracker(i):

    tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'MOSSE', 'CSRT']

    tracker_type = tracker_types[i]


    if tracker_type == 'BOOSTING':
        tracker = cv2.TrackerBoosting_create()
    if tracker_type == 'MIL':
        tracker = cv2.TrackerMIL_create()
    if tracker_type == 'KCF':
        tracker = cv2.TrackerKCF_create()
    if tracker_type == 'TLD':
        tracker = cv2.TrackerTLD_create()
    if tracker_type == 'MEDIANFLOW':
        tracker = cv2.TrackerMedianFlow_create()
    if tracker_type == 'CSRT':
        tracker = cv2.TrackerCSRT_create()
    if tracker_type == 'MOSSE':
        tracker = cv2.TrackerMOSSE_create()
    return tracker

det = False
fps = 0
init_track = True
x1_ball = 0
y1_ball = 0
x2_ball = 0
y2_ball = 0
tracker = None

arr_cords_ball = [0,0]
info_obstacle = [0,0,0]
previous_image = None
image = np.zeros((2, 2))  # Blank image for initialization
result = np.zeros((2, 2))  # Blank image for initialization

semaphore = True  # Semaphore to limit when the callback function is called

def detect_obstacle(cv_image):

    global info_obstacle

    cv_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    # Threshold of blue in HSV space 
    lower_red = np.array([0,120,70])
    upper_red = np.array([10,255,255])
 
    # preparing the mask to overlay 
    mask = cv2.inRange(hsv, lower_red, upper_red) 
      
    # The black region in the mask has the value of 0, 
    # so when multiplied with original image removes all non-blue regions 
    result = cv2.bitwise_and(cv_image, cv_image, mask = mask) 
    
    #cv2.imshow('frame', cv_image) 
    #cv2.imshow('mask', mask) 
    #cv.waitKey(1)
    center_sum=np.sum(result[:, 140:result.shape[1]-140,:])
    left_sum=np.sum(result[:, :139,:])
    rigth_sum=np.sum(result[:,result.shape[1]-139:,:])

    info_obstacle[0]= left_sum 
    info_obstacle[1]= center_sum
    info_obstacle[2]= rigth_sum

    print(info_obstacle)

    return result

def callback_with_threading(data):
    global semaphore, image, arr_cords_ball

    if semaphore:
        semaphore = False
        thread = threading.Thread(target=callback, args=(data,))
        thread.start()
    else:
        print('PREVIOUS PROCESSING STEP UNFINISHED - NEW INPUT IGNORED')

    #print('[INFO] Ball detected position: ' + str(arr_cords_ball))
    #print('[NOTE] If equal to [0, 0] - no ball is detected')
    pub_ball.publish(Int16MultiArray(data=arr_cords_ball))
    pub_obstacle.publish(Int32MultiArray(data=info_obstacle))


    print('----------------------------------------')

    cv2.imshow("img", image)
    cv2.imshow('result', result) 
    cv2.waitKey(1)

def callback(data):

    #print('IMAGE PROCESSING BEGINS')

    global det, fps, init_track, x1_ball, y1_ball, x2_ball, y2_ball, tracker, semaphore, previous_image, image, arr_cords_ball, result

    cv_image = bridge.imgmsg_to_cv2(data, "passthrough")
    rows, cols, channels = cv_image.shape

    nn_ball_detected = tracking_ball_detected = False

    result = detect_obstacle(cv_image)

    if not det:

        #print("[INFO] Detection type: Neural Network")

        nn_ball_detected = False

        tensorflowNet.setInput(cv2.dnn.blobFromImage(cv_image, size=(360, 640), swapRB=True, crop=False))

        networkOutput = tensorflowNet.forward()


        for detection in networkOutput[0,0]:

    
            score = float(detection[2])
            if score > 0.5:
                #print(detection[1])
                x1 = int(detection[3] * cols)
                y1 = int(detection[4] * rows)
                x2 = int(detection[5] * cols)
                y2 = int(detection[6] * rows)
                if detection[1] == 1.0:
                    det = True
                    x1_ball, y1_ball, x2_ball, y2_ball = x1, y1, x2, y2
                    nn_ball_detected = True
        
                #draw a red rectangle around detected objects
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 0, 255), thickness=2)

    else:

        #print("[INFO] Detection type: Tracking")

        tracking_ball_detected = False

        if init_track:
            tracker = initialize_tracker(6) #oppure 2
            bbox = (x1_ball,y1_ball,abs(x2_ball-x1_ball),abs(y2_ball-y1_ball))
            ok = tracker.init(cv_image, bbox)

            if ok:
                init_track = False
                tracking_ball_detected = True

            else:
                det = False

        else:
            ok, bbox = tracker.update(cv_image)

            if ok:
                x1_ball = int(bbox[0])
                y1_ball = int(bbox[1])
                x2_ball = int(bbox[0] + bbox[2])
                y2_ball = int(bbox[1] + bbox[3])
                tracking_ball_detected = True
        
        if tracking_ball_detected:
            p1 = (x1_ball, y1_ball)
            p2 = (x2_ball, y2_ball)
            #draw a blue rectangle around the ball
            cv2.rectangle(cv_image, p1, p2, (255,0,0), 2, 1)

        fps += 1
        if fps > 30:
            det = False
            init_track = True
            fps = 0

    if nn_ball_detected or tracking_ball_detected:
        arr_cords_ball[0] = int((x1_ball+x2_ball) / 2)
        arr_cords_ball[1] = y2_ball
    else:
        arr_cords_ball = [0, 0]

    previous_image = image
    image = cv_image
    

    fps_val.update()
    fps_val.stop()
    print("[INFO] Elasped time: {:.2f}".format(fps_val.elapsed()))
    print("[INFO] Approx. FPS: {:.2f}".format(fps_val.fps()))

    semaphore = True  # Return the semaphore at the end of the thread

    
    
def listener():


    rospy.init_node('node_detection')

    rospy.Subscriber("/robot1/camera1/image_raw", Image, callback_with_threading)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    fps_val = FPS().start()

    pub_ball = rospy.Publisher('Ball_Info', Int16MultiArray, queue_size=1)
    pub_obstacle = rospy.Publisher('Obstacle_Info', Int32MultiArray, queue_size=1)
    

    listener()