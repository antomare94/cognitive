#!/usr/bin/env python
import rospy
from std_msgs.msg import String
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

    tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN', 'MOSSE', 'CSRT']

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

semaphore = True  # Semaphore to limit when the callback function is called

def callback_with_threading(data):
    global semaphore

    if semaphore:
        semaphore = False
        thread = threading.Thread(target=callback, args=(data,))
        thread.start()
    else:
        print('Image still processing - new input ignored')

def callback(data):

    global det, fps, init_track, x1_ball, y1_ball, x2_ball, y2_ball, tracker, semaphore

    cv_image = bridge.imgmsg_to_cv2(data, "passthrough")
    rows, cols, channels = cv_image.shape

    if not det:

        tensorflowNet.setInput(cv2.dnn.blobFromImage(cv_image, size=(360, 640), swapRB=True, crop=False))

        networkOutput = tensorflowNet.forward()


        for detection in networkOutput[0,0]:

    
            score = float(detection[2])
            if score > 0.5:
                #print(detection[1])
                x1 = detection[3] * cols
                y1 = detection[4] * rows
                x2 = detection[5] * cols
                y2 = detection[6] * rows
                if detection[1] == 1.0:
                    det = True
                    x1_ball, y1_ball,x2_ball,y2_ball = x1,y1,x2,y2
                    print("detect ball")
        
                #draw a red rectangle around detected objects
                cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), thickness=2)

    else:
        if init_track:
            tracker = initialize_tracker(7) #oppure 2
            bbox = (x1_ball,y1_ball,abs(x2_ball-x1_ball),abs(y2_ball-y1_ball))
            ok = tracker.init(cv_image, bbox)
            if ok:
                init_track = False
            else:
                det = False
        else:
            ok, bbox = tracker.update(cv_image)
            if ok:
                p1 = (int(bbox[0]), int(bbox[1]))
                p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                cv2.rectangle(cv_image, p1, p2, (255,0,0), 2, 1)
                punto_palla = (int(bbox[0] + bbox[2]//2), int(bbox[1] + bbox[3]))
                #cv2.circle(cv_image,punto_palla,4,(0,0,255))
                print("tracking")
        fps += 1
        if fps > 30:
            det = False
            init_track = True
            fps = 0

    fps_val.update()
    fps_val.stop()
    print("[INFO] elasped time: {:.2f}".format(fps_val.elapsed()))
    print("[INFO] approx. FPS: {:.2f}".format(fps_val.fps()))

    cv2.imshow("img",cv_image)
    cv2.waitKey(1)

    semaphore = True
    
def listener():


    rospy.init_node('test_node')

    rospy.Subscriber("/robot1/camera1/image_raw", Image, callback_with_threading)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    fps_val = FPS().start()
    listener()