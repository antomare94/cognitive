#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import cv2 
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from datetime import datetime
from imutils.video import FPS


tensorflowNet = cv2.dnn.readNetFromTensorflow('/home/mivia/progetto_ws/src/test-pkg/node/output_inference_graph_v1/frozen_inference_graph.pb', '/home/mivia/progetto_ws/src/test-pkg/node/graph.pbtxt')

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

def callback(data):

    global det,fps,init_track,x1_ball,y1_ball,x2_ball,y2_ball,tracker

    #print("image_callback")
    cv_image = bridge.imgmsg_to_cv2(data, "passthrough")
    rows, cols, channels = cv_image.shape

    #startTime = datetime.now()

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
                print("not ok")
        else:
            ok, bbox = tracker.update(cv_image)
            if ok:
                p1 = (int(bbox[0]), int(bbox[1]))
                p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                cv2.rectangle(cv_image, p1, p2, (255,0,0), 2, 1)
                print("tracking")
        fps += 1
        #print(fps)
        if fps > 10:
            det = False
            init_track = True
            fps = 0
    #print(datetime.now()-startTime)

    fps_val.update()
    fps_val.stop()
    print("[INFO] elasped time: {:.2f}".format(fps_val.elapsed()))
    print("[INFO] approx. FPS: {:.2f}".format(fps_val.fps()))

    cv2.imshow("img",cv_image)
    cv2.waitKey(1)
    
def listener():


    rospy.init_node('test_node')

    rospy.Subscriber("/robot1/camera1/image_raw", Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    fps_val = FPS().start()
    listener()