#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np

print(cv.__version__)

tensorflowNet = cv.dnn.readNetFromTensorflow('/home/mivia/prog_ws/src/test-pkg/node/ssd_mobilenet_v2_coco_2018_03_29/frozen_inference_graph.pb', '/home/mivia/prog_ws/src/test-pkg/node/ssd_mobilenet_v2_coco_2018_03_29.pbtxt')

bridge = CvBridge()
 
def callback(data):
    print("image_callback")
    cv_image = bridge.imgmsg_to_cv2(data, "passthrough")
    rows, cols, channels = cv_image.shape

    tensorflowNet.setInput(cv.dnn.blobFromImage(cv_image, size=(360, 640), swapRB=True, crop=False))

    networkOutput = tensorflowNet.forward()

    print(type(networkOutput))


    for detection in networkOutput[0,0]:

    
        score = float(detection[2])
        if score > 0.2:
            print(detection[1])
            left = detection[3] * cols
            top = detection[4] * rows
            right = detection[5] * cols
            bottom = detection[6] * rows
    
            #draw a red rectangle around detected objects
            cv.rectangle(cv_image, (int(left), int(top)), (int(right), int(bottom)), (0, 0, 255), thickness=2)
 

    width = 1280
    height = 720

    cv.imshow("img",cv_image)
    cv.waitKey(1)
    
def listener():


    rospy.init_node('test_node')

    rospy.Subscriber("/robot1/camera1/image_raw", Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()