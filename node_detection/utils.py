#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np

bridge = CvBridge()

i=0
 
def callback(data):
    global i
    print("image_callback")
    cv_image = bridge.imgmsg_to_cv2(data, "passthrough")
    rows, cols, channels = cv_image.shape

    cv_image = cv.cvtColor(cv_image,cv.COLOR_BGR2RGB)

    cv.imshow("img",cv_image)
    key = cv.waitKey(1)
    if key == ord("s"):
        cv.imwrite("img" + str(i)+".jpg",cv_image)
        i+=1
    elif key == ord("q"):
        cv.destroyAllWindows()
    
    
def listener():


    rospy.init_node('test_node')

    rospy.Subscriber("/robot1/camera1/image_raw", Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()