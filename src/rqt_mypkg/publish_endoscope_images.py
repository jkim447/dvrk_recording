#!/usr/bin/env python

import numpy as np
import cv2 as cv
from sensor_msgs.msg import Image, CompressedImage, JointState
from cv_bridge import CvBridge
import rospy

psm1_idx = 0
psm2_idx = 2

cap1 = cv.VideoCapture(psm1_idx)
cap2 = cv.VideoCapture(psm2_idx)

# subscriber
pub1 = rospy.Publisher("/PSM1/endoscope_img", 
                        Image, queue_size=10)

pub2 = rospy.Publisher("/PSM2/endoscope_img", 
                        Image, queue_size=10)

bridge = CvBridge()

if not cap1.isOpened() or not cap2.isOpened():
 print("Cannot open camera")
 exit()

rospy.init_node('endoscope_talker', anonymous=True)
rate = rospy.Rate(30) # 10hz

while not rospy.is_shutdown():
 # Capture frame-by-frame
 ret1, frame1 = cap1.read()
 ret2, frame2 = cap2.read()

 pub1.publish(bridge.cv2_to_imgmsg(frame1, encoding="passthrough"))
 pub2.publish(bridge.cv2_to_imgmsg(frame2, encoding="passthrough"))
 
 # Display the resulting frame
 cv.imshow('frame', frame1)
 cv.imshow('frame2', frame2)

 if cv.waitKey(1) == ord('q'):
    break
 
 rate.sleep()

# When everything done, release the capture
cap1.release()
cap2.release()
cv.destroyAllWindows()