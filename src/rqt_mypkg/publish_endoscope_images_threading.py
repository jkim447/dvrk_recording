#!/usr/bin/env python

import numpy as np
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
import threading

psm1_idx = 2
psm2_idx = 0

desired_width = 640
desired_height = 480

cap1 = cv.VideoCapture(psm1_idx)
cap2 = cv.VideoCapture(psm2_idx)

cap1.set(cv.CAP_PROP_FRAME_WIDTH, desired_width)
cap1.set(cv.CAP_PROP_FRAME_HEIGHT, desired_height)

cap2.set(cv.CAP_PROP_FRAME_WIDTH, desired_width)
cap2.set(cv.CAP_PROP_FRAME_HEIGHT, desired_height)

print("cap1 Width: ", cap1.get(cv.CAP_PROP_FRAME_WIDTH))
print("Height: ", cap1.get(cv.CAP_PROP_FRAME_HEIGHT))

print("cap2 Width: ", cap2.get(cv.CAP_PROP_FRAME_WIDTH))
print("Height: ", cap2.get(cv.CAP_PROP_FRAME_HEIGHT))

pub1 = rospy.Publisher("/PSM1/endoscope_img", Image, queue_size=10)
pub2 = rospy.Publisher("/PSM2/endoscope_img", Image, queue_size=10)

bridge = CvBridge()

if not cap1.isOpened() or not cap2.isOpened():
    print("Cannot open camera")
    exit()

rospy.init_node('endoscope_talker', anonymous=True)
rate = rospy.Rate(30)  # 30hz

def capture_and_publish(cap, pub, window_name):
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            print(f"Failed to grab frame for {window_name}")
            break
        pub.publish(bridge.cv2_to_imgmsg(frame, encoding="passthrough"))
        # cv.imshow(window_name, frame)
        if cv.waitKey(1) == ord('q'):
            rospy.signal_shutdown('User requested shutdown')
            break
        rate.sleep()

# Create and start threads
thread1 = threading.Thread(target=capture_and_publish, args=(cap1, pub1, 'right_wrist'))
thread2 = threading.Thread(target=capture_and_publish, args=(cap2, pub2, 'left_wrist'))

thread1.start()
thread2.start()

# Wait for both threads to finish
thread1.join()
thread2.join()

# When everything is done, release the captures
cap1.release()
cap2.release()
cv.destroyAllWindows()
