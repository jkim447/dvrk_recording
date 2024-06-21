#!/usr/bin/env python

import sys
import signal
from functools import partial

import numpy as np
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy

from utils import measure_execution_time


def signal_handler(sig, frame, cap1, cap2):
    print("Ctrl+C detected, shutting down gracefully...")
    # When everything done, release the capture
    cap1.release()
    cap2.release()
    cv.destroyAllWindows()
    rospy.signal_shutdown("Application closed")
    sys.exit(0)

# --------------- Set parameters ---------------

# Adjust these indices to match the camera indices on your system (switch them if required)
psm1_idx = 2 
psm2_idx = 0 

desired_width = 640
desired_height = 480

displaying_frames_fps = 10
print_execution_time_every_n_seconds = 5
disable_livestream_flag = True

# ------------ Prepare wrist cameras ------------

cap1 = cv.VideoCapture(psm1_idx)
cap2 = cv.VideoCapture(psm2_idx)
cap1.set(cv.CAP_PROP_FRAME_WIDTH, desired_width)
cap1.set(cv.CAP_PROP_FRAME_HEIGHT, desired_height)
cap2.set(cv.CAP_PROP_FRAME_WIDTH, desired_width)
cap2.set(cv.CAP_PROP_FRAME_HEIGHT, desired_height)

# ROS publishers
pub1 = rospy.Publisher("/PSM1/endoscope_img", Image, queue_size=20)
pub2 = rospy.Publisher("/PSM2/endoscope_img", Image, queue_size=20)

bridge = CvBridge()

if not cap1.isOpened() or not cap2.isOpened():
    print("Cannot open camera")
    cap1.release()
    cap2.release()
    cv.destroyAllWindows()
    sys.exit(1)
    
# Set up signal handler for graceful shutdown
signal.signal(signal.SIGINT, partial(signal_handler, cap1=cap1, cap2=cap2))

# -------------- Prepare DaVinci Endoscope camera --------------

# ROS subscriber
class RosTopics:
    def __init__(self):
        self.usb_image_left = None
        self.usb_camera_sub_left = rospy.Subscriber("/jhu_daVinci/left/image_raw", 
                                                    Image, self.get_camera_image_left)

    def get_camera_image_left(self, data):
        self.usb_image_left = data

rt = RosTopics()


rospy.init_node('endoscope_talker', anonymous=True)
ros_fps = 30 # 30hz
rate = rospy.Rate(ros_fps)  

# -------------- Main loop --------------

image_count = 0
print_camera_dim_flag = True
execution_times_list = []
while not rospy.is_shutdown():
    # Display the average execution time every n seconds
    if len(execution_times_list) == ros_fps*print_execution_time_every_n_seconds:
        print(f"Average execution time (publish+show wrist camera frames): {np.mean(execution_times_list)*1000:.2f} ms")
        execution_times_list = []
    
    # Publish wrist camera images + visualize them with the DaVinci Endoscope camera
    with measure_execution_time(execution_times_list):
        # Capture frame-by-frame
        ret1, frame1 = cap1.read()
        ret2, frame2 = cap2.read()

        # Check if frames are captured correctly
        if not ret1 or frame1 is None:
            print("Failed to capture frame from camera 1. Stop the program.")
            break

        if not ret2 or frame2 is None:
            print("Failed to capture frame from camera 2. Stop the program.")
            break

        if print_camera_dim_flag:
            print_camera_dim_flag = False
            print("cap1 shape: ", frame1.shape)
            print("cap2 shape: ", frame2.shape)

        # Create the wrist camera messages (stick to bgr8 encoding as imwrite will then later transform it back to rgb8 for saving the image)
        img_msg1 = bridge.cv2_to_imgmsg(frame1, encoding="bgr8")
        img_msg1.header.stamp = rospy.Time.now()
        img_msg1.header.frame_id = "psm1_frame"
        pub1.publish(img_msg1)
        
        img_msg2 = bridge.cv2_to_imgmsg(frame2, encoding="bgr8") 
        img_msg2.header.stamp = rospy.Time.now()
        img_msg2.header.frame_id = "psm2_frame"
        pub2.publish(img_msg2)

        # Display the wrist camera frames and the DaVinci Endoscope camera frame
        if not disable_livestream_flag and image_count == ros_fps // displaying_frames_fps - 1:
            # Reset the image count
            image_count = 0
            
            # Process left camera image
            if rt.usb_image_left is not None:
                frame_left = bridge.imgmsg_to_cv2(rt.usb_image_left, desired_encoding='bgr8') # Convert to bgr8 for cv2.imshow (which needs bgr encoding)
                scaling_factor = frame1.shape[0] / frame_left.shape[0] # Scale the left image to match the wrist camera image height
                frame_left_scaled = cv.resize(frame_left, (0, 0), fx=scaling_factor, fy=scaling_factor)
                
                # Create a 5-pixel wide vertical line for separating the images
                separator_line = np.ones((desired_height, 5, 3), dtype=np.uint8) * 200

                # Concatenate images with the black line in between
                concatenated_frame = cv.hconcat([frame2, separator_line, frame_left_scaled, separator_line, frame1])

                
                # Display the concatenated frame
                cv.imshow('Concatenated View', concatenated_frame) # TODO: Check why this is not working
            else:
                print("Left image not available")
        else:
            image_count += 1

    # Increase waitKey delay to improve key press detection
    if cv.waitKey(10) & 0xFF == ord('q'):
        rospy.signal_shutdown('User requested shutdown')
        break
    
    rate.sleep()

# -------------- Clean up --------------

# When everything done, release the capture
cap1.release()
cap2.release()
cv.destroyAllWindows()
