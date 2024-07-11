#!/usr/bin/env python
import numpy as np
import cv2 
import rospy
from sensor_msgs.msg import Image, CompressedImage, JointState, Joy
from cv_bridge import CvBridge, CvBridgeError
import time
from std_msgs.msg import String, Float64MultiArray, Bool, Float64
import random


# Specify the font and initialize constants
font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 1   # Font size multiplier
font_color_start = (0, 0, 255)  # Red color
font_color_stopped = (255, 255, 255)  # White color
line_type = 2
r_num = random.randint(0, 1000)

# Position of the text
position = (10, 30)  # (x, y) coordinates of the bottom-left corner of the text

usb_image_left = None
endo_cam_psm1 = None
endo_cam_psm2 = None
isRecording = None
frame_left = None

class ros_topics:

  def __init__(self):
    self.bridge = CvBridge()
    # subscribers
    self.usb_camera_sub_left = rospy.Subscriber("/jhu_daVinci/left/image_raw", Image, self.get_camera_image_left)
    self.endo_cam_psm1_sub = rospy.Subscriber("/PSM1/endoscope_img", Image, self.get_endo_cam_psm1)
    self.endo_cam_psm2_sub = rospy.Subscriber("/PSM2/endoscope_img", Image, self.get_endo_cam_psm2)
    self.s1 = rospy.Subscriber("/recording/isRecording", Bool, self.get_isRecording)
    
    # pedal
    self.sub17 = rospy.Subscriber("/footpedals/coag", Joy, self.get_pedal)

  def get_camera_image_left(self,data):
    global usb_image_left
    global usb_image_left_timestamp
    usb_image_left = bridge.imgmsg_to_cv2(data, desired_encoding = 'passthrough')
    usb_image_left_timestamp = data.header.stamp
    
  def get_endo_cam_psm1(self, data):
    global endo_cam_psm1
    endo_cam_psm1 = bridge.imgmsg_to_cv2(data, desired_encoding = 'passthrough')
    global endo_cam_psm1_timestamp
    endo_cam_psm1_timestamp = data.header.stamp

  def get_endo_cam_psm2(self,data):
    global endo_cam_psm2
    endo_cam_psm2 = bridge.imgmsg_to_cv2(data, desired_encoding = 'passthrough')
    global endo_cam_psm2_timestamp
    endo_cam_psm2_timestamp = data.header.stamp
    
  def get_pedal(self, data):
    global pedal
    pedal = data.buttons[0]
    
  def get_isRecording(self, data):
    global isRecording
    isRecording = data.data

#Create ROS publishers and subscribers
bridge = CvBridge()
rospy.init_node('rostopic_recorder', anonymous=True)
rt = ros_topics()
time.sleep(0.5)

ros_fps = 9 # 30hz
rate = rospy.Rate(ros_fps)
scale = 1.8

while True:
 
  # Display the resulting frame
  frame_left = cv2.cvtColor(cv2.resize(usb_image_left, (int(scale*480), int(scale*270))), cv2.COLOR_BGR2RGB)
  
  if isRecording:
    cv2.imshow('frame_left' + str(r_num), cv2.putText(frame_left, 
                'RECORDING NOW', position, font, font_scale, font_color_start, line_type))
  elif not isRecording:
    cv2.imshow('frame_left' + str(r_num), cv2.putText(frame_left, 
                'RECORDING STOPPED', position, font, font_scale, font_color_stopped, line_type))
      
  cv2.imshow('right_wrist' + str(r_num), endo_cam_psm1)
  cv2.imshow('left_wrist' + str(r_num), endo_cam_psm2)
 
  if cv2.waitKey(1) == ord('q'):
      break
    
  rate.sleep()

# When everything done, release the capture
cv2.destroyAllWindows()