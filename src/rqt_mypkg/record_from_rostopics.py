#!/usr/bin/env python
import numpy as np
import cv2
import os
from datetime import datetime
import numpy as np
import time
import math
import keyboard

# for ros stuff
import rospy
from std_msgs.msg import String, Float64MultiArray, Bool, Float64
from geometry_msgs.msg import Vector3, Transform, PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage, JointState
from std_msgs.msg import Int32
import pandas as pd
import dynamic_reconfigure.client
import dvrk
import crtk

usb_image_left = None
usb_image_right = None
psm1_pose = None
psm1_sp = None
psm1_rcm_pose = None
psm1_jaw = None
psm1_jaw_sp = None

psm2_pose = None
psm2_sp = None
psm2_rcm_pose = None
psm2_jaw = None
psm2_jaw_sp = None

ecm_pose = None
ecm_rcm_pose = None

class ros_topics:

  def __init__(self):
    self.bridge = CvBridge()
    self.dynamic_reconfigure_client = dynamic_reconfigure.client.Client("dynamic_tutorials", timeout=10,
                                            config_callback = self.dynamic_reconfigure_callback)

    # subscribers
    self.usb_camera_sub_left = rospy.Subscriber("/jhu_daVinci/left/decklink/jhu_daVinci_left/image_raw/compressed", 
                                            CompressedImage, self.get_camera_image_left)
    self.usb_camera_sub_right = rospy.Subscriber("/jhu_daVinci/right/decklink/jhu_daVinci_right/image_raw/compressed", 
                                            CompressedImage, self.get_camera_image_right)

    #psm1
    self.psm1_sub = rospy.Subscriber("/PSM1/measured_cp", 
                                            PoseStamped, self.get_psm1_pose)
    self.psm1_sp_sub = rospy.Subscriber("/PSM1/setpoint_cp",
                                            PoseStamped, self.get_psm1_setpoint)
    self.psm1_rcm_sub = rospy.Subscriber("SUJ/PSM1/measured_cp", 
                                            PoseStamped, self.get_psm1_rcm_pose)
    self.psm1_jaw_sub = rospy.Subscriber("PSM1/jaw/measured_js",
                                         JointState, self.get_psm1_jaw)
    self.psm1_jaw_sp_sub = rospy.Subscriber("PSM1/jaw/setpoint_js",
                                         JointState, self.get_psm1_jaw_sp)


    #psm2
    self.psm2_sub = rospy.Subscriber("/PSM2/measured_cp", 
                                            PoseStamped, self.get_psm2_pose)
    
    self.psm2_sp_sub = rospy.Subscriber("/PSM2/setpoint_cp",
                                            PoseStamped, self.get_psm2_setpoint)
    
    self.psm2_rcm_sub = rospy.Subscriber("SUJ/PSM2/measured_cp", 
                                            PoseStamped, self.get_psm2_rcm_pose)
    
    self.psm2_jaw_sub = rospy.Subscriber("PSM2/jaw/measured_js",
                                         JointState, self.get_psm2_jaw)
    self.psm2_jaw_sp_sub = rospy.Subscriber("PSM2/jaw/setpoint_js",
                                         JointState, self.get_psm2_jaw_sp)


    # ecm
    self.ecm_sub = rospy.Subscriber("/ECM/measured_cp",
                                      PoseStamped, self.get_ecm_pose)
    self.ecm_rcm_sub = rospy.Subscriber("/SUJ/ECM/measured_cp",
                                          PoseStamped, self.get_ecm_rcm_pose)
  
  
  def dynamic_reconfigure_callback(self, config):
    global isRecord
    isRecord = config["isRecord"]

  def get_camera_image_left(self,data):
    global usb_image_left
    usb_image_left = data
  
  def get_camera_image_right(self,data):
    global usb_image_right
    usb_image_right = data

  def get_ecm_rcm_pose(self, data):
    global ecm_rcm_pose
    ecm_rcm_pose = data.pose

  def get_ecm_pose(self, data):
    global ecm_pose
    ecm_pose = data.pose

  def get_psm1_pose(self, data):
    global psm1_pose
    psm1_pose = data.pose

  def get_psm1_setpoint(self, data):
    global psm1_sp
    psm1_sp = data.pose

  def get_psm1_rcm_pose(self, data):
    global psm1_rcm_pose
    psm1_rcm_pose = data.pose

  def get_psm2_pose(self, data):
    global psm2_pose
    psm2_pose = data.pose

  def get_psm2_setpoint(self, data):
    global psm2_sp
    psm2_sp = data.pose

  def get_psm2_rcm_pose(self, data):
    global psm2_rcm_pose
    psm2_rcm_pose = data.pose

  def get_psm1_jaw(self, data):
    global psm1_jaw
    psm1_jaw = data.position[0]

  def get_psm1_jaw_sp(self, data):
    global psm1_jaw_sp
    psm1_jaw_sp = data.position[0]

  def get_psm2_jaw(self, data):
    global psm2_jaw
    psm2_jaw = data.position[0]

  def get_psm2_jaw_sp(self, data):
    global psm2_jaw_sp
    psm2_jaw_sp = data.position[0]


#Create ROS publishers and subscribers
rospy.init_node('rostopic_recorder', anonymous=True)
rt = ros_topics()
time.sleep(0.5)

rate = rospy.Rate(30) # ROS Rate at 5Hz
requiresNewDir = True
requiresSaveCsv = False
num_frames = 0
ee_points = []

while(True):
  if isRecord:
    # create a new dir in the beginning
    if requiresNewDir:
      time_stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
      ep_dir = os.path.join("_recordings", time_stamp)
      left_img_dir = os.path.join(ep_dir, "left_img_dir")
      right_img_dir = os.path.join(ep_dir, "right_img_dir")

      # also reset indices and other stuff
      num_frames = 0
      ee_points = []

      if not os.path.exists(ep_dir):
        os.makedirs(ep_dir)
        os.makedirs(left_img_dir)
        os.makedirs(right_img_dir)

      requiresNewDir = False
      # since we just made a new dir, we need to save csv later
      requiresSaveCsv = True
    
    # Capture frame-by-frame
    # frame_no_ann = bridge.imgmsg_to_cv2(usb_image, desired_encoding = 'passthrough')
    # frame = cv2.resize(frame, (640, 480))
    # cv2.imshow('frame', frame)

    np_arr_left = np.fromstring(usb_image_left.data, np.uint8)
    np_arr_right = np.fromstring(usb_image_right.data, np.uint8)

    frame_left = cv2.imdecode(np_arr_left, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
    frame_right = cv2.imdecode(np_arr_right, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
  
    # PyKDL.Frame
    ee_points.append([

     psm1_pose.position.x, psm1_pose.position.y, psm1_pose.position.z, # PSM1
     psm1_pose.orientation.x, psm1_pose.orientation.y, psm1_pose.orientation.z, psm1_pose.orientation.w,
     
     psm1_sp.position.x, psm1_sp.position.y, psm1_sp.position.z,
     psm1_sp.orientation.x, psm1_sp.orientation.y, psm1_sp.orientation.z, psm1_sp.orientation.w,
     
     psm1_jaw, psm1_jaw_sp,

     psm1_rcm_pose.position.x, psm1_rcm_pose.position.y, psm1_rcm_pose.position.z,
     psm1_rcm_pose.orientation.x, psm1_rcm_pose.orientation.y, psm1_rcm_pose.orientation.z, psm1_rcm_pose.orientation.w,
     
     psm2_pose.position.x, psm2_pose.position.y, psm2_pose.position.z, # PSM 2
     psm2_pose.orientation.x, psm2_pose.orientation.y, psm2_pose.orientation.z, psm2_pose.orientation.w,

     psm2_sp.position.x, psm2_sp.position.y, psm2_sp.position.z,
     psm2_sp.orientation.x, psm2_sp.orientation.y, psm2_sp.orientation.z, psm2_sp.orientation.w,

     psm2_jaw, psm2_jaw_sp,
     
     psm2_rcm_pose.position.x, psm2_rcm_pose.position.y, psm2_rcm_pose.position.z,
     psm2_rcm_pose.orientation.x, psm2_rcm_pose.orientation.y, psm2_rcm_pose.orientation.z, psm2_rcm_pose.orientation.w,

     ecm_pose.position.x, ecm_pose.position.y, ecm_pose.position.z, # ECM
     ecm_pose.orientation.x, ecm_pose.orientation.y, ecm_pose.orientation.z, ecm_pose.orientation.w,

     ecm_rcm_pose.position.x, ecm_rcm_pose.position.y, ecm_rcm_pose.position.z,
     ecm_rcm_pose.orientation.x, ecm_rcm_pose.orientation.y, ecm_rcm_pose.orientation.z, ecm_rcm_pose.orientation.w])

    # print(ecm_rcm_pose)
    # print(ee_points[-1])

    # save frame
    save_name_left = os.path.join(left_img_dir, "frame{:06d}_left".format(num_frames) + ".jpg")
    save_name_right = os.path.join(right_img_dir, "frame{:06d}_right".format(num_frames) + ".jpg")
    # write the image
    cv2.imwrite(save_name_left, frame_left) # UNCOMMENT ME WHEN DEBUGGING IS OVER (early)
    cv2.imwrite(save_name_right, frame_right) # UNCOMMENT ME WHEN DEBUGGING IS OVER (early)

    num_frames = num_frames + 1

    # cv2.imshow('frame_left', frame_left)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

  else:
    # if not recording, the flag for creating a new directory should be set to true
    requiresNewDir = True
    cv2.destroyAllWindows()
    if requiresSaveCsv is True:
      # save ee points
      header =  ["psm1_pose.position.x", "psm1_pose.position.y", "psm1_pose.position.z", # PSM1
        "psm1_pose.orientation.x", "psm1_pose.orientation.y", "psm1_pose.orientation.z", "psm1_pose.orientation.w",
        
        "psm1_sp.position.x", "psm1_sp.position.y", "psm1_sp.position.z",
        "psm1_sp.orientation.x", "psm1_sp.orientation.y", "psm1_sp.orientation.z", "psm1_sp.orientation.w",
        
        "psm1_jaw", "psm1_jaw_sp",

        "psm1_rcm_pose.position.x", "psm1_rcm_pose.position.y", "psm1_rcm_pose.position.z", 
        "psm1_rcm_pose.orientation.x", "psm1_rcm_pose.orientation.y", "psm1_rcm_pose.orientation.z", "psm1_rcm_pose.orientation.w",
        
        "psm2_pose.position.x", "psm2_pose.position.y", "psm2_pose.position.z", # PSM 2
        "psm2_pose.orientation.x", "psm2_pose.orientation.y", "psm2_pose.orientation.z", "psm2_pose.orientation.w",
        
        "psm2_sp.position.x", "psm2_sp.position.y", "psm2_sp.position.z",
        "psm2_sp.orientation.x", "psm2_sp.orientation.y", "psm2_sp.orientation.z", "psm2_sp.orientation.w",

        "psm2_jaw", "psm2_jaw_sp",

        "psm2_rcm_pose.position.x", "psm2_rcm_pose.position.y", "psm2_rcm_pose.position.z",
        "psm2_rcm_pose.orientation.x", "psm2_rcm_pose.orientation.y", "psm2_rcm_pose.orientation.z", "psm2_rcm_pose.orientation.w",

        "ecm_pose.position.x", "ecm_pose.position.y", "ecm_pose.position.z", # ECM
        "ecm_pose.orientation.x", "ecm_pose.orientation.y", "ecm_pose.orientation.z", "ecm_pose.orientation.w",

        "ecm_rcm_pose.position.x", "ecm_rcm_pose.position.y", "ecm_rcm_pose.position.z",
        "ecm_rcm_pose.orientation.x", "ecm_rcm_pose.orientation.y", "ecm_rcm_pose.orientation.z", "ecm_rcm_pose.orientation.w"
      ]
      
      csv_data = pd.DataFrame(ee_points)
      ee_save_path = os.path.join(ep_dir, "ee_csv.csv")
      csv_data.to_csv(ee_save_path, index = False, header = header)

      # make sure to set this back to False
      requiresSaveCsv = False
    
  # make sure we spin at 30hz
  rate.sleep()


# When everything done, destroy all windows
cv2.destroyAllWindows()
