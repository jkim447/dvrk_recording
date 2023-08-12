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
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Int32
import pandas as pd
import dynamic_reconfigure.client
import dvrk
import crtk

usb_image = None
psm1_pose = None
psm1_rcm_pose = None

psm2_pose = None
psm2_rcm_pose = None

class ros_topics:

  def __init__(self):
    self.bridge = CvBridge()
    self.dynamic_reconfigure_client = dynamic_reconfigure.client.Client("dynamic_tutorials", timeout=10,
                                            config_callback = self.dynamic_reconfigure_callback)

    # subscribers
    self.usb_camera_sub = rospy.Subscriber("/jhu_daVinci/left/decklink/jhu_daVinci_left/image_raw/compressed", 
                                            CompressedImage, self.get_camera_image)

    self.psm1_sub = rospy.Subscriber("/PSM1/measured_cp", 
                                            PoseStamped, self.get_psm1_pose)

    self.psm1_RCM_sub = rospy.Subscriber("SUJ/PSM1/measured_cp", 
                                            PoseStamped, self.get_psm1_rcm_pose)

    self.psm2_sub = rospy.Subscriber("/PSM2/measured_cp", 
                                            PoseStamped, self.get_psm2_pose)
    
    self.psm2_RCM_sub = rospy.Subscriber("SUJ/PSM2/measured_cp", 
                                            PoseStamped, self.get_psm2_rcm_pose)
                                          
  
  def dynamic_reconfigure_callback(self, config):
    global isRecord
    isRecord = config["isRecord"]

  def get_camera_image(self,data):
    global usb_image
    usb_image = data

  def get_psm1_pose(self, data):
    global psm1_pose
    psm1_pose = data.pose

  def get_psm1_rcm_pose(self, data):
    global psm1_rcm_pose
    psm1_rcm_pose = data.pose

  def get_psm2_pose(self, data):
    global psm2_pose
    psm2_pose = data.pose

  def get_psm2_rcm_pose(self, data):
    global psm2_rcm_pose
    psm2_rcm_pose = data.pose

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

      # also reset indices and other stuff
      num_frames = 0
      ee_points = []

      if not os.path.exists(ep_dir):
        os.makedirs(ep_dir)

      requiresNewDir = False
      # since we just made a new dir, we need to save csv later
      requiresSaveCsv = True
    
    # Capture frame-by-frame
    # frame_no_ann = bridge.imgmsg_to_cv2(usb_image, desired_encoding = 'passthrough')
    # frame = cv2.resize(frame, (640, 480))
    # cv2.imshow('frame', frame)

    np_arr = np.fromstring(usb_image.data, np.uint8)
    # image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
    
    cv2.imshow('cv_img', frame)

    # PyKDL.Frame
    ee_points.append([psm1_pose.position.x, psm1_pose.position.y, psm1_pose.position.z,
     psm1_pose.orientation.x, psm1_pose.orientation.y, psm1_pose.orientation.z, psm1_pose.orientation.w,
     psm2_pose.position.x, psm2_pose.position.y, psm2_pose.position.z,
     psm2_pose.orientation.x, psm2_pose.orientation.y, psm2_pose.orientation.z, psm2_pose.orientation.w
     ])

    print(psm2_pose)
    print(ee_points[-1])


    # save frame
    save_name = os.path.join(ep_dir, "frame{:06d}".format(num_frames) + ".jpg")
    # write the image
    cv2.imwrite(save_name, frame) # UNCOMMENT ME WHEN DEBUGGING IS OVER (early)
    
    num_frames = num_frames + 1

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

  else:
    # if not recording, the flag for creating a new directory should be set to true
    requiresNewDir = True
    cv2.destroyAllWindows()

    if requiresSaveCsv is True:
      # save ee points
      header =  ["psm1_pos_x", "psm1_pos_y", "psm1_pos_z",
                "psm1_rot_x", "psm1_rot_y", "psm1_rot_z", "psm1_rot_w"]
      csv_data = pd.DataFrame(ee_points)
      ee_save_path = os.path.join(ep_dir, "ee_csv.csv")
      csv_data.to_csv(ee_save_path, index = False, header = header)

      # make sure to set this back to False
      requiresSaveCsv = False
    
  # make sure we spin at 30hz
  rate.sleep()


# When everything done, destroy all windows
cv2.destroyAllWindows()
