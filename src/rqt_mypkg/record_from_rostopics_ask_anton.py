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

class ros_topics:

  def __init__(self):
    self.bridge = CvBridge()
    self.dynamic_reconfigure_client = dynamic_reconfigure.client.Client("dynamic_tutorials", timeout=10,
                                            config_callback = self.dynamic_reconfigure_callback)

    # subscribers
    self.usb_camera_sub = rospy.Subscriber("/jhu_daVinci/left/decklink/jhu_daVinci_left/image_raw/compressed", 
                                            CompressedImage, self.get_camera_image)

  def dynamic_reconfigure_callback(self, config):
    global isRecord
    isRecord = config["isRecord"]

  def get_camera_image(self,data):
    global usb_image
    usb_image = data

#Create ROS publishers and subscribers
# rospy.init_node('rostopic_recorder', anonymous=True)
rt = ros_topics()
time.sleep(0.5)

rate = rospy.Rate(30) # ROS Rate at 5Hz
requiresNewDir = True
requiresSaveCsv = False
num_frames = 0
ee_points = []

# Create a Python proxy for PSM1, name must match ros namespace
ral = crtk.ral('testNode')
psm1 = dvrk.psm(ral, 'PSM1')
# psm2 = dvrk.psm(ral, 'PSM2')

while(True):
  if isRecord:
    # # create a new dir in the beginning
    # if requiresNewDir:
    #   time_stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    #   ep_dir = os.path.join("_recordings", time_stamp)

    #   # also reset indices and other stuff
    #   num_frames = 0
    #   ee_points = []

    #   if not os.path.exists(ep_dir):
    #     os.makedirs(ep_dir)

    #   requiresNewDir = False
    #   # since we just made a new dir, we need to save csv later
    #   requiresSaveCsv = True
    
    # # Capture frame-by-frame
    # frame_no_ann = bridge.imgmsg_to_cv2(usb_image, desired_encoding = 'passthrough')
    # frame = cv2.resize(frame, (640, 480))
    # cv2.imshow('frame', frame)

    np_arr = np.fromstring(usb_image.data, np.uint8)
    # image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
    
    cv2.imshow('cv_img', image_np)
    # cv2.waitKey(2)

    # PyKDL.Frame
    tmp = psm1.measured_cp()
    print(tmp)

    # # save frame
    # save_name = os.path.join(ep_dir, "frame{:06d}".format(num_frames) + ".jpg")
    # # write the image
    # # cv2.imwrite(save_name, frame) # UNCOMMENT ME WHEN DEBUGGING IS OVER (early)
    
    # num_frames = num_frames + 1

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

  else:
    # if not recording, the flag for creating a new directory should be set to true
    print("not recording!")
    requiresNewDir = True
    cv2.destroyAllWindows()

    # if requiresSaveCsv is True:
    #   # save ee points
    #   header =  ["vec3_x", "vec3_y", "vec3_z",
    #             "rot_x", "rot_y", "rot_z", "rot_w", 
    #             "isMovingDown",
    #             "isContact",
    #             "isMovingUp",
    #             "isPuncture",
    #             "isCannulating",
    #             "isDetectPuncture",
    #             "tt_uu", 
    #             "tt_vv",
    #             "tt_base_uu", 
    #             "tt_base_vv",
    #             "goal_uu", "goal_vv",
    #             "rcm_point_x", "rcm_point_y", "rcm_point_z",
    #             "max_corr_values",
    #             "isPuncture_probability",
    #             "sclera_error_mean_planned", 
    #             "sclera_error_max_planned", 
    #             "sclera_error_min_planned", 
    #             "sclera_error_mean_executed",
    #             "sclera_error_max_executed", 
    #             "sclera_error_min_executed"]
    #   csv_data = pd.DataFrame(ee_points)
    #   ee_save_path = os.path.join(ep_dir, "ee_csv.csv")
    #   csv_data.to_csv(ee_save_path, index = False, header = header)

    #   # make sure to set this back to False
    #   requiresSaveCsv = False
    
  # make sure we spin at 30hz
  rate.sleep()


# When everything done, destroy all windows
cv2.destroyAllWindows()
