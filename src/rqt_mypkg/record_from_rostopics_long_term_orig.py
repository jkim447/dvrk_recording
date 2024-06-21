#!/usr/bin/env python
import numpy as np
import cv2
import os
from datetime import datetime
import time
import signal
import sys

# for ros stuff
import rospy
from std_msgs.msg import String, Float64MultiArray, Bool, Float64
from geometry_msgs.msg import Vector3, Transform, PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage, JointState
from std_msgs.msg import Int32
import pandas as pd
import dynamic_reconfigure.client
from concurrent.futures import ProcessPoolExecutor

import threading
import queue

from record_w_endoscope_Si.utils import measure_execution_time

def signal_handler(sig, frame):
    print("Ctrl+C detected, shutting down gracefully...")
    rospy.signal_shutdown("Application closed")
    sys.exit(0)

# Set up signal handler for graceful shutdown
signal.signal(signal.SIGINT, signal_handler)

wrist_image_sav_res = (640, 480)
endo_image_save_res = (960, 540)
print_execution_time_every_n_seconds = 5

# Initialize isRecord - otherwise it throws an error in main loop if "dynamic_reconfigure_callback" not called yet
isRecord = False

usb_image_left = None
usb_image_right = None
endo_cam_psm1 = None
endo_cam_psm2 = None

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

kinematics_timestamp = usb_image_left_timestamp = usb_image_right_timestamp = endo_cam_psm1_timestamp = endo_cam_psm2_timestamp = None

# SUJ measured_cp and js
suj1_pose = suj1_jp = None # SUJ/PSM1/measured_cp, measured_js
suj2_pose = suj2_jp = None # SUJ/PSM2/measured_cp, measured_js
suj3_pose = suj3_jp = None # SUJ/PSM3/measured_cp, measured_js
suj_ecm_pose = suj_ecm_jp = None # SUJ/ECM/measured_cp, measured_js

# psm / ecm measured_js and setpoint_js
psm1_js = psm1_set_js = None  #PSM1/measured_js, setpoint_js
psm2_js = psm2_set_js = None  #PSM2/measured_js, setpoint_js
psm3_js = psm3_set_js = None  #PSM3/measured_js, setpoint_js
ecm_js = ecm_set_js = None # ECM/measured_js, setpoint_js

class ros_topics:

  def __init__(self):
    self.bridge = CvBridge()
    self.dynamic_reconfigure_client = dynamic_reconfigure.client.Client("dynamic_tutorials", timeout=10,
                                            config_callback = self.dynamic_reconfigure_callback)

    # subscribers
    self.usb_camera_sub_left = rospy.Subscriber("/jhu_daVinci/left/image_raw", Image, self.get_camera_image_left)
    self.usb_camera_sub_right = rospy.Subscriber("/jhu_daVinci/right/image_raw", Image, self.get_camera_image_right)
    
    # endoscope imgs
    self.endo_cam_psm1_sub = rospy.Subscriber("/PSM1/endoscope_img", 
                                            Image, self.get_endo_cam_psm1)
    self.endo_cam_psm2_sub = rospy.Subscriber("/PSM2/endoscope_img", 
                                            Image, self.get_endo_cam_psm2)

    #psm1
    self.psm1_sub = rospy.Subscriber("/PSM1/measured_cp", PoseStamped, self.get_psm1_pose)
    self.psm1_sp_sub = rospy.Subscriber("/PSM1/setpoint_cp", PoseStamped, self.get_psm1_setpoint)
    self.psm1_rcm_sub = rospy.Subscriber("PSM1/local/measured_cp", PoseStamped, self.get_psm1_rcm_pose)
    self.psm1_jaw_sub = rospy.Subscriber("PSM1/jaw/measured_js", JointState, self.get_psm1_jaw)
    self.psm1_jaw_sp_sub = rospy.Subscriber("PSM1/jaw/setpoint_js", JointState, self.get_psm1_jaw_sp)

    #psm2
    self.psm2_sub = rospy.Subscriber("/PSM2/measured_cp", PoseStamped, self.get_psm2_pose)    
    self.psm2_sp_sub = rospy.Subscriber("/PSM2/setpoint_cp", PoseStamped, self.get_psm2_setpoint)
    self.psm2_rcm_sub = rospy.Subscriber("PSM2/local/measured_cp", PoseStamped, self.get_psm2_rcm_pose)
    self.psm2_jaw_sub = rospy.Subscriber("PSM2/jaw/measured_js", JointState, self.get_psm2_jaw)
    self.psm2_jaw_sp_sub = rospy.Subscriber("PSM2/jaw/setpoint_js", JointState, self.get_psm2_jaw_sp)

    # ecm
    self.ecm_sub = rospy.Subscriber("/ECM/measured_cp", PoseStamped, self.get_ecm_pose)
    self.ecm_rcm_sub = rospy.Subscriber("ECM/local/measured_cp", PoseStamped, self.get_ecm_rcm_pose)
    
    # sujs
    self.sub1 = rospy.Subscriber("/SUJ/PSM1/measured_cp", PoseStamped, self.c1)
    self.sub2 = rospy.Subscriber("/SUJ/PSM1/measured_js", JointState, self.c2)
    self.sub3 = rospy.Subscriber("/SUJ/PSM2/measured_cp", PoseStamped, self.c3)
    self.sub4 = rospy.Subscriber("/SUJ/PSM2/measured_js", JointState, self.c4)
    self.sub5 = rospy.Subscriber("/SUJ/PSM3/measured_cp", PoseStamped, self.c5)
    self.sub6 = rospy.Subscriber("/SUJ/PSM3/measured_js", JointState, self.c6)
    self.sub7 = rospy.Subscriber("/SUJ/ECM/measured_cp", PoseStamped, self.c7)
    self.sub8 = rospy.Subscriber("/SUJ/ECM/measured_js", JointState, self.c8)
    
    # psms js
    self.sub9 = rospy.Subscriber("/PSM1/measured_js", JointState, self.c9)
    self.sub10 = rospy.Subscriber("/PSM1/setpoint_js", JointState, self.c10)
    self.sub11 = rospy.Subscriber("/PSM2/measured_js", JointState, self.c11)
    self.sub12 = rospy.Subscriber("/PSM2/setpoint_js", JointState, self.c12)
    self.sub13 = rospy.Subscriber("/PSM3/measured_js", JointState, self.c13)
    self.sub14 = rospy.Subscriber("/PSM3/setpoint_js", JointState, self.c14)
    self.sub15 = rospy.Subscriber("/ECM/measured_js", JointState, self.c15)
    self.sub16 = rospy.Subscriber("/ECM/setpoint_js", JointState, self.c16)
        
    # Define the codec and create VideoWriter object
    time_stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    self.fourcc = cv2.VideoWriter_fourcc(*'avc1')# *'X264' is not supported
    self.vid_left = cv2.VideoWriter('_recordings_long_term/endoscope_left_' + time_stamp + '.mp4', self.fourcc, 30, endo_image_save_res)
    self.vid_right = cv2.VideoWriter('_recordings_long_term/endoscope_right_' + time_stamp + '.mp4', self.fourcc, 30, endo_image_save_res)
    self.vid_psm1_endo = cv2.VideoWriter('_recordings_long_term/wrist_right_' + time_stamp + '.mp4', self.fourcc, 30, wrist_image_sav_res)
    self.vid_psm2_endo = cv2.VideoWriter('_recordings_long_term/wrist_left_' + time_stamp + '.mp4', self.fourcc, 30, wrist_image_sav_res)
    
  def c1(self, data):
    global suj1_pose
    suj1_pose = data.pose

  def c2(self, data):
    global suj1_jp
    suj1_jp = data.position

  def c3(self, data):
    global suj2_pose
    suj2_pose = data.pose

  def c4(self, data):
    global suj2_jp
    suj2_jp = data.position

  def c5(self, data):
    global suj3_pose
    suj3_pose = data.pose

  def c6(self, data):
    global suj3_jp
    suj3_jp = data.position

  def c7(self, data):
    global suj_ecm_pose
    suj_ecm_pose = data.pose

  def c8(self, data):
    global suj_ecm_jp
    suj_ecm_jp = data.position

  ########################################
  def c9(self, data):
    global psm1_js
    psm1_js = data.position

  def c10(self, data):
    global psm1_set_js
    psm1_set_js = data.position

  def c11(self, data):
    global psm2_js
    psm2_js = data.position

  def c12(self, data):
    global psm2_set_js
    psm2_set_js = data.position

  def c13(self, data):
    global psm3_js
    psm3_js = data.position

  def c14(self, data):
    global psm3_set_js
    psm3_set_js = data.position

  def c15(self, data):
    global ecm_js
    ecm_js = data.position

  def c16(self, data):
    global ecm_set_js
    ecm_set_js = data.position

  def dynamic_reconfigure_callback(self, config):
    global isRecord
    isRecord = config["isRecord"] # TODO: Load here also other information from the GUI as the possible surgical phase that will be recorded?

  def get_camera_image_left(self,data):
    global usb_image_left
    global usb_image_left_timestamp
    usb_image_left = bridge.imgmsg_to_cv2(data, desired_encoding = 'passthrough')
    usb_image_left_timestamp = data.header.stamp
  
  def get_camera_image_right(self,data):
    global usb_image_right
    usb_image_right = bridge.imgmsg_to_cv2(data, desired_encoding = 'passthrough')
    global usb_image_right_timestamp
    usb_image_right_timestamp = data.header.stamp
  
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

  def get_ecm_pose(self, data):
    global ecm_pose
    ecm_pose = data.pose
    global kinematics_timestamp
    kinematics_timestamp = data.header.stamp

  def get_ecm_rcm_pose(self, data):
    global ecm_rcm_pose
    ecm_rcm_pose = data.pose

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

def image_saver(queue):
  while True:
    item = queue.get()
    if item is None:
      break  # None is our signal to stop
    filename, image = item
    cv2.imwrite(filename, image)
    queue.task_done()

# Create a queue to communicate with the worker thread
image_queue = queue.Queue()

# Start worker thread
worker = threading.Thread(target=image_saver, args=(image_queue,))
worker.start()

#Create ROS publishers and subscribers
bridge = CvBridge()
rospy.init_node('rostopic_recorder', anonymous=True)
rt = ros_topics()
time.sleep(0.5)

ros_fps = 30 # 30hz
rate = rospy.Rate(ros_fps)
requiresNewDir = True
requiresSaveCsv = False
num_frames = 0
ee_points = []

time.sleep(1)

execution_times_list = []

while(True):
  # Display the average execution time every n seconds
  if len(execution_times_list) == ros_fps*print_execution_time_every_n_seconds:
    #   print(f"Average execution time (publish+show wrist camera frames): {np.mean(execution_times_list)*1000:.2f} ms")
      execution_times_list = []
    #   print(f"Current queue sizes: {image_queue.qsize()}")
  
      # Publish wrist camera images + visualize them with the DaVinci Endoscope camera
  with measure_execution_time(execution_times_list):
    if isRecord: 
      # create a new dir in the beginning
      
      if requiresNewDir:
        time_stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
        ep_dir = os.path.join("_recordings", time_stamp)
        left_img_dir = os.path.join(ep_dir, "left_img_dir")
        right_img_dir = os.path.join(ep_dir, "right_img_dir")
        endo_p1_dir = os.path.join(ep_dir, "endo_psm1")
        endo_p2_dir = os.path.join(ep_dir, "endo_psm2")


        # also reset indices and other stuff
        num_frames = 0
        ee_points = []

        if not os.path.exists(ep_dir):
          os.makedirs(ep_dir)
          os.makedirs(left_img_dir)
          os.makedirs(right_img_dir)
          os.makedirs(endo_p1_dir)
          os.makedirs(endo_p2_dir)

        requiresNewDir = False
        # since we just made a new dir, we need to save csv later
        requiresSaveCsv = True
      
      # PyKDL.Frame
      ee_points.append([
      kinematics_timestamp,
      #PSM1
      psm1_pose.position.x, psm1_pose.position.y, psm1_pose.position.z, # PSM1
      psm1_pose.orientation.x, psm1_pose.orientation.y, psm1_pose.orientation.z, psm1_pose.orientation.w,
      psm1_sp.position.x, psm1_sp.position.y, psm1_sp.position.z,
      psm1_sp.orientation.x, psm1_sp.orientation.y, psm1_sp.orientation.z, psm1_sp.orientation.w,
      psm1_jaw, psm1_jaw_sp,
      psm1_rcm_pose.position.x, psm1_rcm_pose.position.y, psm1_rcm_pose.position.z,
      psm1_rcm_pose.orientation.x, psm1_rcm_pose.orientation.y, psm1_rcm_pose.orientation.z, psm1_rcm_pose.orientation.w,
      
      # PSM2
      psm2_pose.position.x, psm2_pose.position.y, psm2_pose.position.z, # PSM 2
      psm2_pose.orientation.x, psm2_pose.orientation.y, psm2_pose.orientation.z, psm2_pose.orientation.w,
      psm2_sp.position.x, psm2_sp.position.y, psm2_sp.position.z,
      psm2_sp.orientation.x, psm2_sp.orientation.y, psm2_sp.orientation.z, psm2_sp.orientation.w,
      psm2_jaw, psm2_jaw_sp,
      psm2_rcm_pose.position.x, psm2_rcm_pose.position.y, psm2_rcm_pose.position.z,
      psm2_rcm_pose.orientation.x, psm2_rcm_pose.orientation.y, psm2_rcm_pose.orientation.z, psm2_rcm_pose.orientation.w,
      # ECM
      ecm_pose.position.x, ecm_pose.position.y, ecm_pose.position.z, # ECM
      ecm_pose.orientation.x, ecm_pose.orientation.y, ecm_pose.orientation.z, ecm_pose.orientation.w,
      # ECM RCM
      ecm_rcm_pose.position.x, ecm_rcm_pose.position.y, ecm_rcm_pose.position.z,
      ecm_rcm_pose.orientation.x, ecm_rcm_pose.orientation.y, ecm_rcm_pose.orientation.z, ecm_rcm_pose.orientation.w,
      # suj poses
      suj1_pose.position.x, suj1_pose.position.y, suj1_pose.position.z,
      suj1_pose.orientation.x, suj1_pose.orientation.y, suj1_pose.orientation.z, suj1_pose.orientation.w,
      suj1_jp[0], suj1_jp[1], suj1_jp[2], suj1_jp[3],

      suj2_pose.position.x, suj2_pose.position.y, suj2_pose.position.z,
      suj2_pose.orientation.x, suj2_pose.orientation.y, suj2_pose.orientation.z, suj2_pose.orientation.w,
      suj2_jp[0], suj2_jp[1], suj2_jp[2], suj2_jp[3],

      suj3_pose.position.x, suj3_pose.position.y, suj3_pose.position.z,
      suj3_pose.orientation.x, suj3_pose.orientation.y, suj3_pose.orientation.z, suj3_pose.orientation.w,
      suj3_jp[0], suj3_jp[1], suj3_jp[2], suj3_jp[3],

      suj_ecm_pose.position.x, suj_ecm_pose.position.y, suj_ecm_pose.position.z,
      suj_ecm_pose.orientation.x, suj_ecm_pose.orientation.y, suj_ecm_pose.orientation.z, suj_ecm_pose.orientation.w,
      suj_ecm_jp[0], suj_ecm_jp[1], suj_ecm_jp[2], suj_ecm_jp[3],

      # joints
      psm1_js[0], psm1_js[1], psm1_js[2], psm1_js[3], psm1_js[4], psm1_js[5],
      psm1_set_js[0], psm1_set_js[1], psm1_set_js[2], psm1_set_js[3], psm1_set_js[4], psm1_set_js[5],

      psm2_js[0], psm2_js[1], psm2_js[2], psm2_js[3], psm2_js[4], psm2_js[5],
      psm2_set_js[0], psm2_set_js[1], psm2_set_js[2], psm2_set_js[3], psm2_set_js[4], psm2_set_js[5],

      psm3_js[0], psm3_js[1], psm3_js[2], psm3_js[3], psm3_js[4], psm3_js[5],
      psm3_set_js[0], psm3_set_js[1], psm3_set_js[2], psm3_set_js[3], psm3_set_js[4], psm3_set_js[5],

      ecm_js[0], ecm_js[1], ecm_js[2], ecm_js[3],
      ecm_set_js[0], ecm_set_js[1], ecm_set_js[2], ecm_set_js[3]
      ])
      
    #   save_name_left = os.path.join(left_img_dir, f"frame{num_frames:06d}_left.jpg")
    #   save_name_right = os.path.join(right_img_dir, f"frame{num_frames:06d}_right.jpg")
    #   save_name_endo_p1 = os.path.join(endo_p1_dir, f"frame{num_frames:06d}_psm1.jpg")
    #   save_name_endo_p2 = os.path.join(endo_p2_dir, f"frame{num_frames:06d}_psm2.jpg")

      rt.vid_left.write(cv2.cvtColor(cv2.resize(usb_image_left, endo_image_save_res), cv2.COLOR_BGR2RGB))
      rt.vid_right.write(cv2.cvtColor(cv2.resize(usb_image_right, endo_image_save_res), cv2.COLOR_BGR2RGB))
      rt.vid_psm1_endo.write(cv2.resize(endo_cam_psm1, wrist_image_sav_res))
      rt.vid_psm2_endo.write(cv2.resize(endo_cam_psm2, wrist_image_sav_res))
      num_frames = num_frames + 1

      if num_frames % 100 == 0:
          pass
        # print(f"Queue size is {image_queue.qsize()}")

      if cv2.waitKey(1) & 0xFF == ord('q'):
          break

    else:
      # if not recording, the flag for creating a new directory should be set to true
      requiresNewDir = True
      cv2.destroyAllWindows()
      if requiresSaveCsv is True:
        # save ee points
        header =  [
          "timestamp",
          
          "psm1_pose.position.x", "psm1_pose.position.y", "psm1_pose.position.z", # PSM1
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
          "ecm_rcm_pose.orientation.x", "ecm_rcm_pose.orientation.y", "ecm_rcm_pose.orientation.z", "ecm_rcm_pose.orientation.w",

          "suj1_pose.position.x", "suj1_pose.position.y", "suj1_pose.position.z",
          "suj1_pose.orientation.x", "suj1_pose.orientation.y", "suj1_pose.orientation.z", "suj1_pose.orientation.w",
          "suj1_jp[0]", "suj1_jp[1]", "suj1_jp[2]", "suj1_jp[3]",

          "suj2_pose.position.x", "suj2_pose.position.y", "suj2_pose.position.z",
          "suj2_pose.orientation.x", "suj2_pose.orientation.y", "suj2_pose.orientation.z", "suj2_pose.orientation.w",
          "suj2_jp[0]", "suj2_jp[1]", "suj2_jp[2]", "suj2_jp[3]",

          "suj3_pose.position.x", "suj3_pose.position.y", "suj3_pose.position.z",
          "suj3_pose.orientation.x", "suj3_pose.orientation.y", "suj3_pose.orientation.z", "suj3_pose.orientation.w",
          "suj3_jp[0]", "suj3_jp[1]", "suj3_jp[2]", "suj3_jp[3]",

          "suj_ecm_pose.position.x", "suj_ecm_pose.position.y", "suj_ecm_pose.position.z",
          "suj_ecm_pose.orientation.x", "suj_ecm_pose.orientation.y", "suj_ecm_pose.orientation.z", "suj_ecm_pose.orientation.w",
          "suj_ecm_jp[0]", "suj_ecm_jp[1]", "suj_ecm_jp[2]", "suj_ecm_jp[3]",

          "psm1_js[0]", "psm1_js[1]", "psm1_js[2]", "psm1_js[3]", "psm1_js[4]", "psm1_js[5]",
          "psm1_set_js[0]", "psm1_set_js[1]", "psm1_set_js[2]", "psm1_set_js[3]", "psm1_set_js[4]", "psm1_set_js[5]",

          "psm2_js[0]", "psm2_js[1]", "psm2_js[2]", "psm2_js[3]", "psm2_js[4]", "psm2_js[5]",
          "psm2_set_js[0]", "psm2_set_js[1]", "psm2_set_js[2]", "psm2_set_js[3]", "psm2_set_js[4]", "psm2_set_js[5]",

          "psm3_js[0]", "psm3_js[1]", "psm3_js[2]", "psm3_js[3]", "psm3_js[4]", "psm3_js[5]",
          "psm3_set_js[0]", "psm3_set_js[1]", "psm3_set_js[2]", "psm3_set_js[3]", "psm3_set_js[4]", "psm3_set_js[5]",

          "ecm_js[0]", "ecm_js[1]", "ecm_js[2]", "ecm_js[3]",
          "ecm_set_js[0]", "ecm_set_js[1]", "ecm_set_js[2]", "ecm_set_js[3]"
        ]
        
        csv_data = pd.DataFrame(ee_points)
        time_stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
        ee_save_path = os.path.join("_recordings_long_term", "ee_csv_" + str(time_stamp) + ".csv")
        csv_data.to_csv(ee_save_path, index = False, header = header)

        rt.vid_left.release()
        rt.vid_right.release()
        rt.vid_psm1_endo.release() 
        rt.vid_psm2_endo.release()        
        
        # make sure to set this back to False
        requiresSaveCsv = False
    
  # make sure we spin at 30hz
  rate.sleep()


# When everything done, destroy all windows
cv2.destroyAllWindows()