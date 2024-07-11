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
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Int32
import pandas as pd
import dynamic_reconfigure.client
from concurrent.futures import ProcessPoolExecutor

import threading
import queue

import message_filters

from record_w_endoscope_Si.utils import measure_execution_time

def signal_handler(sig, frame):
    print("Ctrl+C detected, shutting down gracefully...")
    rospy.signal_shutdown("Application closed")
    sys.exit(0)

# Set up signal handler for graceful shutdown
signal.signal(signal.SIGINT, signal_handler)

image_sav_res = (960, 540) # (640, 480)
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

        # subscribers with message filters
        # usb_camera_sub_left = message_filters.Subscriber("/jhu_daVinci/left/image_raw", Image)
        # usb_camera_sub_right = message_filters.Subscriber("/jhu_daVinci/right/image_raw", Image)
        endo_cam_psm1_sub = message_filters.Subscriber("/PSM1/endoscope_img", Image)
        endo_cam_psm2_sub = message_filters.Subscriber("/PSM2/endoscope_img", Image)

        psm1_sub = message_filters.Subscriber("/PSM1/measured_cp", PoseStamped)
        psm1_sp_sub = message_filters.Subscriber("/PSM1/setpoint_cp", PoseStamped)
        psm1_rcm_sub = message_filters.Subscriber("PSM1/local/measured_cp", PoseStamped)
        psm1_jaw_sub = message_filters.Subscriber("PSM1/jaw/measured_js", JointState)
        psm1_jaw_sp_sub = message_filters.Subscriber("PSM1/jaw/setpoint_js", JointState)

        psm2_sub = message_filters.Subscriber("/PSM2/measured_cp", PoseStamped)
        psm2_sp_sub = message_filters.Subscriber("/PSM2/setpoint_cp", PoseStamped)
        psm2_rcm_sub = message_filters.Subscriber("PSM2/local/measured_cp", PoseStamped)
        psm2_jaw_sub = message_filters.Subscriber("PSM2/jaw/measured_js", JointState)
        psm2_jaw_sp_sub = message_filters.Subscriber("PSM2/jaw/setpoint_js", JointState)

        ecm_sub = message_filters.Subscriber("/ECM/measured_cp", PoseStamped)
        ecm_rcm_sub = message_filters.Subscriber("ECM/local/measured_cp", PoseStamped)

        # SUJ measured_cp and js
        suj1_pose_sub = message_filters.Subscriber("/SUJ/PSM1/measured_cp", PoseStamped)
        suj1_jp_sub = message_filters.Subscriber("/SUJ/PSM1/measured_js", JointState)
        suj2_pose_sub = message_filters.Subscriber("/SUJ/PSM2/measured_cp", PoseStamped)
        suj2_jp_sub = message_filters.Subscriber("/SUJ/PSM2/measured_js", JointState)
        suj3_pose_sub = message_filters.Subscriber("/SUJ/PSM3/measured_cp", PoseStamped)
        suj3_jp_sub = message_filters.Subscriber("/SUJ/PSM3/measured_js", JointState)
        suj_ecm_pose_sub = message_filters.Subscriber("/SUJ/ECM/measured_cp", PoseStamped)
        suj_ecm_jp_sub = message_filters.Subscriber("/SUJ/ECM/measured_js", JointState)

        # PSM / ECM measured_js and setpoint_js
        psm1_js_sub = message_filters.Subscriber("/PSM1/measured_js", JointState)
        psm1_set_js_sub = message_filters.Subscriber("/PSM1/setpoint_js", JointState)
        psm2_js_sub = message_filters.Subscriber("/PSM2/measured_js", JointState)
        psm2_set_js_sub = message_filters.Subscriber("/PSM2/setpoint_js", JointState)
        psm3_js_sub = message_filters.Subscriber("/PSM3/measured_js", JointState)
        psm3_set_js_sub = message_filters.Subscriber("/PSM3/setpoint_js", JointState)
        ecm_js_sub = message_filters.Subscriber("/ECM/measured_js", JointState)
        ecm_set_js_sub = message_filters.Subscriber("/ECM/setpoint_js", JointState)

        # Create a TimeSynchronizer with a queue size of 10
        # usb_camera_sub_left, usb_camera_sub_right, 
        self.ts = message_filters.ApproximateTimeSynchronizer([
            endo_cam_psm1_sub, endo_cam_psm2_sub,
            psm1_sub, psm1_sp_sub, psm1_rcm_sub, psm1_jaw_sub, psm1_jaw_sp_sub,
            psm2_sub, psm2_sp_sub, psm2_rcm_sub, psm2_jaw_sub, psm2_jaw_sp_sub,
            ecm_sub, ecm_rcm_sub, suj1_pose_sub, suj1_jp_sub, suj2_pose_sub, suj2_jp_sub,
            suj3_pose_sub, suj3_jp_sub, suj_ecm_pose_sub, suj_ecm_jp_sub,
            psm1_js_sub, psm1_set_js_sub, psm2_js_sub, psm2_set_js_sub,
            psm3_js_sub, psm3_set_js_sub, ecm_js_sub, ecm_set_js_sub
        ], queue_size=1, slop=0.03) # TODO: Does not work as the timestamp of DaVinci ECM is delayed by nature

        self.ts.registerCallback(self.callback)

    def callback(self, *args):
        print("Callback called")
        global usb_image_left, usb_image_right, endo_cam_psm1, endo_cam_psm2
        global psm1_pose, psm1_sp, psm1_rcm_pose, psm1_jaw, psm1_jaw_sp
        global psm2_pose, psm2_sp, psm2_rcm_pose, psm2_jaw, psm2_jaw_sp
        global ecm_pose, ecm_rcm_pose, kinematics_timestamp
        global suj1_pose, suj1_jp, suj2_pose, suj2_jp, suj3_pose, suj3_jp, suj_ecm_pose, suj_ecm_jp
        global psm1_js, psm1_set_js, psm2_js, psm2_set_js, psm3_js, psm3_set_js, ecm_js, ecm_set_js

        # Extract the arguments
        # usb_image_left, usb_image_right, 
        (endo_cam_psm1, endo_cam_psm2,
         psm1_pose, psm1_sp, psm1_rcm_pose, psm1_jaw, psm1_jaw_sp,
         psm2_pose, psm2_sp, psm2_rcm_pose, psm2_jaw, psm2_jaw_sp,
         ecm_pose, ecm_rcm_pose, suj1_pose, suj1_jp, suj2_pose, suj2_jp,
         suj3_pose, suj3_jp, suj_ecm_pose, suj_ecm_jp,
         psm1_js, psm1_set_js, psm2_js, psm2_set_js,
         psm3_js, psm3_set_js, ecm_js, ecm_set_js) = args

        kinematics_timestamp = endo_cam_psm1.header.stamp

        # Process the synchronized messages
        # self.process_messages()

    def process_messages(self):
        global usb_image_left, usb_image_right, endo_cam_psm1, endo_cam_psm2
        global psm1_pose, psm1_sp, psm1_rcm_pose, psm1_jaw, psm1_jaw_sp
        global psm2_pose, psm2_sp, psm2_rcm_pose, psm2_jaw, psm2_jaw_sp
        global ecm_pose, ecm_rcm_pose, kinematics_timestamp
        global suj1_pose, suj1_jp, suj2_pose, suj2_jp, suj3_pose, suj3_jp, suj_ecm_pose, suj_ecm_jp
        global psm1_js, psm1_set_js, psm2_js, psm2_set_js, psm3_js, psm3_set_js, ecm_js, ecm_set_js

        # Convert images
        try:
            usb_image_left = self.bridge.imgmsg_to_cv2(usb_image_left, desired_encoding='passthrough')
            usb_image_right = self.bridge.imgmsg_to_cv2(usb_image_right, desired_encoding='passthrough')
            endo_cam_psm1 = self.bridge.imgmsg_to_cv2(endo_cam_psm1, desired_encoding='passthrough')
            endo_cam_psm2 = self.bridge.imgmsg_to_cv2(endo_cam_psm2, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)

        # Now usb_image_left, usb_image_right, endo_cam_psm1, endo_cam_psm2, and the pose messages are synchronized and can be processed together

        # (Rest of your processing code here)

    def dynamic_reconfigure_callback(self, config):
        global isRecord
        isRecord = config["isRecord"]

def image_saver(queue):
    while True:
        item = queue.get()
        if item is None:
            break  # None is our signal to stop
        filename, image = item
        cv2.imwrite(filename, image)
        queue.task_done()
        print(f"Saved image to {filename}")

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

ros_fps = 30  # 30hz
rate = rospy.Rate(ros_fps)
requiresNewDir = True
requiresSaveCsv = False
num_frames = 0
ee_points = []

time.sleep(1)

execution_times_list = []

while not rospy.is_shutdown():
    
    # TODO: Remove later again
    time.sleep(0.1)
    continue
    
    if len(execution_times_list) == ros_fps * print_execution_time_every_n_seconds:
        execution_times_list = []

    with measure_execution_time(execution_times_list):
        if isRecord:
            if requiresNewDir:
                time_stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
                ep_dir = os.path.join("_recordings", time_stamp)
                left_img_dir = os.path.join(ep_dir, "left_img_dir")
                right_img_dir = os.path.join(ep_dir, "right_img_dir")
                endo_p1_dir = os.path.join(ep_dir, "endo_psm1")
                endo_p2_dir = os.path.join(ep_dir, "endo_psm2")

                num_frames = 0
                ee_points = []

                if not os.path.exists(ep_dir):
                    os.makedirs(ep_dir)
                    os.makedirs(left_img_dir)
                    os.makedirs(right_img_dir)
                    os.makedirs(endo_p1_dir)
                    os.makedirs(endo_p2_dir)

                requiresNewDir = False
                requiresSaveCsv = True

            print("Collect kinematics")

            # TODO: Need to add pose, position, .. as in the first line
            ee_points.append([
                kinematics_timestamp,
                psm1_pose.pose.position.x, psm1_pose.pose.position.y, psm1_pose.pose.position.z,
                psm1_pose.orientation.x, psm1_pose.orientation.y, psm1_pose.orientation.z, psm1_pose.orientation.w,
                psm1_sp.position.x, psm1_sp.position.y, psm1_sp.position.z,
                psm1_sp.orientation.x, psm1_sp.orientation.y, psm1_sp.orientation.z, psm1_sp.orientation.w,
                psm1_jaw, psm1_jaw_sp,
                psm1_rcm_pose.position.x, psm1_rcm_pose.position.y, psm1_rcm_pose.position.z,
                psm1_rcm_pose.orientation.x, psm1_rcm_pose.orientation.y, psm1_rcm_pose.orientation.z, psm1_rcm_pose.orientation.w,
                psm2_pose.position.x, psm2_pose.position.y, psm2_pose.position.z,
                psm2_pose.orientation.x, psm2_pose.orientation.y, psm2_pose.orientation.z, psm2_pose.orientation.w,
                psm2_sp.position.x, psm2_sp.position.y, psm2_sp.position.z,
                psm2_sp.orientation.x, psm2_sp.orientation.y, psm2_sp.orientation.z, psm2_sp.orientation.w,
                psm2_jaw, psm2_jaw_sp,
                psm2_rcm_pose.position.x, psm2_rcm_pose.position.y, psm2_rcm_pose.position.z,
                psm2_rcm_pose.orientation.x, psm2_rcm_pose.orientation.y, psm2_rcm_pose.orientation.z, psm2_rcm_pose.orientation.w,
                ecm_pose.position.x, ecm_pose.position.y, ecm_pose.position.z,
                ecm_pose.orientation.x, ecm_pose.orientation.y, ecm_pose.orientation.z, ecm_pose.orientation.w,
                ecm_rcm_pose.position.x, ecm_rcm_pose.position.y, ecm_rcm_pose.position.z,
                ecm_rcm_pose.orientation.x, ecm_rcm_pose.orientation.y, ecm_rcm_pose.orientation.z, ecm_rcm_pose.orientation.w,
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
                psm1_js[0], psm1_js[1], psm1_js[2], psm1_js[3], psm1_js[4], psm1_js[5],
                psm1_set_js[0], psm1_set_js[1], psm1_set_js[2], psm1_set_js[3], psm1_set_js[4], psm1_set_js[5],
                psm2_js[0], psm2_js[1], psm2_js[2], psm2_js[3], psm2_js[4], psm2_js[5],
                psm2_set_js[0], psm2_set_js[1], psm2_set_js[2], psm2_set_js[3], psm2_set_js[4], psm2_set_js[5],
                psm3_js[0], psm3_js[1], psm3_js[2], psm3_js[3], psm3_js[4], psm3_js[5],
                psm3_set_js[0], psm3_set_js[1], psm3_set_js[2], psm3_set_js[3], psm3_set_js[4], psm3_set_js[5],
                ecm_js[0], ecm_js[1], ecm_js[2], ecm_js[3],
                ecm_set_js[0], ecm_set_js[1], ecm_set_js[2], ecm_set_js[3]
            ])

            save_name_left = os.path.join(left_img_dir, f"frame{num_frames:06d}_left.jpg")
            save_name_right = os.path.join(right_img_dir, f"frame{num_frames:06d}_right.jpg")
            save_name_endo_p1 = os.path.join(endo_p1_dir, f"frame{num_frames:06d}_psm1.jpg")
            save_name_endo_p2 = os.path.join(endo_p2_dir, f"frame{num_frames:06d}_psm2.jpg")

            print("Save images")

            if image_sav_res is None:
                image_queue.put((save_name_left, cv2.cvtColor(usb_image_left, cv2.COLOR_BGR2RGB)))
                image_queue.put((save_name_right, cv2.cvtColor(usb_image_right, cv2.COLOR_BGR2RGB)))
                image_queue.put((save_name_endo_p1, endo_cam_psm1))
                image_queue.put((save_name_endo_p2, endo_cam_psm2))
            else:
                image_queue.put((save_name_left, cv2.cvtColor(cv2.resize(usb_image_left, image_sav_res), cv2.COLOR_BGR2RGB)))
                image_queue.put((save_name_right, cv2.cvtColor(cv2.resize(usb_image_right, image_sav_res), cv2.COLOR_BGR2RGB)))
                image_queue.put((save_name_endo_p1, endo_cam_psm1))
                image_queue.put((save_name_endo_p2, endo_cam_psm2))

            num_frames += 1

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            requiresNewDir = True
            cv2.destroyAllWindows()
            if requiresSaveCsv:
                header = [
                    "timestamp",
                    "psm1_pose.position.x", "psm1_pose.position.y", "psm1_pose.position.z",
                    "psm1_pose.orientation.x", "psm1_pose.orientation.y", "psm1_pose.orientation.z", "psm1_pose.orientation.w",
                    "psm1_sp.position.x", "psm1_sp.position.y", "psm1_sp.position.z",
                    "psm1_sp.orientation.x", "psm1_sp.orientation.y", "psm1_sp.orientation.z", "psm1_sp.orientation.w",
                    "psm1_jaw", "psm1_jaw_sp",
                    "psm1_rcm_pose.position.x", "psm1_rcm_pose.position.y", "psm1_rcm_pose.position.z",
                    "psm1_rcm_pose.orientation.x", "psm1_rcm_pose.orientation.y", "psm1_rcm_pose.orientation.z", "psm1_rcm_pose.orientation.w",
                    "psm2_pose.position.x", "psm2_pose.position.y", "psm2_pose.position.z",
                    "psm2_pose.orientation.x", "psm2_pose.orientation.y", "psm2_pose.orientation.z", "psm2_pose.orientation.w",
                    "psm2_sp.position.x", "psm2_sp.position.y", "psm2_sp.position.z",
                    "psm2_sp.orientation.x", "psm2_sp.orientation.y", "psm2_sp.orientation.z", "psm2_sp.orientation.w",
                    "psm2_jaw", "psm2_jaw_sp",
                    "psm2_rcm_pose.position.x", "psm2_rcm_pose.position.y", "psm2_rcm_pose.position.z",
                    "psm2_rcm_pose.orientation.x", "psm2_rcm_pose.orientation.y", "psm2_rcm_pose.orientation.z", "psm2_rcm_pose.orientation.w",
                    "ecm_pose.position.x", "ecm_pose.position.y", "ecm_pose.position.z",
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

                print("Save csv")

                csv_data = pd.DataFrame(ee_points)
                ee_save_path = os.path.join(ep_dir, "ee_csv.csv")
                csv_data.to_csv(ee_save_path, index=False, header=header)

                requiresSaveCsv = False

    rate.sleep()

# When everything done, destroy all windows
cv2.destroyAllWindows()
