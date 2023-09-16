#!/usr/bin/python

import rospy
import rosnode
import time

import tensorflow as tf
import sys
import os
import time
import datetime

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge, CvBridgeError
import cv2

print("before pytorch")

import torch

print("after pytorch")

class Midas_ROS:

    def __init__(self):

        self.got_image = False

        rospy.init_node("midas", anonymous=True)

        rospy.Subscriber("/uav1/tellopy_wrapper/rgb/image_raw", Image, self.callback)
        rospy.Subscriber("/uav1/tellopy_wrapper/rgb/camera_info", CameraInfo, self.callbackCameraInfo)

        self.publisher_image = rospy.Publisher("/uav1/tellopy_wrapper/depth/image_raw", Image, queue_size=1)
        self.publisher_cam_info = rospy.Publisher("/uav1/tellopy_wrapper/depth/camera_info", CameraInfo, queue_size=1)

        rospy.loginfo('ros node initialized')

        rospy.loginfo('opening cv bridge')

        self.bridge = CvBridge()

        rospy.loginfo('cv bridge opened')

        # model_type = "DPT_Large"     # MiDaS v3 - Large     (highest accuracy, slowest inference speed)
        # model_type = "DPT_Hybrid"   # MiDaS v3 - Hybrid    (medium accuracy, medium inference speed)
        model_type = "MiDaS_small"  # MiDaS v2.1 - Small   (lowest accuracy, highest inference speed)

        rospy.loginfo('loading midas mode {}'.format(model_type))

        midas = torch.hub.load("intel-isl/MiDaS", model_type)

        rospy.loginfo('initializing device')

        # device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
        device = torch.device("cpu")
        midas.to(device)
        midas.eval()

        rospy.loginfo('loading transforms')

        midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")

        rospy.loginfo('midas loaded')

        if model_type == "DPT_Large" or model_type == "DPT_Hybrid":
            transform = midas_transforms.dpt_transform
        else:
            transform = midas_transforms.small_transform

        with torch.no_grad():

            self.rate = rospy.Rate(30)

            while True:

                if not self.got_image:
                    rospy.loginfo_throttle(1.0, 'waiting for image')
                    self.rate.sleep()
                    continue

                original_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)

                input_batch = transform(original_image).to(device)

                prediction = torch.nn.functional.interpolate(
                    prediction.unsqueeze(1),
                    size=original_image.shape[:2],
                    mode="bicubic",
                    align_corners=False,
                ).squeeze()

                output = prediction.cpu().numpy()

                image_message = self.bridge.cv2_to_imgmsg(output, encoding="16UC1")
                image_message.header = self.image_header

                self.camera_info.header = self.image_header

                self.publisher_cam_info.publish(self.camera_info)

                self.publisher_image.publish(image_message)

    def callback(self, msg):
        rospy.loginfo_once('getting images')
        self.image = self.bridge.imgmsg_to_cv2(msg)
        self.image_header = msg.header
        self.got_image = True

    def callbackCameraInfo(self, msg):
        rospy.loginfo_once('getting camera info')
        self.camera_info = msg

if __name__ == '__main__':
    try:
        pydnet_ros = Midas_ROS()
    except rospy.ROSInterruptException:
        pass
