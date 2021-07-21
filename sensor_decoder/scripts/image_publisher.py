#!/usr/bin/python
import rospy
import rospkg
import time
import math
import pypcd

import os
import numpy as np


from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from sensor_decoder.msg import PCAN
from sensor_decoder.msg import PCANArray
from sensor_decoder.msg import Mobileye
from sensor_decoder.msg import LaneMsg
from sensor_decoder.msg import ObstacleMsg
from inertiallabs_msgs.msg import ins_data
from inertiallabs_msgs.msg import gps_data
from inertiallabs_msgs.msg import sensor_data
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray

import json
import datetime
import utils
from message import Message
from data import Data

import cv2
from cv_bridge import CvBridge


class ImagePublisher:
    def __init__(self, data_name):
        self.data_name = "demo3"
        self.data_path = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + self.data_name + "/"
        # self.data_path = "/media/jeongwoooh/Samsung USB/data/" + self.data_name + "/"

        self.bridge = CvBridge()
        self.valid = False

        if not os.path.exists(self.data_path):
            rospy.ERROR("NO DATA PATH! PLEASE CHECK AGAIN!!")
        

        self.state_path = self.data_path + "state/"
        self.image_raw_path = self.data_path + "image_raw/"
        self.bin_path = self.data_path + "bin/"
        self.pcd_path = self.data_path + "pcd/"
        self.bev_map_path = self.data_path + "bev_map/"
        self.local_map_path = self.data_path + "local_map/"

        self.sub = rospy.Subscriber("/query", Int32MultiArray, self.callback)
        self.pub = rospy.Publisher("/driving_image", Image)

        self.query()


    def callback(self, msg):
        self.valid = True
        self.start = msg.data[0]
        self.n_data = msg.data[1]
        self.N = 0

    def query(self):
        while not rospy.is_shutdown():
            if not self.valid:
                continue
            seq = self.start + self.N % self.n_data
            image = cv2.
            self.N += 1


if __name__=='__main__':
    rospy.init_node("data_collector", anonymous=True)
    data_collector = ImagePublisher()
    rospy.spin()
