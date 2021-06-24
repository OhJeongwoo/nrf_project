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
from inertiallabs_msgs.msg import sensor_data
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32

import json
import datetime
import utils
from message import Message
from data import Data

import cv2
from cv_bridge import CvBridge


KEEPING_LANE = 1
CHANGING_LEFT = 2
CHANGING_RIGHT = 3
STOP = 4

def norm(v):
    return math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)

class DataCollector:
    def __init__(self):
        self.sub_gps = rospy.Subscriber("/Inertial_Labs/ins_data", ins_data, self.callback_gps)
        self.sub_imu = rospy.Subscriber("/Inertial_Labs/sensor_data", sensor_data, self.callback_imu)
        self.sub_mobileye = rospy.Subscriber("/mobileye", Mobileye, self.callback_mobileye)
        self.sub_decision = rospy.Subscriber("/decision", Int32, self.callback_decision)
        self.sub_camera = rospy.Subscriber("/camera/color/image_raw", Image, self.callback_camera)
        self.sub_lidar = rospy.Subscriber("/points_raw", PointCloud2, self.callback_lidar)
        self.pub_local_map = rospy.Publisher("/local_map", Image)

        self.bridge = CvBridge()

        self.seq = 0
        self.author = "jeongwoooh"
        self.email = "jeongwoo.oh@rllab.snu.ac.kr"
        self.copy_right = "RLLAB@SNU"
        self.date = datetime.datetime.now().strftime("%Y-%m-%d")
        self.data_name = "demo"
        self.data_path = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + self.data_name + "/"
        
        self.state_path = self.data_path + "state/"
        self.image_raw_path = self.data_path + "image_raw/"
        self.bin_path = self.data_path + "bin/"
        self.pcd_path = self.data_path + "pcd/"
        self.bev_map_path = self.data_path + "bev_map/"
        self.local_map_path = self.data_path + "local_map/"
        
        self.merging_threshold = 1.0

        self.decision = KEEPING_LANE
        self.lane_decision = KEEPING_LANE
        self.lateral_deviation = 0.0
        self.default_lane_width = 3.3

        self.minX = 0.0
        self.maxX = 0.0
        self.minY = 0.0
        self.maxY = 0.0
        self.minZ = 0.0
        self.maxZ = 0.0
        self.resolution = 0.1
        self.cx = (self.minX + self.maxX) / 2.0
        self.cy = (self.minY + self.maxY) / 2.0
        self.bev_width = int((self.maxX - self.minX) / self.resolution)
        self.bev_height = int((self.maxY - self.minY) / self.resolution)
        self.logDensity = math.log(64)

        if not os.path.exists(self.data_path):
            os.mkdir(self.data_path)
        if not os.path.exists(self.state_path):
            os.mkdir(self.state_path)
        if not os.path.exists(self.image_raw_path):
            os.mkdir(self.image_raw_path)
        if not os.path.exists(self.bin_path):
            os.mkdir(self.bin_path)
        if not os.path.exists(self.pcd_path):
            os.mkdir(self.pcd_path)
        if not os.path.exists(self.bev_map_path):
            os.mkdir(self.bev_map_path)
        if not os.path.exists(self.local_map_path):
            os.mkdir(self.local_map_path)

    def xy_to_pixel(self, x, y):
        px = int((x - self.cx) / self.resolution + self.bev_width / 2)
        py = int((y - self.cy) / self.resolution + self.bev_height / 2)
        return px, py

    def callback_gps(self, msg):
        """
        update x, y, v, theta
        """
        self.x = msg.LLH.x
        self.y = msg.LLH.y
        self.v = norm(msg.Vel_ENU)
        self.theta = math.pi / 180.0 * msg.YPR.x


    def callback_imu(self, msg):
        """
        update ax, ay, omega
        """
        self.ax = msg.Accel.x
        self.ay = msg.Accel.y
        self.omega = math.pi / 180.0 * msg.Gyro.z



    def callback_mobileye(self, msg):
        """
        update lane lateral deviation
        1. decode lane info from msg
        2. sorting by increasing order
        3. merging double lanes
        4. calculate lateral deviation for each lane
        5. select lateral deviation close to previous value
        6. return value by decision signal
        """
        # calculate offset
        offset = 0
        if self.lane_decision != self.decision:
            if self.decision == CHANGING_LEFT:
                offset = -1
            if self.decision == CHANGING_RIGHT:
                offset = 1
        self.lane_decision = self.decision

        # decode raw lane deviation data
        raw_data = []
        if msg.left_lane is not None:
            raw_data.append(msg.left_lane.c[0])
        if msg.right_lane is not None:
            raw_data.append(msg.right_lane.c[0])
        for i in range(msg.n_next_lanes):
            raw_data.append(msg.next_lanes[i].c[0])
        
        raw_data.sort()

        # merge double lane
        data = []
        index = 0
        while True:
            if index == len(raw_data):
                break
            x = raw_data[index]
            k = 1
            while True:
                if index + k == len(raw_data):
                    break
                if raw_data[index+k] - raw_data[index] > self.merging_threshold:
                    break
            
            data.append(sum(raw_data[index:index+k])/k)

            index = index + k
        
        # if only few lanes are detected, keep value
        if len(data) < 2:
            rospy.logwarn("only few lanes are detected!")
            return


        # calculate lateral deviation
        dev = [(data[i]+data[i+1])/2 for i in range(0, len(data)-1)]

        nearest = 0
        value = 1e3
        for i in range(len(dev)):
            if value > abs(self.lateral_deviation - dev[i]):
                value = abs(self.lateral_deviation - dev[i])
                nearest = i
        
        target = nearest + offset
        if target < 0 or target >= len(dev):
            self.lateral_deviation = dev[nearest] + offset * self.default_lane_width
            return

        self.lateral_deviation = dev[target]
        return

        
    def callback_decision(self, msg):
        """
        update decision
        """
        self.decision = msg.data


    def callback_camera(self, msg):
        """
        update scene
        """
        self.image_msg = msg


    def callback_lidar(self, msg):
        """
        lidar msg callback function
        1. save current state
        2. save camera image
        3. save raw pointclouds to .pcd and .bin format
        4. generate bird eye view(bev) map with pointclouds and save it
        5. generate local map and save it
        """
        self.seq += 1
        # save current state
        state = self.save_state()

        # save camera image
        cv2.imwrite(self.image_raw_path + str(self.seq).zfill(6) + ".png", self.bridge.imgmsg_to_cv2(self.image_msg, "bgr8"))

        # save raw pointclouds to .pcd and .bin format
        pc = pypcd.PointCloud.from_msg(msg)
        x = pc.pc_data['x']
        y = pc.pc_data['y']
        z = pc.pc_data['z']
        intensity = pc.pc_data['intensity'] / 255.0
        
        arr = np.zeros(x.shape[0] + y.shape[0] + z.shape[0] + intensity.shape[0], dtype=np.float32)
        arr[::4] = x
        arr[1::4] = y
        arr[2::4] = z
        arr[3::4] = intensity
        arr.astype('float32').tofile(self.bin_path + str(self.seq).zfill(6) + '.bin')
        pc.save_pcd(self.pcd_path + str(self.seq).zfill(6) + '.pcd', compression='binary_compressed')
        
        # generate bev map with pointclouds and save it
        # filter point cloud
        x_filter = np.logical_and((x>self.minX), (x<self.maxX))
        y_filter = np.logical_and((y>self.minY), (y<self.maxY))
        z_filter = np.logical_and((z>self.minZ), (z<self.maxZ))
        filter = np.logical_and(x_filter, y_filter, z_filter)
        indices = np.argwhere(filter).flatten()
        x = x[indices]
        y = y[indices]
        z = z[indices]
        intensity = intensity[indices]

        N = len(intensity)

        intensity_layer = np.zeros([self.bev_height, self.bev_width], dtype=np.float)
        density_layer = np.zeros([self.bev_height, self.bev_width], dtype=np.float)
        height_layer = np.zeros([self.bev_height, self.bev_width], dtype=np.float)

        for i in range(N):
            px, py = self.xy_to_pixel(x[i], y[i])
            intensity_layer[px][py] = max(intensity_layer[px][py], intensity[i])
            density_layer[px][py] += 1
            height_layer = max(height_layer[px][py], (z[i]-self.minZ)/(self.maxZ-self.minZ))
        
        density_layer = min(1.0, math.log(1 + density_layer) / self.logDensity)

        intensity_layer = intensity_layer * 255.0
        density_layer = density_layer * 255.0
        height_layer = height_layer * 255.0

        intensity_layer = np.expand_dims(intensity_layer.astype('uint8'), axis = 0)
        density_layer = np.expand_dims(density_layer.astype('uint8'), axis = 0)
        height_layer = np.expand_dims(height_layer.astype('uint8'), axis = 0)
        
        bev_map = np.transpose(np.vstack((intensity_layer, density_layer, height_layer)), (1,2,0))
        bev_map = cv2.resize(bev_map, (self.bev_height, self.bev_width))
        cv2.imwrite(self.bev_map_path + str(self.seq).zfill(6) + ".png", bev_map)

        # generate local map and save it



        # print log in the terminal


    def save_state(self):
        """
        save state to .json file
        return state dictionary
        """
        rt = {}
        rt['x'] = self.x
        rt['y'] = self.y
        rt['v'] = self.v
        rt['ax'] = self.ax
        rt['ay'] = self.ay
        rt['theta'] = self.theta
        rt['omega'] = self.omega
        rt['decision'] = self.lane_decision
        rt['lateral_deviation'] = self.lateral_deviation

        rt['seq'] = self.seq
        rt['author'] = self.author
        rt['email'] = self.email
        rt['copy_right'] = self.copy_right
        rt['date'] = self.date
        rt['data_name'] = self.data_name

        save_path = self.state_path + str(self.seq).zfill(6) + ".json"
        with open(save_path, 'w') as outfile:
            json.dump(rt, outfile, indent=4)

        return rt
