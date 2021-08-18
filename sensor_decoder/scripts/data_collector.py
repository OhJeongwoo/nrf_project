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

def clip(value, limit):
    """
    clip value by given limit and rescale to [-1,1]
    """
    if abs(value) > abs(limit):
        return value / abs(value)
    else:
        return value / limit

    
def get_x(lane, z):
    return lane.c[0] + z * lane.c[1] + z * z * lane.c[2] + z * z * z * lane.c[3]

class DataCollector:
    def __init__(self):
        self.init_gps = False
        self.init_imu = False
        self.init_mobileye = False
        self.init_camera = False
        self.init_lidar = False

        self.bridge = CvBridge()

        self.seq = 0
        self.author = "jeongwoooh"
        self.email = "jeongwoo.oh@rllab.snu.ac.kr"
        self.copy_right = "RLLAB@SNU"
        self.date = datetime.datetime.now().strftime("%Y-%m-%d")
        self.data_name = "0815_exp_jeongwoo_highway_1"
        self.data_path = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + self.data_name + "/"
        # self.data_path = "/media/jeongwoooh/Samsung USB/data/" + self.data_name + "/"

        self.state_path = self.data_path + "state/"
        self.image_raw_path = self.data_path + "image_raw/"
        self.bin_path = self.data_path + "bin/"
        self.pcd_path = self.data_path + "pcd/"
        self.bev_map_path = self.data_path + "bev_map/"
        self.local_map_path = self.data_path + "local_map/"
        self.object_path = self.data_path + "object/"

        self.lanes = []
        self.decision = KEEPING_LANE
        self.lane_decision = KEEPING_LANE
        self.merging_threshold = 1.0
        self.current_deviation = 0.0
        self.target_deviation = 0.0
        self.default_lane_width = 3.3

        self.n_marked_lane = 500
        self.v_clip = 20.0
        self.ax_clip = 5.0
        self.ay_clip = 1.0
        self.w_clip = 0.5
        self.dev_clip = 4.0
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.fontscale = 0.5
        self.fontthickness = 2
        self.fontline = cv2.LINE_AA

        self.minX = -20.0
        self.maxX = 40.0
        self.minY = -30.0
        self.maxY = 30.0
        self.minZ = -2.73
        self.maxZ = 1.27
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
        if not os.path.exists(self.object_path):
            os.mkdir(self.object_path)

        self.gps_time_threshold = 0.2
        self.imu_time_threshold = 0.2
        self.camera_time_threshold = 0.2
        self.mobileye_time_threshold = 0.2
        self.lidar_time_threshold = 0.2

        self.sub_gps = rospy.Subscriber("/Inertial_Labs/gps_data", gps_data, self.callback_gps)
        self.sub_imu = rospy.Subscriber("/Inertial_Labs/sensor_data", sensor_data, self.callback_imu)
        self.sub_mobileye = rospy.Subscriber("/mobileye", Mobileye, self.callback_mobileye)
        self.sub_decision = rospy.Subscriber("/decision", Int32, self.callback_decision)
        self.sub_camera = rospy.Subscriber("/camera/color/image_raw", Image, self.callback_camera)
        # self.sub_camera = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback_camera)
        self.sub_lidar = rospy.Subscriber("/points_raw", PointCloud2, self.callback_lidar)
        self.pub_local_map = rospy.Publisher("/local_map", Image)


    def xy_to_pixel(self, x, y):
        # px = int((x - self.cx) / self.resolution + self.bev_width / 2)
        # py = int(-(y - self.cy) / self.resolution + self.bev_height / 2)
        px = int((y-self.minY)/self.resolution)
        py = int(-(x-self.maxX)/self.resolution)
        return px, py

    def get_decision(self, type):
        if type == 1:
            return "KEEP LANE"
        if type == 2:
            return "CHANGE LEFT"
        if type == 3:
            return "CHANGE RIGHT"
        if type == 4:
            return "STOP"
        return "ERROR"

    def save_state(self):
        """
        save state to .json file
        return state dictionary
        """
        print("# : ", self.seq)
        print("CURRENT STATE : ", self.get_decision(self.lane_decision))
        print("x     : ", self.x)
        print("y     : ", self.y)
        print("v     : ", self.v)
        print("ax    : ", self.ax)
        print("ay    : ", self.ay)
        print("theta : ", self.theta)
        print("omega : ", self.omega)

        rt = {}
        rt['x'] = self.x
        rt['y'] = self.y
        rt['v'] = self.v
        rt['ax'] = self.ax
        rt['ay'] = self.ay
        rt['theta'] = self.theta
        rt['omega'] = self.omega
        rt['decision'] = self.lane_decision
        # rt['current_deviation'] = self.current_deviation
        # rt['target_deviation'] = self.target_deviation

        rt_lanes = []
        for lane in self.lanes:
            rt_lanes.append({'c0':lane.c[0], 'c1':lane.c[1], 'c2':lane.c[2], 'c3':lane.c[3]})
        rt['lanes'] = rt_lanes

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

    def draw_lanes(self):
        for lane in self.lanes:
            for i in range(self.n_marked_lane):
                cv2.circle(self.local_map, (self.xy_to_pixel(0.1*i, get_x(lane,0.1*i))), 1, (0,255,0), -1)
            

    def callback_gps(self, msg):
        """
        update x, y, v, theta
        """
        if not self.init_gps:
            self.gps_time = time.time()
            rospy.loginfo("Start to receive GPS data")
        else :
            if time.time() - self.gps_time > self.gps_time_threshold :
                rospy.logwarn("No GPS")
            self.gps_time = time.time()
        self.init_gps = True

        self.x = msg.LLH.x
        self.y = msg.LLH.y
        self.v = math.sqrt(msg.HorSpeed ** 2 + msg.VerSpeed ** 2)
        self.theta = math.pi / 180.0 * msg.SpeedDir


    def callback_imu(self, msg):
        """
        update ax, ay, omega
        """
        if not self.init_imu:
            self.imu_time = time.time()
            rospy.loginfo("Start to receive IMU data")
        else :
            if time.time() - self.imu_time > self.imu_time_threshold :
                rospy.logwarn("No IMU")
            self.imu_time = time.time()
        self.init_imu = True

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
        if not self.init_mobileye:
            self.mobileye_time = time.time()
            rospy.loginfo("Start to receive MOBILEYE data")
        else :
            if time.time() - self.mobileye_time > self.mobileye_time_threshold :
                rospy.logwarn("No MOBILEYE")
            self.mobileye_time = time.time()
        self.init_mobileye = True

        # calculate offset
        offset = 0
        if self.lane_decision != self.decision:
            if self.decision == CHANGING_LEFT:
                offset = -1
            if self.decision == CHANGING_RIGHT:
                offset = 1
        self.lane_decision = self.decision

        # decode raw lane deviation data
        # self.current_deviation = (msg.left_lane.c[0] + msg.right_lane.c[0]) / 2.0
        # raw_data = []
        cur_lanes = []
        if msg.left_lane is not None:
            # raw_data.append(msg.left_lane.c[0])
            cur_lanes.append(msg.left_lane)
        if msg.right_lane is not None:
            # raw_data.append(msg.right_lane.c[0])
            cur_lanes.append(msg.right_lane)
        for i in range(msg.n_next_lanes):
            # raw_data.append(msg.next_lanes[i].c[0])
            cur_lanes.append(msg.next_lanes[i])
        self.lanes = cur_lanes
        
        # raw_data.sort()

        # merge double lane
        # data = []
        # index = 0
        # while True:
        #     if index == len(raw_data):
        #         break
        #     x = raw_data[index]
        #     k = 1
        #     while True:
        #         if index + k == len(raw_data):
        #             break
        #         if raw_data[index+k] - raw_data[index] > self.merging_threshold:
        #             break
            
        #     data.append(sum(raw_data[index:index+k])/k)

        #     index = index + k
        
        # if only few lanes are detected, keep value
        # if len(data) < 2:
        #     rospy.logwarn("only few lanes are detected!")
        #     return


        # calculate lateral deviation
        # dev = [(data[i]+data[i+1])/2 for i in range(0, len(data)-1)]

        # nearest = 0
        # value = 1e3
        # for i in range(len(dev)):
        #     if value > abs(self.target_deviation - dev[i]):
        #         value = abs(self.target_deviation - dev[i])
        #         nearest = i
        
        # target = nearest + offset
        # if target < 0 or target >= len(dev):
        #     self.target_deviation = dev[nearest] + offset * self.default_lane_width
        #     return

        # self.target_deviation = dev[target]
        # return

        
    def callback_decision(self, msg):
        """
        update decision
        """
        self.decision = msg.data


    def callback_camera(self, msg):
        """
        update scene
        """
        if not self.init_camera:
            self.camera_time = time.time()
            rospy.loginfo("Start to receive CAMERA data")
        else :
            if time.time() - self.camera_time > self.camera_time_threshold :
                rospy.logwarn("No CAMERA")
            self.camera_time = time.time()
        self.init_camera = True

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
        if not self.init_lidar:
            self.lidar_time = time.time()
            rospy.loginfo("Start to receive LIDAR data")
        else :
            if time.time() - self.lidar_time > self.lidar_time_threshold :
                rospy.logwarn("No LIDAR")
            self.lidar_time = time.time()
        self.init_lidar = True
        if not (self.init_gps and self.init_imu and self.init_mobileye and self.init_camera):
            return
        
        print("[DELAY]")
        print("GPS : ", self.lidar_time - self.gps_time)
        print("IMU : ", self.lidar_time - self.imu_time)
        print("MOB : ", self.lidar_time - self.mobileye_time)
        print("CAM : ", self.lidar_time - self.camera_time)

        if self.lidar_time - self.gps_time > 2*self.gps_time_threshold :
            rospy.logwarn("Large delay (GPS)")
        if self.lidar_time - self.imu_time > 2*self.imu_time_threshold :
            rospy.logwarn("Large delay (IMU)")
        if self.lidar_time - self.mobileye_time > 2*self.mobileye_time_threshold :
            rospy.logwarn("Large delay (MOB)")
        if self.lidar_time - self.camera_time > 2*self.camera_time_threshold :
            rospy.logwarn("Large delay (CAM)")

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
        # pc.save_pcd(self.pcd_path + str(self.seq).zfill(6) + '.pcd', compression='binary_compressed')
        
        # generate bev map with pointclouds and save it
        # filter point cloud
        # x_filter = np.logical_and((x>self.minX), (x<self.maxX))
        # y_filter = np.logical_and((y>self.minY), (y<self.maxY))
        # z_filter = np.logical_and((z>self.minZ), (z<self.maxZ))
        # filter = np.logical_and(x_filter, y_filter, z_filter)
        # indices = np.argwhere(filter).flatten()
        # x = x[indices]
        # y = y[indices]
        # z = z[indices]
        # intensity = intensity[indices]

        # N = len(intensity)

        # intensity_layer = np.zeros([self.bev_width, self.bev_height], dtype=np.float)
        # density_layer = np.zeros([self.bev_width, self.bev_height], dtype=np.float)
        # height_layer = np.zeros([self.bev_width, self.bev_height], dtype=np.float)

        # for i in range(N):
        #     py, px = self.xy_to_pixel(x[i], -y[i])
        #     if px < 0 or px >= self.bev_width or py <0 or py >= self.bev_height :
        #         continue
        #     intensity_layer[px][py] = max(intensity_layer[px][py], intensity[i])
        #     density_layer[px][py] += 1
        #     height_layer[px][py] = max(height_layer[px][py], (z[i]-self.minZ)/(self.maxZ-self.minZ))
        
        # for i in range(self.bev_width):
        #     for j in range(self.bev_height):
        #         density_layer[i][j] = min(1.0, math.log(1 + density_layer[i][j]) / self.logDensity)

        # intensity_layer = intensity_layer * 255.0
        # density_layer = density_layer * 255.0
        # height_layer = height_layer * 255.0

        # intensity_layer = np.expand_dims(intensity_layer.astype('uint8'), axis = 0)
        # density_layer = np.expand_dims(density_layer.astype('uint8'), axis = 0)
        # height_layer = np.expand_dims(height_layer.astype('uint8'), axis = 0)
        
        # bev_map = np.transpose(np.vstack((intensity_layer, density_layer, height_layer)), (1,2,0))
        # bev_map = cv2.resize(bev_map, (self.bev_width, self.bev_height))
        # cv2.imwrite(self.bev_map_path + str(self.seq).zfill(6) + ".png", bev_map)

        # # generate local map and save it
        # self.local_map = bev_map

        # # draw lanes
        # self.draw_lanes()

        # # draw heading indicator
        # cv2.circle(self.local_map, (100, 100), 50, (255,255,255), 2)
        # cv2.line(self.local_map, (100,100), (int(100+30.0*math.cos(state['theta'])),int(100+30.0*math.sin(state['theta']))), (255, 0, 0), 1)
        
        # # draw box for v, ax, ay, omega, lateral deviation indicator
        # cv2.putText(self.local_map, "v", (20, 200), self.font, self.fontscale, (255, 255, 255), self.fontthickness, self.fontline)
        # cv2.putText(self.local_map, "ax", (20, 240), self.font, self.fontscale, (255, 255, 255), self.fontthickness, self.fontline)
        # cv2.putText(self.local_map, "ay", (20, 280), self.font, self.fontscale, (255, 255, 255), self.fontthickness, self.fontline)
        # cv2.putText(self.local_map, "w", (20, 320), self.font, self.fontscale, (255, 255, 255), self.fontthickness, self.fontline)
        # cv2.putText(self.local_map, "dev", (20, 360), self.font, self.fontscale, (255, 255, 255), self.fontthickness, self.fontline)

        # cv2.rectangle(self.local_map, (50, 180), (150, 200), (255, 255, 255), 2)
        # cv2.rectangle(self.local_map, (50, 220), (150, 240), (255, 255, 255), 2)
        # cv2.rectangle(self.local_map, (50, 260), (150, 280), (255, 255, 255), 2)
        # cv2.rectangle(self.local_map, (50, 300), (150, 320), (255, 255, 255), 2)
        # cv2.rectangle(self.local_map, (50, 340), (150, 360), (255, 255, 255), 2)

        # cv2.line(self.local_map, (int(100 + 50 * clip(state['v'], self.v_clip)), 180), (int(100 + 50 * clip(state['v'], self.v_clip)), 200), (255, 0, 0), 2)
        # cv2.line(self.local_map, (int(100 + 50 * clip(state['ax'], self.ax_clip)), 220), (int(100 + 50 * clip(state['ax'], self.ax_clip)), 240), (255, 0, 0), 2)
        # cv2.line(self.local_map, (int(100 + 50 * clip(state['ay'], self.ay_clip)), 260), (int(100 + 50 * clip(state['ay'], self.ay_clip)), 280), (255, 0, 0), 2)
        # cv2.line(self.local_map, (int(100 + 50 * clip(state['omega'], self.w_clip)), 300), (int(100 + 50 * clip(state['omega'], self.w_clip)), 320), (255, 0, 0), 2)
        # cv2.line(self.local_map, (int(100 + 50 * clip(state['target_deviation'], self.dev_clip)), 340), (int(100 + 50 * clip(state['target_deviation'], self.dev_clip)), 360), (255, 0, 0), 2)

        # # write state info on the right side (x, y, theta, v, ax, ay, omega, decision, current deviation, target deviation)
        # cv2.putText(self.local_map, "lat      : " + str(state['x']), (400, 50), self.font, self.fontscale, (255, 255, 255), self.fontthickness, self.fontline)
        # cv2.putText(self.local_map, "lng      : " + str(state['y']), (400, 80), self.font, self.fontscale, (255, 255, 255), self.fontthickness, self.fontline)
        # cv2.putText(self.local_map, "theta    : " + str(state['theta']), (400, 110), self.font, self.fontscale, (255, 255, 255), self.fontthickness, self.fontline)
        # cv2.putText(self.local_map, "vel      : " + str(state['v']), (400, 140), self.font, self.fontscale, (255, 255, 255), self.fontthickness, self.fontline)
        # cv2.putText(self.local_map, "ax       : " + str(state['ax']), (400, 170), self.font, self.fontscale, (255, 255, 255), self.fontthickness, self.fontline)
        # cv2.putText(self.local_map, "ay       : " + str(state['ay']), (400, 200), self.font, self.fontscale, (255, 255, 255), self.fontthickness, self.fontline)
        # cv2.putText(self.local_map, "omega    : " + str(state['omega']), (400, 230), self.font, self.fontscale, (255, 255, 255), self.fontthickness, self.fontline)
        # cv2.putText(self.local_map, "dev      : " + str(state['target_deviation']), (400, 260), self.font, self.fontscale, (255, 255, 255), self.fontthickness, self.fontline)
        # cv2.putText(self.local_map, "decision : " + self.get_decision(state['decision']), (400, 290), self.font, self.fontscale, (255, 255, 255), self.fontthickness, self.fontline)


        # # write data info on the left below corner (seq, data name, date)
        # cv2.putText(self.local_map, "seq       : " + str(state['seq']), (30, 500), self.font, self.fontscale, (255, 255, 255), self.fontthickness, self.fontline)
        # cv2.putText(self.local_map, "data      : " + self.data_name, (30, 530), self.font, self.fontscale, (255, 255, 255), self.fontthickness, self.fontline)
        # cv2.putText(self.local_map, "date      : " + self.date, (30, 560), self.font, self.fontscale, (255, 255, 255), self.fontthickness, self.fontline)
        # cv2.putText(self.local_map, "copyright : " + self.copy_right, (30, 590), self.font, self.fontscale, (255, 255, 255), self.fontthickness, self.fontline)

        # cv2.imwrite(self.local_map_path + str(self.seq).zfill(6) + ".png", self.local_map)

        # print log in the terminal
        print("============================")
        # for key, value in state.items():
        #     print(key, ' : ', value)


    

if __name__=='__main__':
    rospy.init_node("data_collector", anonymous=True)
    data_collector = DataCollector()
    rospy.spin()
