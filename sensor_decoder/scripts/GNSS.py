#!/usr/bin/python3
import os
import rospy
import rospkg
import time
import math
import json

from inertiallabs_msgs.msg import sensor_data, gps_data

class GNSS:
    def __init__(self, exp_name="test", scenario_num=0):
        self.gps_time_ = 0.0
        self.imu_time_ = 0.0
        self.data_ = {'gps': None, 'imu': None}
        self.exp_name_ = exp_name
        self.scenario_num_ = scenario_num

        self.save_dir_ = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + self.exp_name_ + "/" + str(self.scenario_num_).zfill(3) + "/gnss/"
        if not os.path.exists(self.save_dir_):
            os.mkdir(self.save_dir_)        

        self.sub_gps_ = rospy.Subscriber("/Inertial_Labs/gps_data", gps_data, self.callback_gps)
        self.sub_imu_ = rospy.Subscriber("/Inertial_Labs/sensor_data", sensor_data, self.callback_imu)
        rospy.loginfo("Initialized GNSS")

    def reset(self, scenario_num=0):
        self.scenario_num_ = scenario_num
        self.save_dir_ = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + self.exp_name_ + "/" + str(self.scenario_num_).zfill(3) + "/gnss/"
        if not os.path.exists(self.save_dir_):
            os.mkdir(self.save_dir_)        

    def callback_gps(self, msg):
        self.gps_time_ = time.time()
        rt = {}
        rt['x'] = msg.LLH.x
        rt['y'] = msg.LLH.y
        rt['v'] = (msg.HorSpeed ** 2 + msg.VerSpeed ** 2) ** 0.5
        rt['theta'] = math.pi / 180.0 * msg.SpeedDir
        self.data_['gps'] = rt

    
    def callback_imu(self, msg):
        self.imu_time_ = time.time()
        rt = {}
        rt['ax'] = msg.Accel.x
        rt['ay'] = msg.Accel.y
        self.data_['imu'] = rt
    

    def is_alive(self):
        if time.time() - min(self.gps_time_, self.imu_time_) > 0.2:
            return False
        return True


    def get(self):
        if self.data_['gps'] is None or self.data_['imu'] is None:
            rospy.logwarn("No GNSS")
        if time.time() - self.gps_time_ > 0.2 or time.time() - self.imu_time_ > 0.2:
            rospy.logwarn("Disconnected GNSS")
        
        return self.data_


    def save(self, seq):
        file_name = self.save_dir_ + str(seq).zfill(6) + ".json"
        with open(file_name, 'w') as jf:
            json.dump(self.data_, jf, indent=4)