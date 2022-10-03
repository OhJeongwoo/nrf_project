#!/usr/bin/python3
import os
import rospy
import rospkg
import time

import cv2
import numpy as np

class Camera:
    def __init__(self, exp_name="test", scenario_num=0):
        self.time_ = 0.0
        self.data_ = None
        self.exp_name_ = exp_name
        self.scenario_num_ = scenario_num

        self.save_dir_ = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + self.exp_name_ + "/" + str(self.scenario_num_).zfill(3) + "/camera/"
        if not os.path.exists(self.save_dir_):
            os.mkdir(self.save_dir_)

        self.sub_camera_ = cv2.VideoCapture(0)
        rospy.loginfo("Initialized Camera")


    def reset(self, scenario_num=0):
        self.scenario_num_ = scenario_num
        self.save_dir_ = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + self.exp_name_ + "/" + str(self.scenario_num_).zfill(3) + "/camera/"
        if not os.path.exists(self.save_dir_):
            os.mkdir(self.save_dir_)  


    def update(self):
        success, data = self.sub_camera_.read()
        self.time_ = time.time()
        self.data_ = data
        return success


    def is_alive(self):
        if self.update():
            return True
        return False


    def get(self):
        if self.update():
            return self.data
        rospy.logwarn("Error occurs for Camera")
        return None
        

    def save(self, seq):
        if self.update():
            file_name = self.save_dir_ + str(seq).zfill(6) + ".png"
            cv2.imwrite(file_name, self.data)
            return True
        rospy.logwarn("Error occurs for Camera")
        return None
        
