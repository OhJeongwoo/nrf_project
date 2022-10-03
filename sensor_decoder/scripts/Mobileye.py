#!/usr/bin/python3
import os
import rospy
import rospkg
import time
import math
import json

from sensor_decoder.msg import Mobileye

class Mobileye:
    def __init__(self, exp_name="test", scenario_num=0):
        self.time_ = 0.0
        self.data_ = None
        self.exp_name_ = exp_name
        self.scenario_num_ = scenario_num

        self.save_dir_ = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + self.exp_name_ + "/" + str(self.scenario_num_).zfill(3) + "/mobileye/"
        if not os.path.exists(self.save_dir_):
            os.mkdir(self.save_dir_)        

        self.sub_mobileye_ = rospy.Subscriber("/mobileye", Mobileye, self.callback_mobileye)
        rospy.loginfo("Initialized Mobileye")


    def reset(self, scenario_num=0):
        self.scenario_num_ = scenario_num
        self.save_dir_ = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + self.exp_name_ + "/" + str(self.scenario_num_).zfill(3) + "/mobileye/"
        if not os.path.exists(self.save_dir_):
            os.mkdir(self.save_dir_)  


    def callback_mobileye(self, msg):
        self.time_ = time.time()
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
        rt_lanes = []
        for lane in cur_lanes:
            rt_lanes.append({'c0':lane.c[0], 'c1':lane.c[1], 'c2':lane.c[2], 'c3':lane.c[3]})
        self.data_ = rt_lanes


    def is_alive(self):
        if time.time() - self.time_ > 0.2:
            return False
        return True


    def get(self):
        if self.data_ is None:
            rospy.logwarn("No Mobileye")
        if time.time() - self.time_ > 0.2:
            rospy.logwarn("Disconnected Mobileye")
        
        return self.data_


    def save(self, seq):
        file_name = self.save_dir_ + str(seq).zfill(6) + ".json"
        with open(file_name, 'w') as jf:
            json.dump(self.data_, jf, indent=4)