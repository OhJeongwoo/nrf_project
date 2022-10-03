#!/usr/bin/python3
import os
import rospy
import rospkg
import time
import pypcd
import numpy as np

from sensor_msgs.msg import PointCloud2

class LiDAR:
    def __init__(self, exp_name="test", scenario_num=0):
        self.time_ = 0.0
        self.data_ = None
        self.exp_name_ = exp_name
        self.scenario_num_ = scenario_num

        self.save_dir_ = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + self.exp_name_ +  "/" + str(self.scenario_num_).zfill(3) + "/lidar/"
        if not os.path.exists(self.save_dir_):
            os.mkdir(self.save_dir_)

        self.sub_lidar_ = rospy.Subscriber("/points_raw", PointCloud2, self.callback_lidar)
        rospy.loginfo("Initialized LiDAR")


    def reset(self, scenario_num=0):
        self.scenario_num_ = scenario_num
        self.save_dir_ = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + self.exp_name_ + "/" + str(self.scenario_num_).zfill(3) + "/lidar/"
        if not os.path.exists(self.save_dir_):
            os.mkdir(self.save_dir_)  


    def callback_lidar(self, msg):
        self.time_ = time.time()
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
        self.data_ = arr.astype('float32')


    def is_alive(self):
        if time.time() - self.time_ > 0.2:
            return False
        return True


    def get(self):
        if self.data_ is None:
            rospy.logwarn("No LiDAR")
        if time.time() - self.time_ > 0.2:
            rospy.logwarn("Disconnected LiDAR")
        
        return self.data_


    def save(self, seq):
        file_name = self.save_dir_ + str(seq).zfill(6) + ".bin"
        self.data_.tofile(file_name)
