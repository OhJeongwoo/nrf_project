#!/usr/bin/python3
import rospy
import time

from GNSS import GNSS
from LiDAR import LiDAR
from Mobileye import Mobileye
from Camera import Camera
from Control import Control

class Env:
    def __init__(self):
        self.gnss_ = GNSS()
        self.lidar_ = LiDAR()
        self.mobileye_ = Mobileye()
        self.camera_ = Camera()

    
    def get_state(self):
        state = {}
        if self.gnss_.is_alive:
            state['gnss'] = self.gnss_.get()
        else:
            state['gnss'] = None
            rospy.logwarn("GNSS is not alive")

        if self.lidar_.is_alive:
            state['lidar'] = self.lidar_.get()
        else:
            state['lidar'] = None
            rospy.logwarn("LiDAR is not alive")

        if self.mobileye_.is_alive:
            state['mobileye'] = self.mobileye_.get()
        else:
            state['mobileye'] = None
            rospy.logwarn("Mobileye is not alive")

        if self.camera_.is_alive:
            state['camera'] = self.camera_.get()
        else:
            state['camera'] = None
            rospy.logwarn("Camera is not alive")
        

class GunminEnv:
    def __init__(self, exp_name="test"):
        self.exp_name_ = exp_name
        self.scenario_num_ = 0
        self.timestep_ = 0

        self.gnss_ = GNSS(self.exp_name_)
        self.mobileye_ = Mobileye(self.exp_name_)
        self.camera_ = Camera(self.exp_name_)
        self.control_ = Control()

    def reset(self):
        self.scenario_num_ += 1
        self.timestep_ = 0

        self.gnss_.reset(self.scenario_num_)
        self.mobileye_.reset(self.scenario_num_)
        self.camera_.reset(self.scenario_num_)

    
    def get_state(self):
        state = {}
        if self.gnss_.is_alive:
            state['gnss'] = self.gnss_.get()
        else:
            state['gnss'] = None
            rospy.logwarn("GNSS is not alive")

        if self.mobileye_.is_alive:
            state['mobileye'] = self.mobileye_.get()
        else:
            state['mobileye'] = None
            rospy.logwarn("Mobileye is not alive")

        if self.camera_.is_alive:
            state['camera'] = self.camera_.get()
        else:
            state['camera'] = None
            rospy.logwarn("Camera is not alive")
        
        return state


    def act(self, action):
        self.control_.act(action)

            
    def step(self, action):
        self.timestep_ += 1
        self.act(action)

    def save(self):
        self.gnss_.save(self.timestep_)
        self.mobileye_.save(self.timestep_)
        self.camera_.save(self.timestep_)


class TimothyEnv:
    def __init__(self, exp_name="test"):
        self.exp_name_ = exp_name
        self.scenario_num_ = 0
        self.timestep_ = 0

        self.gnss_ = GNSS(self.exp_name_)
        self.mobileye_ = Mobileye(self.exp_name_)
        self.camera_ = Camera(self.exp_name_)
        self.lidar_ = LiDAR(self.exp_name_)

    def reset(self):
        self.scenario_num_ += 1
        self.timestep_ = 0

        self.gnss_.reset(self.scenario_num_)
        self.mobileye_.reset(self.scenario_num_)
        self.camera_.reset(self.scenario_num_)
        self.lidar_.reset(self.scenario_num_)

    
    def get_state(self):
        state = {}
        if self.gnss_.is_alive:
            state['gnss'] = self.gnss_.get()
        else:
            state['gnss'] = None
            rospy.logwarn("GNSS is not alive")

        if self.mobileye_.is_alive:
            state['mobileye'] = self.mobileye_.get()
        else:
            state['mobileye'] = None
            rospy.logwarn("Mobileye is not alive")

        if self.camera_.is_alive:
            state['camera'] = self.camera_.get()
        else:
            state['camera'] = None
            rospy.logwarn("Camera is not alive")

        if self.lidar_.is_alive:
            state['lidar'] = self.lidar_.get()
        else:
            state['lidar'] = None

            rospy.logwarn("LiDAR is not alive")
        
        return state


    def save(self):
        self.gnss_.save(self.timestep_)
        self.mobileye_.save(self.timestep_)
        self.camera_.save(self.timestep_)
        self.lidar_.save(self.timestep_)

            
    def step(self):
        self.timestep_ += 1
        self.save()

