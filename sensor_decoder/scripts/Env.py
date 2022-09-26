#!/usr/bin/python3
import rospy
import time

from GNSS import GNSS
from LiDAR import LiDAR
from Mobileye import Mobileye
from Camera import Camera

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
        