#!/usr/bin/python3
import os
import rospy
import rospkg
import time
import math
import json

from can_data_msgs.msg import Car_ctrl_input

class Control:
    def __init__(self, exp_name="test"):
        self.control_time_ = 0.0        

        self.pub_ = rospy.Publisher("/car_ctrl_input", Car_ctrl_input, queue_size=10)
        rospy.loginfo("Initialized Controller")


    def act(self, action):
        msg = Car_ctrl_input()
        msg.acceleration = action[0]
        msg.steering_wheel_angle = action[1]
        self.pub_.publish(msg)