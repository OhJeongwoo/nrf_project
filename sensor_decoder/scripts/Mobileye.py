#!/usr/bin/python3
import rospy
import time

from sensor_decoder.msg import Mobileye

class Mobileye:
    def __init__(self):
        self.time_ = 0.0
        self.data_ = None

        self.sub_mobileye_ = rospy.Subscriber("/mobileye", Mobileye, self.callback_mobileye)
        rospy.loginfo("Initialized Mobileye")

    def callback_mobileye(self, msg):
        self.time_ = time.time()
        self.data_ = msg

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