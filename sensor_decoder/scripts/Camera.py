#!/usr/bin/python3
import rospy
import time

from sensor_msgs.msg import Image

class Camera:
    def __init__(self):
        self.time_ = 0.0
        self.data_ = None

        self.sub_camera_ = rospy.Subscriber("/camera/color/image_raw", Image, self.callback_camera)
        rospy.loginfo("Initialized Camera")

    def callback_camera(self, msg):
        self.time_ = time.time()
        self.data_ = msg

    def is_alive(self):
        if time.time() - self.time_ > 0.2:
            return False
        return True

    def get(self):
        if self.data_ is None:
            rospy.logwarn("No Camera")
        if time.time() - self.time_ > 0.2:
            rospy.logwarn("Disconnected Camera")
        
        return self.data_