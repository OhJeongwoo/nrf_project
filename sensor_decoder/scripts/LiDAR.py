#!/usr/bin/python3
import rospy
import time

from sensor_msgs.msg import PointCloud2

class LiDAR:
    def __init__(self):
        self.time_ = 0.0
        self.data_ = None

        self.sub_lidar_ = rospy.Subscriber("/points_raw", PointCloud2, self.callback_lidar)
        rospy.loginfo("Initialized LiDAR")

    def callback_lidar(self, msg):
        self.time_ = time.time()
        self.data_ = msg

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