#!/usr/bin/python3
import rospy
import time

from inertiallabs_msgs.msg import sensor_data, gps_data

class GNSS:
    def __init__(self):
        self.gps_time_ = 0.0
        self.imu_time_ = 0.0
        self.data_ = {'gps': None, 'imu': None}

        self.sub_gps_ = rospy.Subscriber("/Inertial_Labs/gps_data", gps_data, self.callback_gps)
        self.sub_imu_ = rospy.Subscriber("/Inertial_Labs/sensor_data", sensor_data, self.callback_imu)
        rospy.loginfo("Initialized GNSS")

    def callback_gps(self, msg):
        self.gps_time_ = time.time()
        self.data_['gps'] = msg

    
    def callback_imu(self, msg):
        self.imu_time_ = time.time()
        self.data_['imu'] = msg
    

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