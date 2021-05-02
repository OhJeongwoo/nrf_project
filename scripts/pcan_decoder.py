#!/usr/bin/python
import rospy
import time

from nrf_project.msg import PCAN
from nrf_project.msg import PCANArray
from nrf_project.msg import Mobileye
from nrf_project.msg import LaneMsg
from nrf_project.msg import ObstacleMsg

import os
import json
import datetime
import utils
from message import Message
from data import Data

def decode_pcan_message(header, msg):
    rt = Message()

    rt.seq = header.seq
    rt.time = header.stamp.to_sec()
    rt.type = msg.type
    rt.size = msg.size

    data = []
    for i in range(msg.size):
        d = []
        tmp = msg.data[i]
        for j in range(8):
            d.append(tmp % 2)
            tmp = tmp / 2
        data.append(d)
    rt.data = data

    return rt

def decode_pcan_array(msg):
    messages = []
    header = msg.header
    for message in msg.messages:
        messages.append(decode_pcan_message(header, message))

    return msg.header.seq, msg.header.stamp.to_sec(), messages

def generate_message(data):
    rt = Mobileye()

    rt.header.seq = data.seq
    rt.header.stamp = rospy.Time.from_sec(data.time)

    rt.valid = True

    rt.left_lane = data.left_lane.generate_message()
    rt.right_lane = data.right_lane.generate_message()
    rt.n_next_lanes = data.n_next_lanes
    next_lanes = []
    for i in range(data.n_next_lanes):
        next_lanes.append(data.next_lanes[i].generate_message())
    rt.next_lanes = next_lanes

    rt.n_obstacles = data.n_obstacles
    obstacles = []
    for i in range(data.n_obstacles):
        obstacles.append(data.obstacles[i].generate_message())
    rt.obstacles = obstacles

    return rt

class PCANDecoder:
    def __init__(self):
        self.sub_ = rospy.Subscriber("/pcan_data", PCANArray, self.callback)
        self.pub_ = rospy.Publisher("/mobileye", Mobileye, queue_size=20)
        self.valid = True
        self.left_lane_valid = True
        self.right_lane_valid = True
        self.next_lane_valid = True
        self.next_lane_valid_array = []
        self.obstacle_status_valid = True
        self.obstacle_valid = True
        self.obstacle_valid_array = []
        self.message_stack = []
        self.seq = 0
        self.time = 0.0


    def callback(self, raw_msg):
        """
        callback function for PCAN raw data

        step 1. if msg type is 0x700, reset data
        step 2. stack received msg
        step 3. if msg type is 0x728, publish data
        """
        self.seq, self.time, self.message_stack = decode_pcan_array(raw_msg)
        
        self.publish()



    def publish(self):
        rt = Mobileye()
        data = Data(self.seq, self.time, self.message_stack)
        valid = data.decode_data()

        if not valid:
            rt.valid = False
        else:
            rt = generate_message(data)
        
        self.pub_.publish(rt)
        
        

            




if __name__=='__main__':
    rospy.init_node("pcan_decoder", anonymous=True)
    pcan_decoder = PCANDecoder()
    rospy.spin()
