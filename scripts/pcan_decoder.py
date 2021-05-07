#!/usr/bin/python
import rospy
import time
import math

from nrf_project.msg import PCAN
from nrf_project.msg import PCANArray
from nrf_project.msg import Mobileye
from nrf_project.msg import LaneMsg
from nrf_project.msg import ObstacleMsg
from inertiallabs_msgs.msg import ins_data
from inertiallabs_msgs.msg import sensor_data
from geometry_msgs.msg import Vector3

import os
import json
import datetime
import utils
from message import Message
from data import Data

def norm(v):
    return math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)

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

    rt.x = data.x
    rt.y = data.y
    rt.v = data.v
    rt.ax = data.ax
    rt.ay = data.ay
    rt.theta = data.theta
    rt.omega = data.omega

    return rt

class PCANDecoder:
    def __init__(self):
        self.sub_ = rospy.Subscriber("/pcan_data", PCANArray, self.callback)
        self.sub_ins_ = rospy.Subscriber("/Inertial_Labs/ins_data", ins_data, self.callback_ins)
        self.sub_sensor_ = rospy.Subscriber("/Inertial_Labs/sensor_data", self.callback_sensor)
        self.pub_ = rospy.Publisher("/mobileye", Mobileye, queue_size=20)
        self.result_path = "/home/jeongwoooh/result/"
        self.data_name = "test01"
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
        self.x = 0.0
        self.y = 0.0
        self.v = 0.0
        self.ax = 0.0
        self.ay = 0.0
        self.theta = 0.0
        self.omega = 0.0
        self.author = "jeongwoooh"
        self.email = "jeongwoo.oh@rllab.snu.ac.kr"
        self.copy_right = "RLLAB@SNU"
        self.date = datetime.datetime.now().strftime("%Y-%m-%d")


    def callback_ins(self, msg):
        """
        update x, y, v, theta
        """
        self.x = msg.LLH.x
        self.y = msg.LLH.y
        self.v = norm(msg.Vel_ENU)
        self.theta = math.pi / 180.0 * msg.YPR.x


    def callback_sensor(self, msg):
        """
        update ax, ay, omega
        """
        self.ax = msg.Accel.x
        self.ay = msg.Accel.y
        self.omega = msg.Gyro.z


    def callback(self, raw_msg):
        """
        callback function for PCAN raw data

        step 1. if msg type is 0x700, reset data
        step 2. stack received msg
        step 3. if msg type is 0x728, publish data
        """
        self.seq, self.time, self.message_stack = decode_pcan_array(raw_msg)
        
        self.publish()



    def save(self, data):
        save_path = self.result_path + "/" + self.data_name + "/" + str(msg.header.seq).zfill(6) + ".json"

        save_file = data.get_data()
        save_file['author'] = self.author
        save_file['email'] = self.email
        save_file['copyright'] = self.copy_right
        save_file['date'] = self.date
        
        with open(save_path, 'w') as outfile:
            json.dump(save_file, outfile, indent=4)


    def publish(self):
        rt = Mobileye()
        data = Data(self.seq, self.time, self.message_stack)
        data.add_vehicle_state(self.x,
                               self.y,
                               self.v,
                               self.ax,
                               self.ay,
                               self.theta,
                               self.omega)
        valid = data.decode_data()

        if not valid:
            rt.valid = False
        else:
            rt = generate_message(data)

        self.save(data)
        
        self.pub_.publish(rt)
        
        

if __name__=='__main__':
    rospy.init_node("pcan_decoder", anonymous=True)
    pcan_decoder = PCANDecoder()
    rospy.spin()
