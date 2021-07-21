#!/usr/bin/python
import rospy
import rospkg
import time
import math
import json
import os

from std_msgs.msg import Int32MultiArray


class ObjectQuery:
    def __init__(self):
        self.data_name = "sumin_highway"
        self.data_path = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + self.data_name + "/"
        
        self.state_path = self.data_path + "state/"

        self.seq = 200
        self.mode = 0
        # 0 : normal
        # 1 : insert mode
        # 2 : delete mode
        
        self.pub = rospy.Publisher("/query", Int32MultiArray, queue_size=10)

        self.query()
        print("PRESS CTRL + C")

    def print_query(self):
        print("*********************************")
        print("current seq : %d" %(self.seq))
        if self.mode == 0:
            print("NORMAL MODE")
        if self.mode == 1:
            print("INSERT MODE")
        if self.mode == 2:
            print("DELETE MODE")
        print("=================================")
        
        print("a : move to prev data")
        print("d : move to next data")
        print("j : jump to certain data")
        print("z : change to insert mode")
        print("x : change to delete mode")
        print("c : show object info for current data")
        print("e : exit program")
        


    def load_state(self):
        self.current_state_path = self.state_path+str(self.seq).zfill(6)+".json"
        with open(self.current_state_path, "r") as st_json:
            self.state = json.load(st_json)
        rt = Int32MultiArray()
        rt.data = [self.seq, 1]
        self.pub.publish(rt)
        
    def save_state(self):
        with open(self.current_state_path, 'w') as outfile:
            json.dump(self.state, outfile, indent=4)
                

    def query(self):
        self.load_state()
        while(1):
            self.print_query()
            x = raw_input("input query :")
            if x == 'a':
                self.save_state()
                self.seq -= 1
                if self.seq < 0:
                    rospy.ERROR("invalid seqeunce")
                    self.seq = 0
                self.load_state()
            if x == 'd':
                self.save_state()
                self.seq += 1
                self.load_state()
            if x == 'z':
                print("CHANGE TO INSERT MODE")
                print("number : input object id which you want to change to valid")
                print("-1 : exit INSERT mode")
                while(1):
                    y = input()
                    if y == -1:
                        break
                    elif y >= 0:
                        self.state['objects'][y]['valid'] = True
                    else :
                        continue
                self.save_state()
            if x == 'x':
                print("CHANGE TO DELETE MODE")
                print("number : input object id which you want to change to invalid")
                print("-1 : exit DELETE mode")
                while(1):
                    y = input()
                    if y == -1:
                        break
                    elif y >= 0:
                        self.state['objects'][y]['valid'] = False
                    else :
                        continue
                self.save_state()
            if x == 'c':
                print("CURRENT STATUS")
                valid_list = []
                invalid_list = []
                for i in range(len(self.state['objects'])):
                    if self.state['objects'][i]['valid']:
                        valid_list.append(i)
                    else :
                        invalid_list.append(i)
                print("valid list")
                for i in valid_list:
                    print(i)
                print("invalid list")
                for i in invalid_list:
                    print(i)
            if x == 'j':
                self.save_state()
                print("INPUT seq num which you want to JUMP")
                y = input()
                self.seq = y
                self.load_state()
            if x == 'e':
                break
            


if __name__=='__main__':
    rospy.init_node("object_query", anonymous=True)
    object_query = ObjectQuery()
    rospy.spin()
