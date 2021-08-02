#!/usr/bin/python
import rospy
import rospkg
import time
import math
import json
import os

from std_msgs.msg import Int32MultiArray
from sensor_decoder.msg import Data
from sensor_decoder.msg import Object
from sensor_decoder.msg import Label

INSERT_MODE = 1
DELETE_MODE = 2

class ObjectQuery:
    def __init__(self):
        self.data_name = "0729_exp_gunmin_highway"
        self.data_path = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + self.data_name + "/"
        self.state_path = self.data_path + "state/"

        self.seq = 200
        self.mode = 0
        # 0 : normal
        # 1 : insert mode
        # 2 : delete mode
        
        self.pub_data = rospy.Publisher("/query_data", Data, queue_size=10)
        self.pub_label = rospy.Publisher("/query_label", Label, queue_size=10)

        print("1 : valid query, 2 : object query")
        self.query_type = input()
        if self.query_type == 1:
            self.query()
        else :
            self.object_query()
        print("PRESS CTRL + C")

    def print_query(self):
        print("*********************************")
        print("current seq : %d" %(self.seq))
        if self.mode == 0:
            print("NORMAL MODE")
        if self.mode == INSERT_MODE:
            print("INSERT MODE")
        if self.mode == DELETE_MODE:
            print("DELETE MODE")
        print("=================================")
        
        print("a : move to prev data")
        print("d : move to next data")
        print("r : refresh current data")
        print("j : jump to certain data")
        print("q : change mode")
        print("c : show object info for current data")
        print("e : exit program")
        print("number : insert/delete object into/from valid list")
    
    def print_object_query(self):
        print("*********************************")
        print("current seq : %d" %(self.seq))
        if self.mode == 0:
            print("NORMAL MODE")
        if self.mode == INSERT_MODE:
            print("INSERT MODE")
        if self.mode == DELETE_MODE:
            print("DELETE MODE")
        print("=================================")
        
        print("a : move to prev data")
        print("d : move to next data")
        print("r : refresh current data")
        print("j : jump to certain data")
        print("c : show object info for current data")
        print("e : exit program")
        print("number : select object number")


    def load_state(self):
        self.current_state_path = self.state_path+str(self.seq).zfill(6)+".json"
        with open(self.current_state_path, "r") as st_json:
            self.state = json.load(st_json)
        rt = Data()
        rt.name = self.data_name
        rt.seq = self.seq
        self.pub_data.publish(rt)
        time.sleep(1.0)
        self.pub_object(self.state['objects'])
        
    def save_state(self):
        with open(self.current_state_path, 'w') as outfile:
            json.dump(self.state, outfile, indent=4)
                
    def pub_object(self, objects):
        rt = Label()
        obj_list = []
        for i in range(len(objects)):
            obj = Object()
            obj.x = objects[i]['x']
            obj.y = objects[i]['y']
            obj.theta = objects[i]['theta']
            obj.l = objects[i]['l']
            obj.w = objects[i]['w']
            obj.valid = objects[i]['valid']
            obj.id = i
            obj_list.append(obj)

        rt.objects = obj_list
        self.pub_label.publish(rt)


    def query(self):
        print("Input data name : (e.g. 0729_exp_jeongwoo_FMTC )")
        self.data_name = raw_input()
        self.data_path = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + self.data_name + "/"
        self.state_path = self.data_path + "state/"
        print("Input starting seq number : (e.g. 2100)")
        self.seq = int(raw_input())
        self.mode = INSERT_MODE
        self.load_state()
        while(1):
            print()
            self.print_query()
            x = raw_input("input query :")
            if x == 'a':
                self.save_state()
                self.seq -= 1
                if self.seq < 0:
                    rospy.ERROR("invalid seqeunce")
                    self.seq = 0
                self.load_state()
            elif x == 'd':
                self.save_state()
                self.seq += 1
                self.load_state()
            elif x == 'r':
                self.save_state()
                self.load_state()
            elif x == 'q':
                print("CHANGE MODE")
                self.mode = 3 - self.mode
                if self.mode == INSERT_MODE:
                    print("Now we are in INSERT MODE")
                else :
                    print("Now we are in DELETE MODE")
            elif x == 'c':
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
            elif x == 'j':
                self.save_state()
                print("INPUT seq num which you want to JUMP")
                y = input()
                self.seq = y
                self.load_state()
            elif x == 'e':
                break
            else :
                x = int(x)
                if self.mode == INSERT_MODE:
                    self.state['objects'][x]['valid'] = True
                else :
                    self.state['objects'][x]['valid'] = False
            self.pub_object(self.state['objects'])

    def object_query(self):
        print("Input data name : (e.g. 0729_exp_jeongwoo_FMTC )")
        self.data_name = raw_input()
        self.data_path = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + self.data_name + "/"
        self.state_path = self.data_path + "state/"
        print("Input starting seq number : (e.g. 2100)")
        self.seq = int(raw_input())
        self.mode = 0
        self.load_state()
        while(1):
            self.print_object_query()
            x = raw_input("input query :")
            if x == 'a':
                self.save_state()
                self.seq -= 1
                if self.seq < 0:
                    rospy.ERROR("invalid seqeunce")
                    self.seq = 0
                self.load_state()
            elif x == 'd':
                self.save_state()
                self.seq += 1
                self.load_state()
            elif x == 'r':
                self.save_state()
                self.load_state()
            elif x == 'c':
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
            elif x == 'j':
                self.save_state()
                print("INPUT seq num which you want to JUMP")
                y = input()
                self.seq = y
                self.load_state()
            elif x == 'e':
                break
            else :
                x = int(x)
                obj = self.state['objects'][x]
                print("obj info : x y theta l w")
                print("%.2f %.2f %.2f %.2f %.2f" %(obj['x'], obj['y'], obj['theta']/math.pi*180, obj['l'], obj['w']))
                print("x : change x value")
                print("y : change x value")
                print("t : change x value")
                print("l : change x value")
                print("w : change x value")
                y = raw_input()
                print("input value")
                z = input()
                if y == 'x':
                    self.state['objects'][x]['x'] = z
                if y == 'y':
                    self.state['objects'][x]['y'] = z
                if y == 't':
                    self.state['objects'][x]['theta'] = z
                if y == 'l':
                    self.state['objects'][x]['l'] = z
                if y == 'w':
                    self.state['objects'][x]['w'] = z

            self.pub_object(self.state['objects'])
            


if __name__=='__main__':
    rospy.init_node("object_query", anonymous=True)
    object_query = ObjectQuery()
    rospy.spin()
