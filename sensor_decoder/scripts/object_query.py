#!/usr/bin/python
import rospy
import rospkg
import time
import math
import json
import os
import numpy as np

from std_msgs.msg import Int32MultiArray
from sensor_decoder.msg import Data
from sensor_decoder.msg import Object
from sensor_decoder.msg import Label

INSERT_MODE = 1
DELETE_MODE = 2

data_name_list = ['0729_exp_gunmin_FMTC'
                ,'0729_exp_gunmin_highway'
                ,'0729_exp_gunmin_road'
                ,'0729_exp_jeongwoo_FMTC'
                ,'0729_exp_sumin_FMTC'
                ,'0729_exp_sumin_highway'
                ,'0729_exp_sumin_road'
                ,'0729_exp_wooseok_FMTC'
                ,'0729_neg_gunmin_01_1'
                ,'0729_neg_gunmin_02_1'
                ,'0729_neg_gunmin_03_1'
                ,'0729_neg_gunmin_05_1'
                ,'0729_neg_gunmin_06_1'
                ,'0729_neg_gunmin_08_1'
                ,'0729_neg_gunmin_09_1'
                ,'0729_neg_gunmin_10_1'
                ,'0729_neg_gunmin_16_1'
                ,'0729_neg_gunmin_16_2'
                ,'0729_neg_gunmin_28_1'
                ,'0729_neg_gunmin_28_2'
                ,'0729_neg_gunmin_29_1'
                ,'0729_neg_gunmin_30_1'
                ,'0729_neg_gunmin_30_2'
                ,'0729_neg_gunmin_31_1'
                ,'0729_neg_gunmin_31_2'
                ,'0729_neg_gunmin_34_1'
                ,'0729_neg_gunmin_34_2'
                ,'0729_neg_gunmin_35_1'
                ,'0729_neg_gunmin_35_2'
                ,'0729_neg_gunmin_36_1'
                ,'0729_neg_gunmin_36_2'
                ,'0729_neg_gunmin_37_1'
                ,'0729_neg_gunmin_37_2'
                ,'0729_neg_gunmin_38_1'
                ,'0729_neg_gunmin_38_2'
                ,'0729_neg_gunmin_38_3'
                ,'0729_neg_gunmin_38_4'
                ,'0729_neg_gunmin_38_5'
                ,'0729_neg_gunmin_38_6'
                ,'0729_neg_gunmin_50_1'
                ,'0729_neg_gunmin_50_2'
                ,'0729_neg_jeongwoo_01_1'
                ,'0729_neg_jeongwoo_02_1'
                ,'0729_neg_jeongwoo_03_1'
                ,'0729_neg_jeongwoo_05_1'
                ,'0729_neg_jeongwoo_06_1'
                ,'0729_neg_jeongwoo_08_1'
                ,'0729_neg_jeongwoo_09_1'
                ,'0729_neg_jeongwoo_10_1'
                ,'0729_neg_jeongwoo_50_1'
                ,'0729_neg_jeongwoo_50_2'
                ,'0729_neg_sumin_01_1'
                ,'0729_neg_sumin_02_1'
                ,'0729_neg_sumin_03_1'
                ,'0729_neg_sumin_05_1'
                ,'0729_neg_sumin_06_1'
                ,'0729_neg_sumin_08_1'
                ,'0729_neg_sumin_09_1'
                ,'0729_neg_sumin_10_1'
                ,'0729_neg_sumin_38_1'
                ,'0729_neg_sumin_38_2'
                ,'0729_neg_sumin_38_3'
                ,'0729_neg_sumin_38_4'
                ,'0729_neg_sumin_42_1'
                ,'0729_neg_sumin_42_2'
                ,'0729_neg_sumin_42_3'
                ,'0729_neg_sumin_42_4'
                ,'0729_neg_sumin_50_1'
                ,'0729_neg_sumin_50_2'
                ,'0729_neg_wooseok_01_1'
                ,'0729_neg_wooseok_02_1'
                ,'0729_neg_wooseok_03_1'
                ,'0729_neg_wooseok_05_1'
                ,'0729_neg_wooseok_06_1'
                ,'0729_neg_wooseok_08_1'
                ,'0729_neg_wooseok_09_1'
                ,'0729_neg_wooseok_10_1'
                ,'0729_neg_wooseok_28'
                ,'0729_neg_wooseok_28_1'
                ,'0729_neg_wooseok_29_1'
                ,'0729_neg_wooseok_29_2'
                ,'0729_neg_wooseok_30_1'
                ,'0729_neg_wooseok_30_2'
                ,'0729_neg_wooseok_31_1'
                ,'0729_neg_wooseok_31_2'
                ,'0729_neg_wooseok_34_2'
                ,'0729_neg_wooseok_35_1'
                ,'0729_neg_wooseok_35_2'
                ,'0729_neg_wooseok_36_1'
                ,'0729_neg_wooseok_36_2'
                ,'0729_neg_wooseok_37_1'
                ,'0729_neg_wooseok_37_2'
                ,'0729_neg_wooseok_46'
                ,'0729_neg_wooseok_47'
                ,'0729_neg_wooseok_50_1'
                ,'0729_neg_wooseok_50_2'
                ,'0813_exp_jeongwoo_road_1'
                ,'0813_exp_jeongwoo_road_2'
                ,'0815_exp_jeongwoo_highway_1'
                ,'0815_exp_jeongwoo_highway_2']


seq_list = [(50,2450),
            (6000,9000),
            (5000,8000),
            (0,2200),
            (150,2550),
            (500,3500),
            (6000,9000),
            (50,1450),
            (40,140),
            (0,140),
            (0,160),
            (90,150),
            (90,170),
            (50,150),
            (90,190),
            (30,80),
            (80,180),
            (220,260),
            (200,220),
            (120,140),
            (120,140),
            (130,160),
            (180,200),
            (160,180),
            (145,165),
            (120,140),
            (90,110),
            (140,160),
            (180,200),
            (120,140),
            (180,200),
            (80,100),
            (120,130),
            (120,140),
            (150,170),
            (140,160),
            (150,170),
            (240,270),
            (180,200),
            (100,200),
            (80,200),
            (0,150),
            (0,150),
            (0,100),
            (80,190),
            (110,190),
            (50,150),
            (170,220),
            (70,100),
            (90,160),
            (90,290),
            (80,150),
            (70,160),
            (40,140),
            (40,80),
            (230,270),
            (50,150),
            (50,70),
            (130,150),
            (180,200),
            (120,140),
            (155,175),
            (120,140),
            (190,210),
            (140,160),
            (110,130),
            (130,150),
            (80,250),
            (80,180),
            (80,160),
            (50,150),
            (50,130),
            (120,190),
            (100,150),
            (40,110),
            (300,350),
            (360,400),
            (160,190),
            (125,155),
            (120,140),
            (100,110),
            (80,100),
            (90,110),
            (70,90),
            (110,130),
            (130,150),
            (90,110),
            (90,110),
            (230,250),
            (80,100),
            (100,130),
            (110,140),
            (170,190),
            (160,190),
            (30,130),
            (170,220),
            (1400,4400),
            (14000,17000),
            (1400,4400),
            (5500,8500)]


class ObjectQuery:
    def __init__(self):
        # 0 : normal
        # 1 : insert mode
        # 2 : delete mode
        
        self.pub_data = rospy.Publisher("/query_data", Data, queue_size=10)
        self.pub_label = rospy.Publisher("/query_label", Label, queue_size=10)

        print("1: valid query")
        print("2: object query")
        print("3: test query(don't do this query")
        print("4: init theta query")
        print("5: l,w query")
        print("6: fast valid query")
        self.query_type = input()
        if self.query_type == 1:
            self.query()
        elif self.query_type == 2 :
            self.object_query()
        elif self.query_type == 3 :
            self.test_query()
        elif self.query_type == 4 :
            self.theta_query()
        elif self.query_type == 5 :
            self.l_query()
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

    def save_theta(self):
        for seq in range(self.seq, seq_list[self.data_name_index][1] + 1):
            cur_file = self.state_path+str(seq).zfill(6)+".json"
            with open(cur_file, "r") as st_json:
                state = json.load(st_json)
            for i in range(len(state['objects'])):
                if state['objects'][i]['id'] == self.id:
                    state['objects'][i] = self.theta + np.random.normal(0, 0.01)
            self.theta -= state['omega'] * 0.1
        

                
    def pub_object(self, objects):
        rt = Label()
        rt.only_valid = (self.query_type == 2)
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
            # obj.id = objects[i]['id']
            obj_list.append(obj)

        rt.objects = obj_list
        self.pub_label.publish(rt)


    def query(self):
        print("DATA NAME LIST")
        print("========================================")
        for i in range(0,100):
            print("%d : %s" %(i, data_name_list[i]))
        print("========================================")
        print("Input data name index")
        self.data_name_index = input()
        self.data_name = data_name_list[self.data_name_index]
        self.data_path = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + self.data_name + "/"
        self.state_path = self.data_path + "state/"
        st = seq_list[self.data_name_index][0]
        en = seq_list[self.data_name_index][1]
        print("========================================")
        print("YOU HAVE TO WORK LABELING WITH SEQUENCE NUM %d to %d" %(st+1, en))
        print("========================================")
        
        print("Input starting seq number : (e.g. 2100)")
        self.seq = int(raw_input())
        self.mode = INSERT_MODE
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
        print("DATA NAME LIST")
        print("========================================")
        for i in range(0,100):
            print("%d : %s" %(i, data_name_list[i]))
        print("========================================")
        print("Input data name index")
        self.data_name_index = input()
        self.data_name = data_name_list[self.data_name_index]
        self.data_path = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + self.data_name + "/"
        self.state_path = self.data_path + "new_state/"
        st = seq_list[self.data_name_index][0]
        en = seq_list[self.data_name_index][1]
        N = en - st
        valid_list = []
        for seq in range(st + 1, en + 1):
            self.current_state_path = self.state_path+str(seq).zfill(6)+".json"
            with open(self.current_state_path, "r") as st_json:
                self.state = json.load(st_json)
            
            check = False
            for obj in self.state['objects']:
                if obj['valid']:
                    check = True
                    break
            if check:
                valid_list.append(seq)
        
            if (seq - st) % 10 == 0:
                print("[%d/%d] Searching valid sequence from data named %s" %(seq - st, N, self.data_name))
        print("Finish searching valid list")
        print("Valid list")
        for seq in valid_list:
            print(seq)
        print("========================================")
        print("Recommend to copy list to other file")        
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
                for i in range(len(self.state['objects'])):
                    if self.state['objects'][i]['id'] ==x:
                        x = i
                        break
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
                    self.state['objects'][x]['x'] += z
                if y == 'y':
                    self.state['objects'][x]['y'] += z
                if y == 't':
                    self.state['objects'][x]['theta'] += z * math.pi / 180
                if y == 'l':
                    self.state['objects'][x]['l'] = z
                if y == 'w':
                    self.state['objects'][x]['w'] = z

            self.pub_object(self.state['objects'])

    def test_query(self):
        print("DATA NAME LIST")
        print("========================================")
        for i in range(0,100):
            print("%d : %s" %(i, data_name_list[i]))
        print("========================================")
        print("Input data name index")
        self.data_name_index = input()
        self.data_name = data_name_list[self.data_name_index]
        self.data_path = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + self.data_name + "/"
        self.state_path = self.data_path + "new_state/"
        st = seq_list[self.data_name_index][0]
        en = seq_list[self.data_name_index][1]
        N = en - st
        valid_list = []
        for seq in range(st + 1, en + 1):
            self.current_state_path = self.state_path+str(seq).zfill(6)+".json"
            with open(self.current_state_path, "r") as st_json:
                self.state = json.load(st_json)
            
            check = False
            for obj in self.state['objects']:
                if obj['valid']:
                    check = True
                    break
            if check:
                valid_list.append(seq)
        
            if (seq - st) % 10 == 0:
                print("[%d/%d] Searching valid sequence from data named %s" %(seq - st, N, self.data_name))
        print("Finish searching valid list")
        print("Valid list")
        for seq in valid_list:
            print(seq)
        print("========================================")
        print("Recommend to copy list to other file")        
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
                    self.state['objects'][x]['x'] += z
                if y == 'y':
                    self.state['objects'][x]['y'] += z
                if y == 't':
                    self.state['objects'][x]['theta'] += z * math.pi / 180
                if y == 'l':
                    self.state['objects'][x]['l'] = z
                if y == 'w':
                    self.state['objects'][x]['w'] = z

            self.pub_object(self.state['objects'])
        
    def theta_query(self):
        print("DATA NAME LIST")
        print("========================================")
        for i in range(0,100):
            print("%d : %s" %(i, data_name_list[i]))
        print("========================================")
        print("Input data name index")
        self.data_name_index = input()
        self.data_name = data_name_list[self.data_name_index]
        self.data_path = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + self.data_name + "/"
        self.state_path = self.data_path + "new_state/"
        st = seq_list[self.data_name_index][0]
        en = seq_list[self.data_name_index][1]
        N = en - st
        omega_list = []
        obj_list = []
        for seq in range(st+1, en+1):
            current_new_state_path = self.state_path+str(seq).zfill(6)+".json"
            with open(current_new_state_path, "r") as st_json:
                state = json.load(st_json)
            omega_list.append(state['omega'])
            obj_list.append(state['objects'])
        
        M = -1
        for objs in obj_list:
            for obj in objs:
                if M < obj['id']:
                    M = obj['id']
        M += 1 # NUMBER OF ID
        
        print("========================================")
        print("YOU HAVE TO WORK LABELING WITH OBJECT %d" %(M))
        print("========================================")
        start = input()
        for id in range(start,M):
            self.id = id
            for seq in range(st+1, en+1):
                current_new_state_path = self.state_path+str(seq).zfill(6)+".json"
                with open(current_new_state_path, "r") as st_json:
                    state = json.load(st_json)
                check = False
                for obj in state['objects']:
                    if obj['id'] == self.id:
                        self.seq = seq
                        self.theta = obj['theta']
                        check = True
                        break
                if check :
                    break
            self.load_state()
            while True :
                print("input theta, if input 1000, break")
                x = int(input())
                if x == 1000:
                    break
                self.theta = self.theta + x / 180.0 * math.pi
                for i in range(len(self.state['objects'])):
                    if self.state['objects'][i]['id'] == self.id:
                        print(self.id)
                        print(self.state['objects'][i]['id'])
                        self.state['objects'][i]['theta'] = self.theta
                self.pub_object(self.state['objects'])
            for seq in range(self.seq, en+1):
                cur_file = self.state_path+str(seq).zfill(6)+".json"
                with open(cur_file, "r") as st_json:
                    state = json.load(st_json)
                for i in range(len(state['objects'])):
                    if state['objects'][i]['id'] == self.id:
                        state['objects'][i]['theta'] = self.theta + np.random.normal(0, 0.01)
                with open(cur_file, 'w') as outfile:
                    json.dump(state, outfile, indent=4)

                self.theta -= state['omega'] * 0.1

    def l_query(self):
        print("DATA NAME LIST")
        print("========================================")
        for i in range(0,100):
            print("%d : %s" %(i, data_name_list[i]))
        print("========================================")
        print("Input data name index")
        self.data_name_index = input()
        self.data_name = data_name_list[self.data_name_index]
        self.data_path = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + self.data_name + "/"
        self.state_path = self.data_path + "new_state/"
        st = seq_list[self.data_name_index][0]
        en = seq_list[self.data_name_index][1]
        N = en - st
        omega_list = []
        obj_list = []
        for seq in range(st+1, en+1):
            current_new_state_path = self.state_path+str(seq).zfill(6)+".json"
            with open(current_new_state_path, "r") as st_json:
                state = json.load(st_json)
            omega_list.append(state['omega'])
            obj_list.append(state['objects'])
        
        M = -1
        for objs in obj_list:
            for obj in objs:
                if M < obj['id']:
                    M = obj['id']
        M += 1 # NUMBER OF ID
        
        print("========================================")
        print("YOU HAVE TO WORK LABELING WITH OBJECT %d" %(M))
        print("========================================")
        start = input()
        for id in range(start,M):
            print(id)
            self.id = id
            check = False
            for seq in range(st+1, en+1):
                current_new_state_path = self.state_path+str(seq).zfill(6)+".json"
                with open(current_new_state_path, "r") as st_json:
                    state = json.load(st_json)
                for obj in state['objects']:
                    if obj['id'] == self.id and obj['x'] > 10.0:
                        self.seq = seq
                        self.l = obj['l']
                        check = True
                        print("success")
                        break
                if check :
                    break
            if not check :
                print("fail")
                continue
            self.load_state()
            print(self.seq)
            while True :
                print("input theta, if input x, break")
                x = raw_input()
                if x == 'x':
                    break
                x = float(x)
                self.l = x
                print(len(self.state['objects']))
                for i in range(len(self.state['objects'])):
                    print(self.state['objects'][i]['id'])
                    if self.state['objects'][i]['id'] == self.id:
                        print(self.id)
                        print(self.state['objects'][i]['id'])
                        self.state['objects'][i]['l'] = self.l
                self.pub_object(self.state['objects'])
            for seq in range(st+1, en+1):
                cur_file = self.state_path+str(seq).zfill(6)+".json"
                with open(cur_file, "r") as st_json:
                    state = json.load(st_json)
                for i in range(len(state['objects'])):
                    if state['objects'][i]['id'] == self.id:
                        state['objects'][i]['l'] = self.l + np.random.normal(0, 0.5)
                with open(cur_file, 'w') as outfile:
                    json.dump(state, outfile, indent=4)

                
    

        


if __name__=='__main__':
    rospy.init_node("object_query", anonymous=True)
    object_query = ObjectQuery()
    rospy.spin()
