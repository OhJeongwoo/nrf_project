#!/usr/bin/python
import rospy
import rospkg
import time
import math
import json
import os
import operator

from std_msgs.msg import Int32MultiArray
from sensor_decoder.msg import Data
from sensor_decoder.msg import Object
from sensor_decoder.msg import Label



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

data_index_list = [98,99]

score_threshold = 5.0
MAX_RELIABILITY = 5


def calculate_score(A, B):
    return math.sqrt((A['x'] - B['x']) ** 2 + (A['y'] - B['y']) ** 2)

def matching_algorithm(cur_obj_list, obj_list, hash_id):
    A = len(cur_obj_list)
    B = len(obj_list)

    q = []
    for a in range(A):
        for b in range(B):
            q.append({'score':calculate_score(cur_obj_list[a], obj_list[b]), "index":[a,b]})
    
    q = sorted(q, key=operator.itemgetter('score'))

    visitA = [False for i in range(A)]
    visitB = [False for i in range(B)]
    n = len(q)
    for i in range(n):
        if q[i]['score'] > score_threshold:
            break
        a = q[i]['index'][0]
        b = q[i]['index'][1]
        if visitA[a] or visitB[b] :
            continue
        visitA[a] = True
        visitB[b] = True
        cur_obj_list[a]['age'] += 1
        cur_obj_list[a]['reliability'] = min(cur_obj_list[a]['reliability'] + 1, MAX_RELIABILITY)
        cur_obj_list[a]['x'] = obj_list[b]['x']
        cur_obj_list[a]['y'] = obj_list[b]['y']

        obj_list[b]['id'] = cur_obj_list[a]['id']
        obj_list[b]['age'] = cur_obj_list[a]['age']
        obj_list[b]['reliability'] = cur_obj_list[a]['reliability']
    
    for a in range(A):
        if visitA[a]:
            continue
        cur_obj_list[a]['age'] += 1
        cur_obj_list[a]['reliability'] -= 1

    for b in range(B):
        if visitB[b]:
            continue
        obj_list[b]['id'] = hash_id
        hash_id += 1
        obj_list[b]['age'] = 1
        obj_list[b]['reliability'] = 1
        cur_obj_list.append({'x':obj_list[b]['x'],
                             'y':obj_list[b]['y'],
                             'id':obj_list[b]['id'],
                             'age':obj_list[b]['age'],
                             'reliability':obj_list[b]['reliability']})
    
    updated_obj_list = []
    for obj in cur_obj_list:
        if obj['reliability'] > 0:
            updated_obj_list.append(obj)
    return updated_obj_list, obj_list, hash_id


if __name__=='__main__':
    rospy.init_node("assign_id", anonymous=True)
    for data_index in data_index_list:
        hash_id = 0
        data_name = data_name_list[data_index]
        data_path = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + data_name + "/"
        new_state_path = data_path + "new_state/"
        print(data_name)
        
        st = seq_list[data_index][0]
        en = seq_list[data_index][1]

        N = en - st
        hash_id = 0

        cur_obj_list = []
        # each object in cur_obj_list has 'x', 'y', 'id', 'reliability', 'age' attributes

        print("load object list")
        for seq in range(st+1, en+1):
            current_new_state_path = new_state_path+str(seq).zfill(6)+".json"
            with open(current_new_state_path, "r") as st_json:
                state = json.load(st_json)
            
            obj_list = state['objects']
            cur_obj_list, obj_list, hash_id = matching_algorithm(cur_obj_list, obj_list, hash_id)

            state['objects'] = obj_list
            with open(current_new_state_path, 'w') as outfile:
                json.dump(state, outfile, indent=4)
        print(hash_id)
        
    rospy.spin()
