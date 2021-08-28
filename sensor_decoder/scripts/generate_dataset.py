#!/usr/bin/python
from math import exp
import os
import rospy
import rospkg
import shutil
import json
import operator


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

def make_dir(path):
    if not os.path.exists(path):
        os.mkdir(path)

is_light = True
if is_light:
    dataset_path = rospkg.RosPack().get_path("sensor_decoder") + "/light_mixquality/"
else :
    dataset_path = rospkg.RosPack().get_path("sensor_decoder") + "/mixquality/"
make_dir(dataset_path)

exp_path = dataset_path + "exp/"
neg_path = dataset_path + "neg/"
make_dir(exp_path)
make_dir(neg_path)

dev = 0.0

data_index_list = [i for i in range(100)]

for data_index in data_index_list:
    data_name = data_name_list[data_index]
    if data_name[5:8] == 'exp':
        data_path = exp_path + data_name + "/"
    elif data_name[5:8] == 'neg':
        data_path = neg_path + data_name + "/"
    else:
        print("wrong data name %s" %(data_name))
        continue
    make_dir(data_path)
    src_data_path = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + data_name + "/"
    src_state_path = src_data_path + "new_state/"
    src_bin_path = src_data_path + "bin/"
    src_img_path = src_data_path + "image_raw/"
    src_map_path = src_data_path + "local_map/"


    state_path = data_path + "state/"
    bin_path = data_path + "bin/"
    img_path = data_path + "image_raw/"
    map_path = data_path + "local_map/"

    make_dir(state_path)
    make_dir(bin_path)
    make_dir(img_path)
    make_dir(map_path)

    st = seq_list[data_index][0]
    en = seq_list[data_index][1]

    print("START TO GENERATE DATASET %s" %(data_name))
    for seq in range(st+1, en+1):
        file_name = str(seq).zfill(6)
        src_state_file = src_state_path + file_name + ".json"
        src_bin_file = src_bin_path + file_name + ".bin"
        src_img_file = src_img_path + file_name + ".png"
        src_map_file = src_map_path + file_name + ".png"

        dst_state_file = state_path + file_name + ".json"
        dst_bin_file = bin_path + file_name + ".bin"
        dst_img_file = img_path + file_name + ".png"
        dst_map_file = map_path + file_name + ".png"

        # shutil.copy(src_bin_file, dst_bin_file)
        # shutil.copy(src_img_file, dst_img_file)
        # shutil.copy(src_map_file, dst_map_file)

        with open(src_state_file, "r") as st_json:
            state = json.load(st_json)
        
        save_state = {}
        if 'target_deviation' not in state:
            print(data_name)
            print(seq)
        save_state['date'] = state['date']
        save_state['data_name'] = state['data_name']
        save_state['author'] = state['author']
        save_state['email'] = state['email']
        save_state['copy_right'] = state['copy_right']
        save_state['x'] = state['x']
        save_state['y'] = state['y']
        save_state['theta'] = state['theta']
        save_state['v'] = state['v']
        save_state['ax'] = state['ax']
        save_state['ay'] = state['ay']
        save_state['omega'] = state['omega']
        if 'target_deviation' not in state:
            save_state['deviation'] = dev
        else:
            save_state['deviation'] = state['target_deviation']
            dev = save_state['deviation']
        save_state['decision'] = state['decision']
        if 'is_tunnel' not in state:
            save_state['is_tunnel'] = False
        else :    
            save_state['is_tunnel'] = state['is_tunnel']
        save_state['lanes'] = state['lanes']
        object_lists = state['filtered_objects']
        for i in range(len(object_lists)):
            object_lists[i]['value'] = abs(object_lists[i]['x']) + abs(object_lists[i]['y'])
        save_state['objects'] = sorted(object_lists, key=operator.itemgetter('value'))
        # save_state['objects'] = state['filtered_objects']

        with open(dst_state_file, 'w') as outfile:
                json.dump(save_state, outfile, indent=4)
        
        if (seq - st) % 100 == 0:
            print("[CUR %d / %d]" %(seq - st, en - st))
    