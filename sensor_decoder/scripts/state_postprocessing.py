import rospy
import rospkg

import numpy as np
import cv2
import json
import math
import time



# start_data = 1
# end_data = 5001
# invalid = (8933, 9409)

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
                ,'0729_neg_gunmin_29_2'
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
                ,'0729_neg_wooseok_34_1'
                ,'0729_neg_wooseok_34_2'
                ,'0729_neg_wooseok_35_1'
                ,'0729_neg_wooseok_35_2'
                ,'0729_neg_wooseok_36_1'
                ,'0729_neg_wooseok_36_2'
                ,'0729_neg_wooseok_37_1'
                ,'0729_neg_wooseok_37_2'
                ,'0729_neg_wooseok_46'
                ,'0729_neg_wooseok_47'
                ,'0729_neg_wooseok_48'
                ,'0729_neg_wooseok_50_1'
                ,'0729_neg_wooseok_50_2']

n_data_list = [2519,
            10002,
            8887,
            2307,
            2719,
            11232,
            9053,
            1560,
            137,
            142,
            154,
            155,
            161,
            167,
            241,
            92,
            222,
            263,
            233,
            159,
            150,
            263,
            178,
            221,
            196,
            192,
            169,
            135,
            187,
            238,
            159,
            225,
            118,
            168,
            302,
            229,
            213,
            271,
            314,
            229,
            265,
            247,
            155,
            179,
            116,
            198,
            192,
            160,
            236,
            135,
            275,
            380,
            145,
            190,
            130,
            88,
            309,
            183,
            88,
            170,
            232,
            184,
            205,
            178,
            228,
            174,
            142,
            159,
            293,
            200,
            300,
            570,
            170,
            211,
            164,
            121,
            471,
            492,
            227,
            182,
            149,
            163,
            104,
            124,
            94,
            136,
            176,
            210,
            193,
            134,
            263,
            145,
            140,
            183,
            683,
            208,
            159,
            322,
            515]

prev_state = None
prev_theta = 0.0

N = len(data_name_list)

start = time.time()
end = time.time()

for i, data_name in enumerate(data_name_list):
    print("[%d / %d, %.2f] Start to state post-processing, data name is %s" %(i+1, N, end - start, data_name))
    data_path = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + data_name + "/"
    state_path = data_path + "state/"

    M = n_data_list[i]

    for seq in range(1, M+1):
        state_file = state_path + str(seq).zfill(6) + ".json"
        with open(state_file, "r") as st_json:
            state = json.load(st_json)

        state['data_name'] = data_name

        # add lateral deviation
        dev_list = []
        for lane in state['lanes']:
            dev_list.append(lane['c0'])
        dev_list = sorted(dev_list, key = abs)
        state['target_deviation'] = (dev_list[0] + dev_list[1]) / 2.0

        # revise decision
        # if v is small, decision turns into stop(4), and theta still keeps previous value
        # if v is not small but decision is stop(4), decision turns into keep lane(1)
        if state['v'] < 1.0:
            state['decision'] = 4
            state['theta'] = prev_theta
        else :
            if state['decision'] == 4:
                state['decision'] = 1
            prev_theta = state['theta']

        # add tunnel attributes
        # if v is not small and location doesn't change, is_tunnel is true
        if prev_state is None:
            state['is_tunnel'] = False
        else :
            distance = math.sqrt((prev_state['x'] - state['x']) ** 2 + (prev_state['y'] - state['y']) ** 2)
            if state['v'] > 5.0 and distance < 1e-7:
                state['is_tunnel'] = True
                # print("[WARN] %d-th seq is in a tunnel" %(seq))
            else :
                state['is_tunnel'] = False


        with open(state_file, 'w') as outfile:
                json.dump(state, outfile, indent=4)

        prev_state = state

        end = time.time()
        if seq % 1000 == 0:
            print("[%d / %d, %.2f] In progress to state post-processing" %(seq, M, end-start))
    


# for seq in range(start_data, end_data):
#     print(seq)
#     # if seq in range(invalid[0], invalid[1]):
#     #     continue
#     with open(state_path+str(seq).zfill(6)+".json", "r") as st_json:
#         state = json.load(st_json)

    
    
    


