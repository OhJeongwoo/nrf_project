import rospy
import rospkg
import json
import time

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

N = len(data_name_list)

start = time.time()
end = time.time()

for i, data_name in enumerate(data_name_list) :
    print("[%d / %d, %.2f] Start to object processing, data name is %s" %(i+1, N, end - start, data_name))
    data_path = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + data_name + "/"
    obj_path = data_path + "object/"
    state_path = data_path + "state/"
    M = n_data_list[i]
    for seq in range(1, M + 1):
        state_file = state_path + str(seq).zfill(6) + ".json"
        object_file = obj_path + str(seq).zfill(6) + ".txt"

        with open(state_file, "r") as st_json:
            state = json.load(st_json)

        with open(object_file, "r") as f:
            objects = []
            for line in f :
                box = line.split()
                obj = {"loss":{"box":float(box[1]), "total":float(box[0])}, "x":float(box[2]), "y":float(box[3]), "theta":float(box[4]), "l":float(box[5]), "w":float(box[6]), "valid":False}
                objects.append(obj)
            state["objects"] = objects

        with open(state_file, 'w') as outfile:
                json.dump(state, outfile, indent=4)

        end = time.time()
        
        if seq % 1000 == 0:
            print("[%d / %d, %.2f] In progress to object processing" %(seq, M, end-start))



