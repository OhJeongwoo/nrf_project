import os
import rospy
import rospkg
import numpy as np

import json
import operator
import math


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

seq_list = [(50,2450),
            (6000,9000),
            (4000,7000),
            (0,2200),
            (150,2550),
            (500,3500),
            (5000,8000),
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
            (110,140),
            (130,160),
            (180,200),
            (160,180),
            (145,165),
            (110,130),
            (80,110),
            (120,150),
            (170,200),
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
            (140,170),
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
            (120,140),
            (130,150),
            (90,110),
            (90,110),
            (230,250),
            (80,100),
            (100,130),
            (110,140),
            (170,190),
            (160,190),
            (130,150),
            (30,130),
            (170,220)]

score_threshold = 3.0
P_init = np.eyes(9)
Q = np.eyes(9)
R = np.eyes(6)
H = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],])


def predict(obj):
    state = np.array(obj['state'])
    dt = obj['dt']
    xt = state[0]
    yt = state[1]
    tt = state[2]
    vt = state[3]
    at = state[4]
    wt = state[5]
    ve = state[6]
    ae = state[7]
    we = state[8]
    F = np.zeros(9,9)
    
    
    F[0,0] = 1.0
    F[0,1] = 0.0
    F[0,2] = -math.sin(tt) * (vt * dt + 0.5 * at * dt * dt) - math.cos(tt) * 0.5 * vt * wt * dt * dt
    F[0,3] = math.cos(tt) * dt - math.sin(tt) * 0.5 * wt * dt * dt
    F[0,4] = 0.5 * math.cos(tt) * dt * dt
    F[0,5] = -math.sin(tt) * 0.5 * vt * dt * dt
    F[0,6] = -dt
    F[0,7] = -dt * dt
    F[0,8] = 0.0

    F[1,0] = 0.0
    F[1,1] = 1.0
    F[1,2] = math.cos(tt) * (vt * dt + 0.5 * at * dt * dt) - math.sin(tt) * 0.5 * vt * wt * dt * dt
    F[1,3] = math.sin(tt) + 0.5 * math.cos(tt) * wt * dt * dt
    F[1,4] = 0.5 * math.sin(tt) * dt * dt
    F[1,5] = 0.5 * math.cos(tt) * vt * dt * dt
    F[1,6] = -0.5 * we * dt * dt
    F[1,7] = 0.0
    F[1,8] = -0.5 * ve * dt * dt
    

    F[2,2] = 1.0
    F[2,5] = dt
    F[2,8] = -dt

    F[3,3] = 1.0
    F[3,4] = 0.1

    F[4,4] = 1.0

    F[5,5] = 1.0

    F[6,6] = 1.0
    F[6,7] = 0.1

    F[7,7] = 1.0

    F[8,8] = 1.0
    
    predict = F * state
    obj['F'] = F
    obj['prediction'] = predict.tolist()

    return obj

def calculate_score(A, B):
    return ((A['prediction'][0]-B['x'])**2 + (A['prediction'][1]-B['y'])**2) / max(A['prediction'][3], 1.0) / max(B['v'], 1.0)

def kalman_update(cur_objects_list, observed_data, ego, hash_id):
    A = len(cur_objects_list)
    B = len(observed_data)
    
    arr = []
    for a in range(A):
        cur_objects_list[a] = predict(cur_objects_list[a])
    for a in range(A):
        for b in range(B):
            arr.append({'score':calculate_score(cur_objects_list[a], observed_data[b]), 'id': [a,b]})

    sorted(arr, key=operator.itemgetter('value'))
    
    rt = []
    visitA = [False for i in range(A)]
    visitB = [False for i in range(B)]
    for i in range(len(arr)):
        if arr[i]['score'] > score_threshold:
            break
        a = arr[i]['id'][0]
        b = arr[i]['id'][1]
        if visitA[a] or visitB[b]:
            continue
        visitA[a] = True
        visitB[b] = True
        obj = cur_objects_list[a]
        obv = observed_data[b]
        F = obj['F']
        P = np.array(obj['P'])
        r = obj['reliability']
        P = F * P * np.transpose(F) + Q / r
        z = [obv['x'], obv['y'], obv['theta'], ego['v'], ego['ax'], ego['omega']]
        y = z - H * np.array(obj['prediction'])
        S = H * P * np.transpose(H) + R / r
        K = P * np.transpose(H) * np.linalg.inv(S)
        state =  np.array(obj['prediction']) + K * y
        obj['P'] = ((np.eye(9) - K * H) * P).tolist()
        obj['age'] = obj['age'] + 1
        
        

    for a in range(A):
        if visitA[a]:
            continue
        tar = cur_objects_list[a]
        if tar['reliability'] == 1:
            continue
        obj = {}
        obj['state'] = tar['prediction']
        obj['age'] = tar['age'] + 1
        obj['id'] = tar['id']
        obj['reliability'] = tar['reliability'] - 1
        obj['P'] = tar['P']
        obj['dt'] = tar['dt'] + 0.1
        rt.append(obj)

    for b in range(B):
        if visitB[b]:
            continue
        tar = observed_data[b]
        obj = {}
        obj['state'] = [tar['x'], 
                        tar['y'], 
                        tar['theta'], 
                        ego['v'] + np.random.normal(0.0, 5.0), 
                        ego['ax'] + np.random.normal(0.0, 1.0),
                        ego['omega'] + np.random.normal(0.0, 0.01),
                        ego['v'],
                        ego['ax'],
                        ego['omega']]
        obj['reliability'] = 1
        obj['age'] = 1
        obj['id'] = hash_id
        obj['P'] = P_init
        obj['dt'] = 0.1
        hash_id += 1
        rt.append(obj)


for data_index in range(0,99):
    data_name = data_name_list[data_index]
    data_path = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + data_name + "/"
    state_path = data_path + "state/"
    save_path = data_path + "result/"

    st = seq_list[data_index][0]
    en = seq_list[data_index][1]
    N = en - st

    # load dataset
    observed_data = []
    ego_data = []
    for seq in range(st + 1, en + 1):
        state_file = state_path + str(seq).zfill(6)+".json"
        with open(state_file, "r") as st_json:
            state = json.load(st_json)
        
        obj_list = []
        for obj in state['objects']:
            if obj['valid']:
                obj_list.append(obj)
        observed_data.append(obj_list)

        ego_data.append({'v': state['v'], 'omega': state['omega'], 'ax': state['ax']})

    cur_objects_list = []
    # state: 9-dimension [xt,yt,theta_t, vt, axt, wt, vego, axego, wego]
    # F: hat(xt) = Fxt + Qt, F is jacobian matrix (EKF)
    # reliability(r):(1~5) Qt = Q/r, Rt = R/r
    # age
    # id
    hash_id = 0
    
    for seq in range(0, N):
        save_seq = st + seq + 1
        
        cur_objects_list, hash_id = kalman_update(cur_objects_list, observed_data[seq], ego_data[seq], hash_id)
        
        
        
        save_object_info(object_list, save_seq)