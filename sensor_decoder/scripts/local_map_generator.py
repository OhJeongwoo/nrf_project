import rospy
import rospkg

import numpy as np
import cv2
import json
import math
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

"""
For bird eye view map
"""
minX = -20.0
maxX = 40.0
minY = -30.0
maxY = 30.0
minZ = 0.5
maxZ = 2.0

resolution = 1.0
bev_height = 60
bev_width = 60
cx = (minX + maxX) / 2.0
cy = (minY + maxY) / 2.0

maxlogDensity = math.log(20)
"""
For local map
"""
n_marked_lane = 200
v_clip = 20.0
ax_clip = 5.0
ay_clip = 5.0
w_clip = 0.5
dev_clip = 4.0
font = cv2.FONT_HERSHEY_SIMPLEX
fontscale = 0.5
fontthickness = 2
fontline = cv2.LINE_AA

def get_pixel(x, y):
    px = int((x - maxX) / (minX - maxX) * bev_height)
    py = int((y - maxY) / (minY - maxY) * bev_width)

    return px, py


def get_decision(type):
    if type == 1:
        return "KEEP LANE"
    if type == 2:
        return "CHANGE LEFT"
    if type == 3:
        return "CHANGE RIGHT"
    if type == 4:
        return "STOP"
    return "ERROR"

def clip(value, limit):
    """
    clip value by given limit and rescale to [-1,1]
    """
    if abs(value) > abs(limit):
        return value / abs(value)
    else:
        return value / limit

def get_x(lane, z):
    return lane['c0'] + lane['c1'] * z + lane['c2'] * z * z + lane['c3'] * z * z * z


def draw_lanes(image, lanes):
    for lane in lanes:
        for i in range(-n_marked_lane, 2*n_marked_lane):
            px, py = get_pixel(0.1*i, -get_x(lane,0.1*i))
            cv2.circle(image, (py, px), 1, (0,255,255), -1)

def draw_object(image, objects):
    for i in range(len(objects)):
        cx = objects[i]['x']
        cy = objects[i]['y']
        l = objects[i]['l']
        w = objects[i]['w']
        theta = objects[i]['theta']
        box = []
        box.append(get_pixel(cx + l/2 * math.cos(theta) - w/2 * math.sin(theta), cy + l/2 * math.sin(theta) + w/2 * math.cos(theta)))
        box.append(get_pixel(cx - l/2 * math.cos(theta) - w/2 * math.sin(theta), cy - l/2 * math.sin(theta) + w/2 * math.cos(theta)))
        box.append(get_pixel(cx - l/2 * math.cos(theta) + w/2 * math.sin(theta), cy - l/2 * math.sin(theta) - w/2 * math.cos(theta)))
        box.append(get_pixel(cx + l/2 * math.cos(theta) + w/2 * math.sin(theta), cy + l/2 * math.sin(theta) - w/2 * math.cos(theta)))
        box.append(get_pixel(cx, cy))
        cv2.line(image, (box[0][1], box[0][0]), (box[1][1], box[1][0]), (0, 0, 255), 3)
        cv2.line(image, (box[1][1], box[1][0]), (box[2][1], box[2][0]), (0, 0, 255), 3)
        cv2.line(image, (box[2][1], box[2][0]), (box[3][1], box[3][0]), (0, 0, 255), 3)
        cv2.line(image, (box[3][1], box[3][0]), (box[0][1], box[0][0]), (0, 255, 255), 3)
        cv2.putText(image, format(objects[i]['id'],"d"), (box[4][1], box[4][0]), font, fontscale, (255, 255, 255), fontthickness, fontline)
        cv2.putText(image, str(objects[i]['v']), (box[4][1], box[4][0]+20), font, fontscale, (255, 255, 255), fontthickness, fontline)

# data_list = [1]
# data_list = [2]
# data_list = [3, 4]
# data_list = [5]
# data_list = [6]
# data_list = [96]
# data_list = [97]
data_list = [98]
# data_list = [99]
# data_list = []
# for i in range(7,96):
#     data_list.append(i)
total_images = 0
for data_index in data_list:
    total_images += seq_list[data_index][1] - seq_list[data_index][0]

cur_images = 0
init_time = time.time()

for data_index in data_list:
    data_name = data_name_list[data_index]
    data_path = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + data_name + "/"
    
    state_path = data_path + "new_state/"
    bin_path = data_path + "bin/"
    bev_path = data_path + "bev_map/"
    local_map_path = data_path + "local_map/"

    valid_path = data_path + "valid.txt"

    st = seq_list[data_index][0]
    en = seq_list[data_index][1]
    seqs = []
    for i in range(st+1, en+1):
        seqs.append(i)
    # f = open(valid_path, 'r')
    # while True:
    #     line = f.readline()
    #     if not line : break
    #     seq_list.append(int(line))
    # f.close()

    # n_data = 20000 # number of data
    # start_data = 13000
    # end_data = 16000

    


        


    cnt = 0
    N = len(seqs)
    for seq in seqs:
        start = time.time()
        cnt += 1
        cur_images += 1
        # load bin file
        pcs = np.fromfile(bin_path + str(seq).zfill(6) + ".bin", dtype=np.float32).reshape(-1, 4)

        # load state file
        with open(state_path+str(seq).zfill(6)+".json", "r") as st_json:
            state = json.load(st_json)
        
        # calculate deviation
        dev_list = []
        for lane in state['lanes']:
            dev_list.append(lane['c0'])
        dev_list = sorted(dev_list, key = abs)
        state['target_deviation'] = (dev_list[0] + dev_list[1]) / 2.0
        

        # build bev map
        x = pcs[:,0]
        y = pcs[:,1]
        z = pcs[:,2]
        intensity = pcs[:,3]
        
        indices = []
        for i in range(len(pcs)):
            if x[i] > minX and x[i] < maxX and y[i] > minY and y[i] < maxY and z[i] > minZ and z[i] < maxZ:
                indices.append(i)
        pcs = pcs[indices,:]
        x = x[indices]
        y = y[indices]
        z = z[indices]
        intensity = intensity[indices]
        n_points = len(intensity)

        # intensity_layer = np.zeros([bev_height, bev_width], dtype=np.float)
        # density_layer = np.zeros([bev_height, bev_width], dtype=np.float)
        # height_layer = np.zeros([bev_height, bev_width], dtype=np.float)
        # for i in range(n_points):
        #     px, py = get_pixel(x[i], y[i])
        #     if px < 0 or px >= bev_height or py < 0 or py >= bev_width:
        #         continue
        #     intensity_layer[px][py] = max(intensity_layer[px][py], intensity[i])
        #     density_layer[px][py] += 1
        #     height_layer[px][py] = max(height_layer[px][py], (z[i]-minZ)/ (maxZ-minZ))
        # for i in range(bev_height):
        #     for j in range(bev_width):
        #         density_layer[px][py] = min(1.0, math.log(1 + density_layer[px][py]) / maxlogDensity)
        # intensity_layer = intensity_layer * 255.0
        # density_layer = density_layer * 255.0
        # height_layer = height_layer * 255.0
        # intensity_layer = np.expand_dims(intensity_layer.astype('uint8'), axis = 0)
        # density_layer = np.expand_dims(density_layer.astype('uint8'), axis = 0)
        # height_layer = np.expand_dims(height_layer.astype('uint8'), axis = 0)
        # bev_map = np.transpose(np.vstack((intensity_layer, density_layer, height_layer)), (1,2,0))
        # bev_map = cv2.resize(bev_map, (bev_height, bev_width))
        # cv2.imwrite(bev_path + str(seq).zfill(6) + ".png", bev_map)

        
        resolution = 0.1
        bev_height = 600
        bev_width = 600
        maxlogDensity = math.log(20)

        intensity_layer = np.zeros([bev_height, bev_width], dtype=np.float)
        density_layer = np.zeros([bev_height, bev_width], dtype=np.float)
        height_layer = np.zeros([bev_height, bev_width], dtype=np.float)
        for i in range(n_points):
            px, py = get_pixel(x[i], y[i])
            if px < 0 or px >= bev_height or py < 0 or py >= bev_width:
                continue
            intensity_layer[px][py] = max(intensity_layer[px][py], intensity[i])
            density_layer[px][py] += 1
            height_layer[px][py] = max(height_layer[px][py], (z[i]-minZ)/ (maxZ-minZ))
        for i in range(bev_height):
            for j in range(bev_width):
                density_layer[px][py] = min(1.0, math.log(1 + density_layer[px][py]) / maxlogDensity)
        intensity_layer = intensity_layer * 255.0
        density_layer = density_layer * 255.0
        height_layer = height_layer * 255.0
        intensity_layer = np.expand_dims(intensity_layer.astype('uint8'), axis = 0)
        density_layer = np.expand_dims(density_layer.astype('uint8'), axis = 0)
        height_layer = np.expand_dims(height_layer.astype('uint8'), axis = 0)
        local_map = np.transpose(np.vstack((intensity_layer, density_layer, height_layer)), (1,2,0))
        local_map = cv2.resize(local_map, (bev_height, bev_width))

        # ours
        cv2.rectangle(local_map, (290, 400), (310, 445), (0, 255, 0), 2)

        draw_lanes(local_map, state['lanes'])

        draw_object(local_map, state['filtered_objects'])

        # draw heading indicator
        cv2.circle(local_map, (100, 100), 50, (255,255,255), 2)
        cv2.line(local_map, (100,100), (int(100+30.0*math.cos(state['theta'])),int(100+30.0*math.sin(state['theta']))), (255, 255, 255), 3)
        
        # draw box for v, ax, ay, omega, lateral deviation indicator
        cv2.putText(local_map, "v", (20, 200), font, fontscale, (255, 255, 255), fontthickness, fontline)
        cv2.putText(local_map, "ax", (20, 240), font, fontscale, (255, 255, 255), fontthickness, fontline)
        cv2.putText(local_map, "ay", (20, 280), font, fontscale, (255, 255, 255), fontthickness, fontline)
        cv2.putText(local_map, "w", (20, 320), font, fontscale, (255, 255, 255), fontthickness, fontline)
        cv2.putText(local_map, "dev", (20, 360), font, fontscale, (255, 255, 255), fontthickness, fontline)

        cv2.rectangle(local_map, (50, 180), (150, 200), (255, 255, 255), 2)
        cv2.rectangle(local_map, (50, 220), (150, 240), (255, 255, 255), 2)
        cv2.rectangle(local_map, (50, 260), (150, 280), (255, 255, 255), 2)
        cv2.rectangle(local_map, (50, 300), (150, 320), (255, 255, 255), 2)
        cv2.rectangle(local_map, (50, 340), (150, 360), (255, 255, 255), 2)

        cv2.line(local_map, (int(100 + 50 * clip(state['v'], v_clip)), 180), (int(100 + 50 * clip(state['v'], v_clip)), 200), (255, 0, 0), 2)
        cv2.line(local_map, (int(100 + 50 * clip(state['ax'], ax_clip)), 220), (int(100 + 50 * clip(state['ax'], ax_clip)), 240), (255, 0, 0), 2)
        cv2.line(local_map, (int(100 + 50 * clip(state['ay'], ay_clip)), 260), (int(100 + 50 * clip(state['ay'], ay_clip)), 280), (255, 0, 0), 2)
        cv2.line(local_map, (int(100 + 50 * clip(state['omega'], w_clip)), 300), (int(100 + 50 * clip(state['omega'], w_clip)), 320), (255, 0, 0), 2)
        cv2.line(local_map, (int(100 + 50 * clip(state['target_deviation'], dev_clip)), 340), (int(100 + 50 * clip(state['target_deviation'], dev_clip)), 360), (255, 0, 0), 2)

        # write state info on the right side (x, y, theta, v, ax, ay, omega, decision, current deviation, target deviation)
        cv2.putText(local_map, "lat      : " + str(state['x']), (400, 50), font, fontscale, (255, 255, 255), fontthickness, fontline)
        cv2.putText(local_map, "lng      : " + str(state['y']), (400, 80), font, fontscale, (255, 255, 255), fontthickness, fontline)
        cv2.putText(local_map, "theta    : " + str(state['theta']), (400, 110), font, fontscale, (255, 255, 255), fontthickness, fontline)
        cv2.putText(local_map, "vel      : " + str(state['v']), (400, 140), font, fontscale, (255, 255, 255), fontthickness, fontline)
        cv2.putText(local_map, "ax       : " + str(state['ax']), (400, 170), font, fontscale, (255, 255, 255), fontthickness, fontline)
        cv2.putText(local_map, "ay       : " + str(state['ay']), (400, 200), font, fontscale, (255, 255, 255), fontthickness, fontline)
        cv2.putText(local_map, "omega    : " + str(state['omega']), (400, 230), font, fontscale, (255, 255, 255), fontthickness, fontline)
        cv2.putText(local_map, "dev      : " + str(state['target_deviation']), (400, 260), font, fontscale, (255, 255, 255), fontthickness, fontline)
        cv2.putText(local_map, "decision : " + get_decision(state['decision']), (400, 290), font, fontscale, (255, 255, 255), fontthickness, fontline)


        # write data info on the left below corner (seq, data name, date)
        cv2.putText(local_map, "seq       : " + str(state['seq']), (30, 500), font, fontscale, (255, 255, 255), fontthickness, fontline)
        cv2.putText(local_map, "data      : " + data_name, (30, 530), font, fontscale, (255, 255, 255), fontthickness, fontline)
        cv2.putText(local_map, "date      : " + state['date'], (30, 560), font, fontscale, (255, 255, 255), fontthickness, fontline)
        cv2.putText(local_map, "copyright : " + state['copy_right'], (30, 590), font, fontscale, (255, 255, 255), fontthickness, fontline)

        cv2.imwrite(local_map_path + str(seq).zfill(6) + ".png", local_map)

        end = time.time()
        print("[TOT %d / %d]" %(cur_images, total_images))
        print("[CUR %d / %d]" %(cnt, N))
        print("elapsed time : %.2f" %(end - start))
        print("total elapsed time : %.2f" %(end - init_time))
        avg_time = (end-init_time) / cur_images
        print("expected remaining time : %.2f" %(avg_time * (total_images - cur_images)))
        print("========================================================================")
