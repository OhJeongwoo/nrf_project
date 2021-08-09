import rospy
import rospkg

import numpy as np
import cv2
import json
import math
import time

data_name = "0729_neg_jeongwoo_03_1"
data_path = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + data_name + "/"
state_path = data_path + "new_state/"
bin_path = data_path + "bin/"
bev_path = data_path + "bev_map/"
local_map_path = data_path + "local_map/"

valid_path = data_path + "valid.txt"

seq_list = []
for i in range(1,101):
    seq_list.append(i)
# f = open(valid_path, 'r')
# while True:
#     line = f.readline()
#     if not line : break
#     seq_list.append(int(line))
# f.close()

# n_data = 20000 # number of data
# start_data = 13000
# end_data = 16000

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
    

init_time = time.time()
cnt = 0
N = len(seq_list)
for seq in seq_list:
    start = time.time()
    cnt += 1
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
    print("[%d / %d]" %(cnt, N))
    print("elapsed time : %.2f" %(end - start))
    print("total elapsed time : %.2f" %(end - init_time))
    avg_time = (end-init_time) / cnt
    print("expected remaining time : %.2f" %(avg_time * (N-cnt)))
    print("========================================================================")
