import rospy
import rospkg

import numpy as np
import cv2
import json
import math
import time

data_name = "sumin_highway"
data_path = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + data_name + "/"
state_path = data_path + "state/"

start_data = 1
end_data = 10344
# invalid = (8933, 9409)


prev_state = None

n_valid = 0

valid_list = []

for seq in range(start_data, end_data):
    print(seq)
    # if seq in range(invalid[0], invalid[1]):
    #     continue
    with open(state_path+str(seq).zfill(6)+".json", "r") as st_json:
        state = json.load(st_json)
    
    state['data_name'] = data_name
    # add lateral deviation
    dev_list = []
    for lane in state['lanes']:
        dev_list.append(lane['c0'])
    dev_list = sorted(dev_list, key = abs)
    state['target_deviation'] = (dev_list[0] + dev_list[1]) / 2.0

    # revise decision
    # if v is small, decision turns into stop(4)
    # if v is not small but decision is stop(4), decision turns into keep lane(1)
    if state['v'] < 1.0:
        state['decision'] = 4
    else :
        if state['decision'] == 4:
            state['decision'] = 1

    # add tunnel attributes
    # if v is not small and location doesn't change, is_tunnel is true
    if prev_state is None:
        state['is_tunnel'] = False
    else :
        distance = math.sqrt((prev_state['x'] - state['x']) ** 2 + (prev_state['y'] - state['y']) ** 2)
        if state['v'] > 5.0 and distance < 1e-7:
            state['is_tunnel'] = True
            print("[WARN] %d-th seq is in a tunnel" %(seq))
        else :
            state['is_tunnel'] = False
            n_valid += 1
            valid_list.append(seq)


    with open(state_path+str(seq).zfill(6)+".json", 'w') as outfile:
            json.dump(state, outfile, indent=4)

    prev_state = state

print(n_valid)
f = open(data_path + "valid.txt", 'w')
for v in valid_list:
    data = "%d\n" % v
    f.write(data)
f.close()

