import rospy
import rospkg
import json
import time
from seq import data_name_list
from seq import seq_list



N = len(data_name_list)

start = time.time()
end = time.time()
data_index_list = [i for i in range(101,382)]

for i in data_index_list:
    data_name = data_name_list[i]
    print("[%d / %d, %.2f] Start to object processing, data name is %s" %(i+1, N, end - start, data_name))
    data_path = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + data_name + "/"
    obj_path = data_path + "object/"
    state_path = data_path + "state/"
    st = seq_list[i][0]
    en = seq_list[i][1]
    M = en - st
    for j in range(1, M + 1):
        seq = j + st
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
        
        if j % 1000 == 0:
            print("[%d / %d, %.2f] In progress to object processing" %(j, M, end-start))



