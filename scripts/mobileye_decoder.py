import os
import json
import datetime
import utils
from message import Message
from data import Data


author = "jeongwoooh"
email = "jeongwoo.oh@rllab.snu.ac.kr"
copy_right = "RLLAB@SNU"
date = datetime.datetime.now().strftime("%Y-%m-%d")
print(date)

project_path = os.path.dirname(os.path.abspath(os.path.dirname(__file__)))
input_path = project_path + "/data/first.trc"

raw_data = open(input_path, "r")
input_lines = raw_data.readlines()

messages= []
cnt = 0
for line in input_lines:
    cnt = cnt + 1
    raw_msg = line.split()
    if cnt == len(input_lines):
        break
    if not raw_msg[0].isdigit():
        continue
    if raw_msg[4] == "Tx":
        continue
    """
    msg = Message(raw_msg)
    
    msg = {"seq": int(raw_msg[0]),
           "time": float(raw_msg[1]),
           "type": int(raw_msg[3], base=16),
           "size": int(raw_msg[5])}
    data = []
    for i in range(msg['size']):
        d = []
        tmp =int(raw_msg[6+i], base=16)
        for j in range(8):
            d.append(tmp % 2)
            tmp = tmp / 2
        data.append(d)
    msg["data"] = data
    """
    messages.append(Message(raw_msg))

N = len(messages)
cur_seq = 0
cur_time = 0
data = []
data_unit = []
for i in range(N):
    if messages[i].type == utils.INIT:
        if cur_seq > 0:
            data.append(Data(cur_seq, cur_time, data_unit))
        
        cur_seq = cur_seq + 1
        cur_time = messages[i].time
        data_unit = []
    data_unit.append(messages[i])

N = len(data)

for i in range(N):
    data[i].decode_data()

max_obstacles = 0
for i in range(N):
    max_obstacles = max(max_obstacles, data[i].n_obstacles)
print(max_obstacles)

save_file = {}
save_file['author'] = author
save_file['email'] = email
save_file['copyright'] = copy_right
save_file['date'] = date
save_file['data'] = {'size': len(data),
                     'data': [d.get_data() for d in data]}

save_path = project_path + "/first_result.json"

with open(save_path, 'w') as outfile:
    json.dump(save_file, outfile, indent=4)
