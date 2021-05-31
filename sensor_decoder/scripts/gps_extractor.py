import rospy
import rospkg

import json

data_name = "gunmin_04"
result_path = rospkg.RosPack().get_path("sensor_decoder") + "/result/" + data_name + "/state/"

gps_data = []

offset = 10000
n_data = 5000

for i in range(offset, offset+n_data):
    if i % 10 != 0:
        continue
    path = result_path + str(i).zfill(6) + ".json"
    f = open(path)
    data = json.load(f)
    gps_data.append({'lat':data['vehicle_state']['x'], 'lng':data['vehicle_state']['y']})

save_path = result_path + "gps.txt"
f = open(save_path, 'w')
for d in gps_data:
    s = "{lat: %.9f, lng: %.9f }," %(d['lat'], d['lng'])
    f.write(s)

f.close()


