#!/usr/bin/python
import rospy
import rospkg

from sensor_decoder.msg import Data
from sensor_decoder.msg import Object
from sensor_decoder.msg import Label
from sensor_msgs.msg import Image

import numpy as np
import math
import cv2
from cv_bridge import CvBridge
import copy

# minX = -10.0
# maxX = 20.0
# minY = -10.0
# maxY = 10.0
# minZ = 0.5
# maxZ = 2.0
resolution = 0.1
bev_height = 600
bev_width = 600

"""
For bird eye view map
"""
minX = -20.0
maxX = 40.0
minY = -30.0
maxY = 30.0
minZ = 0.5
maxZ = 2.0

cx = (minX + maxX) / 2.0
cy = (minY + maxY) / 2.0

maxlogDensity = math.log(20)
"""
For local map
"""
font = cv2.FONT_HERSHEY_SIMPLEX
fontscale = 0.5
fontthickness = 2
fontline = cv2.LINE_AA

def get_pixel(x, y):
    px = int((x - maxX) / (minX - maxX) * bev_height)
    py = int((y - maxY) / (minY - maxY) * bev_width)

    return px, py

class QueryVizTool:
    def __init__(self):
        self.pub = rospy.Publisher("/query_image", Image, queue_size=10)
        self.seq = 0
        self.bin_path = ""
        self.bridge = CvBridge()
        self.valid_data = False
        self.valid_label = False

        self.sub_data = rospy.Subscriber("/query_data", Data, self.callback_data)
        self.sub_label = rospy.Subscriber("/query_label", Label, self.callback_label)

    def draw_object(self, objects):
        local_map = self.bev_map
        for i in range(len(objects)):
            cx = objects[i].x
            cy = objects[i].y
            l = objects[i].l
            w = objects[i].w
            theta = objects[i].theta
            box = []
            box.append(get_pixel(cx + l/2 * math.cos(theta) - w/2 * math.sin(theta), cy + l/2 * math.sin(theta) + w/2 * math.cos(theta)))
            box.append(get_pixel(cx - l/2 * math.cos(theta) - w/2 * math.sin(theta), cy - l/2 * math.sin(theta) + w/2 * math.cos(theta)))
            box.append(get_pixel(cx - l/2 * math.cos(theta) + w/2 * math.sin(theta), cy - l/2 * math.sin(theta) - w/2 * math.cos(theta)))
            box.append(get_pixel(cx + l/2 * math.cos(theta) + w/2 * math.sin(theta), cy + l/2 * math.sin(theta) - w/2 * math.cos(theta)))
            box.append(get_pixel(cx, cy))
            if objects[i].valid:
                cv2.line(local_map, (box[0][1], box[0][0]), (box[1][1], box[1][0]), (0, 0, 255), 2)
                cv2.line(local_map, (box[1][1], box[1][0]), (box[2][1], box[2][0]), (0, 0, 255), 2)
                cv2.line(local_map, (box[2][1], box[2][0]), (box[3][1], box[3][0]), (0, 0, 255), 2)
                cv2.line(local_map, (box[3][1], box[3][0]), (box[0][1], box[0][0]), (0, 255, 255), 2)
                cv2.putText(local_map, format(objects[i].id,"d"), (box[4][1], box[4][0]), font, fontscale, (255, 255, 255), fontthickness, fontline)
            else :
                if not self.only_valid :    
                    cv2.line(local_map, (box[0][1], box[0][0]), (box[1][1], box[1][0]), (255, 0, 0), 1)
                    cv2.line(local_map, (box[1][1], box[1][0]), (box[2][1], box[2][0]), (255, 0, 0), 1)
                    cv2.line(local_map, (box[2][1], box[2][0]), (box[3][1], box[3][0]), (255, 0, 0), 1)
                    cv2.line(local_map, (box[3][1], box[3][0]), (box[0][1], box[0][0]), (255, 255, 0), 1)
                    cv2.putText(local_map, format(objects[i].id,"d"), (box[4][1], box[4][0]), font, fontscale, (255, 255, 255), fontthickness, fontline)
  
        self.local_map = local_map

    def callback_data(self, msg):
        # generate bev map & image
        # build bev map
        self.data_name = msg.name
        self.seq = msg.seq

        self.data_path = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + self.data_name + "/"
        self.img_path = self.data_path + "image_raw/"
        self.bin_path = self.data_path + "bin/"

        self.bin_file = self.bin_path + str(self.seq).zfill(6) + ".bin"
        self.img_file = self.img_path + str(self.seq).zfill(6) + ".png"
        self.img_raw = cv2.imread(self.img_file, cv2.COLOR_BGR2RGB)
        pcs = np.fromfile(self.bin_file, dtype=np.float32).reshape(-1, 4)
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

        self.bev_map = local_map

        self.valid_data = True

    def callback_label(self, msg):
        # update local map
        self.only_valid = msg.only_valid
        self.draw_object(msg.objects)
        self.valid_label = True


    def query(self):
        if not self.valid_data or not self.valid_label :
            return
        img = cv2.hconcat([cv2.resize(self.img_raw, dsize=(600,600)), self.local_map])
        self.pub.publish(self.bridge.cv2_to_imgmsg(img))
    



if __name__=='__main__':
    rospy.init_node("query_viz_tool", anonymous=True)
    q = QueryVizTool()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        q.query()
        rate.sleep()