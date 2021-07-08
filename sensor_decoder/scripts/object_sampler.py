import random
import math
import rospkg
import json
import numpy as np
import os
import time

from convex_hull import Point
from convex_hull import ConvexHull

Xmin = 10.0
Xmax = 30.0
Ymin = 10.0
Ymax = 20.0
l_g = 5.0
w_g = 2.5

data_name = "object_detector"
data_path = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + data_name + "/"
bin_path = data_path + "bin/"
label_path = data_path + "labels/"
N = 100
M = 1000

if not os.path.exists(data_path):
    os.mkdir(data_path)
if not os.path.exists(bin_path):
    os.mkdir(bin_path)
if not os.path.exists(label_path):
    os.mkdir(label_path)


step_size = 0.03
maxstep = 1700

def rotation(x, y, theta):
    ct = math.cos(theta)
    st = math.sin(theta)
    return x * ct - y * st, x * st + y * ct

init_time = time.time()
for seq in range(N):
    x = random.uniform(Xmin, Xmax)
    y = random.uniform(Ymin, Ymax)
    theta = random.uniform(-math.pi/2, math.pi/2)
    l = random.uniform(l_g - 2.0, l_g + 2.0)
    w = random.uniform(w_g - 0.5, w_g + 0.5)
    label = {'x': x, 'y': y, 'theta': theta, 'l': l, 'w': w}
    save_path = label_path + str(seq).zfill(6) + ".json"
    with open(save_path, 'w') as outfile:
        json.dump(label, outfile, indent=4)
    pts = []
    minTheta = math.pi/2
    maxTheta = 0.0
    px, py = rotation(l/2, w/2, theta)
    pts.append(Point(x+px, y+py))
    t = math.atan((y+py)/(x+px))
    if t < minTheta:
        minTheta = t
    if t > maxTheta:
        maxTheta = t
    px, py = rotation(l/2, -w/2, theta)
    pts.append(Point(x+px, y+py))
    t = math.atan((y+py)/(x+px))
    if t < minTheta:
        minTheta = t
    if t > maxTheta:
        maxTheta = t
    px, py = rotation(-l/2, w/2, theta)
    pts.append(Point(x+px, y+py))
    t = math.atan((y+py)/(x+px))
    if t < minTheta:
        minTheta = t
    if t > maxTheta:
        maxTheta = t
    px, py = rotation(-l/2, -w/2, theta)
    pts.append(Point(x+px, y+py))
    t = math.atan((y+py)/(x+px))
    if t < minTheta:
        minTheta = t
    if t > maxTheta:
        maxTheta = t
    cvh = ConvexHull(pts)
    pointclouds = []
    iter = 0
    while iter < M:
        beam_theta = random.uniform(minTheta, maxTheta)
        d = 0.0
        hit = False
        while d < 50.0:
            d += step_size
            point = Point(d * math.cos(beam_theta), d * math.sin(beam_theta))
            if cvh.is_include(point):
                pointclouds.append(Point(point.x + np.random.normal(0,0.03), point.y + np.random.normal(0, 0.03)))
                hit = True
                break
        if hit:
            iter += 1
    m = len(pointclouds)
    rt = np.zeros(m * 4, dtype=np.float32)
    for i in range(m):
        rt[i*4] = pointclouds[i].x
        rt[i+4+1] = pointclouds[i].y
    rt.astype('float32').tofile(bin_path + str(seq).zfill(6) + '.bin')
    save_time = time.time()
    remain_time = (save_time - init_time) / (seq + 1) * (N - seq -1)
    print("[%.2f]Complete to save %d-th data, expected remain time: %.2f" %(save_time - init_time, seq, remain_time))
