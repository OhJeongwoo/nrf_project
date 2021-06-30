import numpy as np
import cv2
import json
import math
import time


def get_pixel(x, y):
    px = int((x - maxX) / (minX - maxX) * bev_height)
    py = int((y - maxY) / (minY - maxY) * bev_width)

    return px, py

bin_path = "000001.bin"
minX = -20.0
maxX = 40.0
minY = -30.0
maxY = 30.0
minZ = 1.0
maxZ = 3.5
resolution = 0.1
bev_height = 600
bev_width = 600
cx = (minX + maxX) / 2.0
cy = (minY + maxY) / 2.0

maxlogDensity = math.log(64)

pcs = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)
x = pcs[:,0]
y = pcs[:,1]
z = pcs[:,2]
intensity = pcs[:,3]
x_filter = np.logical_and((x>minX), (x<maxX))
y_filter = np.logical_and((y>minY), (y<maxY))
z_filter = np.logical_and((z>minZ), (z<maxZ))
filter = np.logical_and(x_filter, y_filter, z_filter)
indices = np.argwhere(filter).flatten()
x = x[indices]
y = y[indices]
z = z[indices]
intensity = intensity[indices]
n_points = len(intensity)


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
bev_map = np.transpose(np.vstack((intensity_layer, density_layer, height_layer)), (1,2,0))
bev_map = cv2.resize(bev_map, (bev_height, bev_width))

cv2.imshow("map",bev_map)
cv2.waitKey(0)
cv2.destroyAllWindows()