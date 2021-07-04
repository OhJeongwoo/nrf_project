import json
import os
import rospkg
import rospy
import numpy as np
from convex_hull import ConvexHull
from convex_hull import Point
import math
from object_detector_tuning import build_grid_map
from object_detector_tuning import pixel_to_xy
from object_detector_tuning import optimized_parameters

data_name = "object_detector"
data_path = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + data_name + "/"
bin_path = data_path + "bin/"
object_path = data_path + "object/"
N = 10000 # # of data

minX = -20.0
maxX = 40.0
minY = -30.0
maxY = 30.0
minZ = 0.5
maxZ = 2.5

resolution = 0.1
grid_x = int((maxX-minX) / resolution)
grid_y = int((maxY-minY) / resolution)

dx = [1, -1, 0, 0]
dy = [0, 0, 1, -1]

loss_threshold = 100.0

pointclouds = []
for seq in range(N):
    pcs = np.fromfile(bin_path + str(seq).zfill(6) + ".bin", dtype=np.float32).reshape(-1, 4)
    x = pcs[:,0]
    y = pcs[:,1]
    z = pcs[:,2]
    intensity = pcs[:,3]
    x_filter = np.logical_and((x>minX), (x<maxX))
    y_filter = np.logical_and((y>minY), (y<maxY))
    z_filter = np.logical_and((z>minZ), (z<maxZ))
    filter = np.logical_and(x_filter, y_filter, z_filter)
    indices = np.argwhere(filter).flatten()
    pcs = pcs[indices,:]

    grid_map = build_grid_map(pcs)
    # clustering using bfs
    visited = [[False] * grid_y for i in range(grid_x)]
    clusters = []
    for i in range(grid_x):
        for j in range(grid_y):
            if not grid_map[i][j] or visited[i][j]:
                visited[i][j] = True
                continue
            queue = []
            queue.append([i,j])
            cluster = []
            while True:
                cur = queue[0]
                queue.pop(0)
                cx = cur[0]
                cy = cur[1]
                if visited[cx][cy]:
                    continue
                visited[cx][cy]
                cluster.append([cx, cy])
                for k in range(4):
                    nx = cx + dx[k]
                    ny = cy + dy[k]
                    if nx < 0 or nx >= grid_x or ny < 0 or ny >= grid_y:
                        continue
                    if not grid_map[nx][ny] or visited[nx][ny]:
                        visited[nx][ny] = True
                        continue
                    queue.append([nx, ny])
                clusters.append(cluster)

    objects = []
    for cluster in clusters:
        pts = []
        for pt in cluster:
            x,y = pixel_to_xy(pt[0], pt[1])
            pts.append(Point(x+resolution/2, y+resolution/2))
            pts.append(Point(x+resolution/2, y+resolution/2))
            pts.append(Point(x+resolution/2, y+resolution/2))
            pts.append(Point(x+resolution/2, y+resolution/2))

        solution, loss = optimized_parameters(ConvexHull(pts))
        if loss < loss_threshold:
            objects.append(objects)
    
    save_path = object_path + str(seq).zfill(6) + ".json"
    with open(save_path, 'w') as outfile:
        json.dump(objects, outfile, indent=4)

    
    # for each cluster generate convex hull and optimize parameters