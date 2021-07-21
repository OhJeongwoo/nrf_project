import json
import os
import rospkg
import rospy
import numpy as np
from convex_hull import ConvexHull, get_clearance, merge_convex_hull
from convex_hull import Point
import math
from object_detector_tuning import pixel_to_xy, xy_to_pixel, build_grid_map, optimized_parameters
from PIL import Image, ImageDraw
import cv2
import operator
import time

data_name = "sumin_highway"
data_path = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + data_name + "/"
bin_path = data_path + "bin/"
state_path = data_path + "state/"
object_path = data_path + "object/"
check_path = data_path + "check/"
start_data = 5500
end_data = 8500
N = end_data -start_data
SVAL = 1e-3

minX = -20.0
maxX = 40.0
minY = -30.0
maxY = 30.0
minZ = 0.2
maxZ = 3.2

resolution = 0.1
grid_x = int((maxX-minX) / resolution)
grid_y = int((maxY-minY) / resolution)
density_threshold = 1
MAX_OBJECT = 20

dx = [1, -1, 0, 0, 2, -2, 0, 0, 1, 1, -1, -1]
dy = [0, 0, 1, -1, 0, 0, 2, -2, 1, -1, 1, -1]

loss_threshold = 100.0
distance_threshold = 0.3
font = cv2.FONT_HERSHEY_SIMPLEX
fontscale = 0.5
fontthickness = 2
fontline = cv2.LINE_AA

pointclouds = []
cnt = 0
init_time = time.time()

def get_x(lane, z):
    return lane['c0'] + lane['c1'] * z + lane['c2'] * z * z + lane['c3'] * z * z * z

for seq in range(start_data, end_data):
    cnt += 1
    start = time.time()
    pcs = np.fromfile(bin_path + str(seq).zfill(6) + ".bin", dtype=np.float32).reshape(-1, 4)
    x = pcs[:,0]
    y = pcs[:,1]
    z = pcs[:,2]
    intensity = pcs[:,3]
    with open(state_path+str(seq).zfill(6)+".json", "r") as st_json:
        state = json.load(st_json)
    
    
    indices = []
    for i in range(len(pcs)):
        check = False
        # remove ours
        if x[i] > -0.2 and x[i] < 5.0 and abs(y[i]) < 1.5:
            continue

        # filtering bev map
        if x[i] > minX and x[i] < maxX and y[i] > minY and y[i] < maxY and z[i] > minZ and z[i] < maxZ:
            # filtering out of road
            # for lane in state['lanes']:
            #     if abs(-get_x(lane, x[i]) - y[i]) < 2.5:
            #         check =True
            #         break 
            # if check:
            indices.append(i)
    pcs = pcs[indices,:]

    grid_map = build_grid_map(pcs)
    # layer = np.expand_dims(100*np.array(grid_map).astype('uint8'), axis = 0)
    # bev_map = np.transpose(np.vstack((layer, layer, layer)), (1,2,0))
    # bev_map = cv2.resize(bev_map, (600,600))
    # image.show()
    # clustering using bfs
    visited = [[False] * grid_y for i in range(grid_x)]
    clusters = []
    for i in range(grid_x):
        for j in range(grid_y):
            # print(i,j)
            # if j == 0 :
            #     visited_layer = np.expand_dims(255*np.array(visited).astype('uint8'), axis = 0)
            #     visited_map = np.transpose(np.vstack((visited_layer, visited_layer, visited_layer)), (1,2,0))
            #     visited_map = cv2.resize(visited_map, (600,600))
            #     cv2.imshow("map",visited_map)
            #     cv2.waitKey(20)
            
            if (grid_map[i][j] < density_threshold) or visited[i][j]:
                visited[i][j] = True
                continue
            queue = []
            queue.append([i,j])
            cluster = []
            while len(queue) > 0:
                cur = queue[0]
                queue.pop(0)
                cx = cur[0]
                cy = cur[1]
                if visited[cx][cy]:
                    continue
                visited[cx][cy] = True
                cluster.append([cx, cy])
                for k in range(12):
                    nx = cx + dx[k]
                    ny = cy + dy[k]
                    if nx < 0 or nx >= grid_x or ny < 0 or ny >= grid_y:
                        continue
                    if (grid_map[nx][ny] < density_threshold) or visited[nx][ny]:
                        visited[nx][ny] = True
                        continue
                    queue.append([nx, ny])
            clusters.append(cluster)

    cvh_list = []
    for cluster in clusters:
        if len(cluster) < 5:
            continue
        pts = []
        for pt in cluster:
            x,y = pixel_to_xy(pt[0], pt[1])
            pts.append(Point(x+(resolution-SVAL)/2, y+(resolution-SVAL)/2))
            pts.append(Point(x+(resolution-SVAL)/2, y-(resolution-SVAL)/2))
            pts.append(Point(x-(resolution-SVAL)/2, y+(resolution-SVAL)/2))
            pts.append(Point(x-(resolution-SVAL)/2, y-(resolution-SVAL)/2))
        # mismatch coordinate x,y and row,col
        # we should implement xy to pixel function
        

        cvh_list.append(ConvexHull(pts))
    # print(len(cvh_list))
    # xy_list = []
    # for cvh in cvh_list:
    #     xy = []
    #     x,y = cvh.get_center()
    #     x,y = xy_to_pixel(x,y)
    #     cv2.circle(bev_map, (y, x), 20, (255,255,255),2)
    #     for point in cvh.points:
    #         x,y = xy_to_pixel(point.x, point.y)
    #         xy.append((x,y))
    #     xy_list.append(xy)
    # # cv2.circle(bev_map, (100, 100), 50, (255,255,255), 2)
    # for xy in xy_list:
    #     N = len(xy)
    #     for i in range(N):
    #         s = xy[i]
    #         e = xy[(i+1)%N]
    #         cv2.line(bev_map, (s[1], s[0]), (e[1], e[0]), (255,0,0), 10) 
    # cv2.imshow("map",bev_map)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    
    
    
    K = len(cvh_list)
    # print(K)
    adj = [[] for i in range(K)]
    for i in range(K):
        for j in range(K):
            if i == j :
                continue
            if get_clearance(cvh_list[i], cvh_list[j]) < distance_threshold:
                adj[i].append(j)


    cvh_visited = [False for i in range(K)]
    cvh_clusters = []
    for i in range(K):
        if cvh_visited[i]:
            continue
        queue = []
        queue.append(i)
        cluster = []
        while len(queue) > 0:
            cur = queue[0]
            queue.pop(0)
            if cvh_visited[cur]:
                continue
            cvh_visited[cur] = True
            cluster.append(cur)
            for next in adj[cur]:
                if cvh_visited[next]:
                    continue
                queue.append(next) 
        cvh_clusters.append(cluster)

    
    

    objects = []
    for cluster in cvh_clusters:
        solution, loss = optimized_parameters(merge_convex_hull([cvh_list[i] for i in cluster]))
        solution['valid'] = False
        if solution['l'] > 15.0:
            continue
        if solution['w'] > 5.0:
            continue
        # if solution['loss']['box'] > 0.1:
        #     solution['valid'] = False
        # if loss < loss_threshold:
        solution['value'] = abs(solution['x']) * 0.5 + abs(solution['y'])
        if True:
            objects.append(solution)
    # print(objects)
    objects = sorted(objects, key=operator.itemgetter('value'))
    objects = objects[0:min(MAX_OBJECT,len(objects))]

    # xy_list = []
    # # print(len(cvh_clusters))
    # for cluster in cvh_clusters:
    #     cvh = merge_convex_hull([cvh_list[i] for i in cluster])
    #     xy = []
    #     for point in cvh.points:
    #         x,y = xy_to_pixel(point.x, point.y)
    #         xy.append((x,y))
    #     xy_list.append(xy)
    # # cv2.circle(bev_map, (100, 100), 50, (255,255,255), 2)
    # for xy in xy_list:
    #     N = len(xy)
    #     for i in range(N):
    #         s = xy[i]
    #         e = xy[(i+1)%N]
    #         cv2.line(bev_map, (s[1], s[0]), (e[1], e[0]), (255,0,0), 1) 

    # for object in objects:
    #     cx = object['x']
    #     cy = object['y']
    #     l = object['l']
    #     w = object['w']
    #     theta = object['theta']
        # box = []
        # box.append(xy_to_pixel(cx + l/2 * math.cos(theta) - w/2 * math.sin(theta), cy + l/2 * math.sin(theta) + w/2 * math.cos(theta)))
        # box.append(xy_to_pixel(cx - l/2 * math.cos(theta) - w/2 * math.sin(theta), cy - l/2 * math.sin(theta) + w/2 * math.cos(theta)))
        # box.append(xy_to_pixel(cx - l/2 * math.cos(theta) + w/2 * math.sin(theta), cy - l/2 * math.sin(theta) - w/2 * math.cos(theta)))
        # box.append(xy_to_pixel(cx + l/2 * math.cos(theta) + w/2 * math.sin(theta), cy + l/2 * math.sin(theta) - w/2 * math.cos(theta)))
        # box.append(xy_to_pixel(cx, cy))
        # cv2.line(bev_map, (box[0][1], box[0][0]), (box[1][1], box[1][0]), (0, 255, 0 ), 1)
        # cv2.line(bev_map, (box[1][1], box[1][0]), (box[2][1], box[2][0]), (0, 255, 0 ), 1)
        # cv2.line(bev_map, (box[2][1], box[2][0]), (box[3][1], box[3][0]), (0, 255, 0 ), 1)
        # cv2.line(bev_map, (box[3][1], box[3][0]), (box[0][1], box[0][0]), (0, 255, 0 ), 1)
        # cv2.putText(bev_map, format(object['loss']['box'],".2f"), (box[4][1], box[4][0]), font, fontscale, (255, 255, 255), fontthickness, fontline)
    
    # cv2.imshow("map",bev_map)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    # cv2.imwrite(check_path + str(seq).zfill(6) + ".png", bev_map)

    state['objects'] = objects
    state['n_objects'] = len(objects)
    save_path = state_path + str(seq).zfill(6) + ".json"
    with open(save_path, 'w') as outfile:
        json.dump(state, outfile, indent=4)

    end = time.time()
    print("[%d / %d]" %(cnt, N))
    print("elapsed time : %.2f" %(end - start))
    print("total elapsed time : %.2f" %(end - init_time))
    avg_time = (end-init_time) / cnt
    print("expected remaining time : %.2f" %(avg_time * (N-cnt)))
    print("========================================================================")

