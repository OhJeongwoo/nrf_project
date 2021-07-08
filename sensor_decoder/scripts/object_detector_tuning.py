import json
import os
import rospkg
import rospy
import numpy as np
from convex_hull import ConvexHull
from convex_hull import Point
from convex_hull import box_convex_hull
from convex_hull import calculate_IOU
import math
import matplotlib.pyplot as plt

#### hyperparameters ####
data_name = "test3"
data_path = rospkg.RosPack().get_path("sensor_decoder") + "/data/" + data_name + "/"
bin_path = data_path + "bin/"
label_path = data_path + "labels/"
N = 8000 # # of data

l_coeff = 0.1
w_coeff = 0.1
l_size_coeff = 0.1
w_size_coeff = 0.1
box_coeff = 1.0
l_g = 5.0
w_g = 2.5
lr_init = 0.03
decay_k = 0.05
loss_threshold = 1e-3

minX = -20.0
maxX = 40.0
minY = -30.0
maxY = 30.0

resolution = 0.1
grid_x = int((maxX-minX) / resolution)
grid_y = int((maxY-minY) / resolution)

max_iteration = 1000



def pixel_to_xy(px, py):
    return (px + 0.5) * resolution + minX, (py + 0.5) * resolution + minY

def xy_to_pixel(x, y):
    return int((x-minX)/resolution), int((y-minY)/resolution)

def calculate_loss_and_grad(type, px, py, rx, ry, st, ct, x, y, l, w):
    values = []
    values.append(abs(rx-l/2))
    values.append(abs(rx+l/2))
    values.append(abs(ry-w/2))
    values.append(abs(ry+w/2))
    if type == 0:
        loss = values[type] ** 2
        grad_x = 2.0 * (l/2 - rx) * ct
        grad_y = 2.0 * (l/2 - rx) * st
        grad_theta = -2.0 * (l/2 - rx) * ((px - x) * (-st) + (py - y) * ct)
        grad_l = l/2 - rx
        grad_w = 0
        return loss, grad_x, grad_y, grad_theta, grad_l, grad_w
    if type == 1:
        loss = values[type] ** 2
        grad_x = -2.0 * (l/2 + rx) * ct
        grad_y = -2.0 * (l/2 + rx) * st
        grad_theta = 2.0 * (l/2 + rx) * ((px - x) * (-st) + (py - y) * ct)
        grad_l = l/2 + rx
        grad_w = 0
        return loss, grad_x, grad_y, grad_theta, grad_l, grad_w
    if type == 2:
        loss = values[type] ** 2
        grad_x = -2.0 * (w/2 - ry) * st
        grad_y = 2.0 * (w/2 - ry) * ct
        grad_theta = 2.0 * (w/2 - ry) * ((px - x) * ct + (py - y) * st)
        grad_l = 0
        grad_w = w/2 - ry
        return loss, grad_x, grad_y, grad_theta, grad_l, grad_w
    if type == 3:
        loss = values[type] ** 2
        grad_x = 2.0 * (w/2 + ry) * st
        grad_y = -2.0 * (w/2 + ry) * ct
        grad_theta = -2.0 * (w/2 + ry) * ((px - x) * ct + (py - y) * st)
        grad_l = 0
        grad_w = w/2 + ry
        return loss, grad_x, grad_y, grad_theta, grad_l, grad_w


def calculate_box_loss(point, x, y, theta, l, w):
    """
    First, calculate rp = R^t(p-c)
    R = [[cos theta, -sin theta], [sin theta, cos theta]]
    case 1
    rp is in bounding box
    case 2
    rp is out of boudning box, but some perpendicular foot is on the edge of the bounding box
    case 3
    rp is out of bounding box and all of feet are not on the edge of the bounding box
    
    """
    px = point.x
    py = point.y
    ct = math.cos(theta)
    st = math.sin(theta)
    rx = (px - x) * ct + (py - y) * st
    ry = - (px - x) * st + (py - y) * ct

    
    if abs(rx) < l/2 and abs(ry) < w/2:
        values = []
        values.append(abs(rx-l/2))
        values.append(abs(rx+l/2))
        values.append(abs(ry-w/2))
        values.append(abs(ry+w/2))
        # find nearest edge
        type = values.index(min(values))
        return calculate_loss_and_grad(type, px, py, rx, ry, st, ct, x, y, l, w)
    else:
        rt_loss = 0.0
        rt_grad_x = 0.0
        rt_grad_y = 0.0
        rt_grad_theta = 0.0
        rt_grad_l = 0.0
        rt_grad_w = 0.0
        if rx > l/2:
            loss, grad_x, grad_y, grad_theta, grad_l, grad_w = calculate_loss_and_grad(0, px, py, rx, ry, st, ct, x, y, l, w)
            rt_loss += loss
            rt_grad_x += grad_x
            rt_grad_y += grad_y
            rt_grad_theta += grad_theta
            rt_grad_l += grad_l
            rt_grad_w += grad_w
        if rx < -l/2:
            loss, grad_x, grad_y, grad_theta, grad_l, grad_w = calculate_loss_and_grad(1, px, py, rx, ry, st, ct, x, y, l, w)
            rt_loss += loss
            rt_grad_x += grad_x
            rt_grad_y += grad_y
            rt_grad_theta += grad_theta
            rt_grad_l += grad_l
            rt_grad_w += grad_w
        if ry > w/2:
            loss, grad_x, grad_y, grad_theta, grad_l, grad_w = calculate_loss_and_grad(2, px, py, rx, ry, st, ct, x, y, l, w)
            rt_loss += loss
            rt_grad_x += grad_x
            rt_grad_y += grad_y
            rt_grad_theta += grad_theta
            rt_grad_l += grad_l
            rt_grad_w += grad_w
        if rx < -w/2:
            loss, grad_x, grad_y, grad_theta, grad_l, grad_w = calculate_loss_and_grad(3, px, py, rx, ry, st, ct, x, y, l, w)
            rt_loss += loss
            rt_grad_x += grad_x
            rt_grad_y += grad_y
            rt_grad_theta += grad_theta
            rt_grad_l += grad_l
            rt_grad_w += grad_w
        return rt_loss, rt_grad_x, rt_grad_y, rt_grad_theta, rt_grad_l, rt_grad_w



def build_grid_map(points):
    grid_map = [[False] * grid_y for _ in range(grid_x)]
    valid_cnt = 0
    for point in points:
        px, py = xy_to_pixel(point[0], point[1])
        if px < 0 or px >= grid_x or py < 0 or py >= grid_y:
            continue
        grid_map[px][py] = True
        valid_cnt += 1
        # print(px, py)
    print(valid_cnt)
    return grid_map


def optimized_parameters(cvh):
    # initialize parameters
    x,y = cvh.get_center()
    theta = 0.0
    l = l_g
    w = w_g
    prev_loss = None
    loss = 0.0
    final_box_loss = 0.0
    for iter in range(max_iteration):
        theta = theta - math.pi * 2 * int(theta / math.pi / 2)
        if theta < -math.pi:
            theta += math.pi
        if theta > math.pi:
            theta -= math.pi
        loss = 0.0
        grad_x = 0.0
        grad_y = 0.0
        grad_theta = 0.0
        grad_l = 0.0
        grad_w = 0.0
        lr = lr_init / (1 + decay_k * iter)
        
        # calculate loss and gradient
        # length loss
        loss += l_coeff * (l - l_g) ** 2
        grad_l += l_coeff * 2 * (l - l_g)

        # width loss
        loss += w_coeff * ((w-w_g) ** 2)
        grad_w += w_coeff * 2 * (w-w_g)

        loss += l_size_coeff * (l ** 2)
        grad_l += l_size_coeff * 2 * l

        loss += w_size_coeff * (w ** 2)
        grad_w += w_size_coeff * 2 * w

        # box loss
        total_box_loss = 0.0
        total_box_grad_x = 0.0
        total_box_grad_y = 0.0
        total_box_grad_theta = 0.0
        total_box_grad_l = 0.0 
        total_box_grad_w = 0.0
        for point in cvh.points:
            box_loss, box_grad_x, box_grad_y, box_grad_theta, box_grad_l, box_grad_w = calculate_box_loss(point, x, y, theta, l, w)
            total_box_loss += box_loss
            total_box_grad_x += box_grad_x
            total_box_grad_y += box_grad_y
            total_box_grad_theta += box_grad_theta
            total_box_grad_l += box_grad_l
            total_box_grad_w += box_grad_w
        
        total_box_loss /= cvh.n_points
        total_box_grad_x /= cvh.n_points
        total_box_grad_y /= cvh.n_points
        total_box_grad_theta /= cvh.n_points
        total_box_grad_l /= cvh.n_points
        total_box_grad_w /= cvh.n_points
        final_box_loss = total_box_loss

        loss += box_coeff * total_box_loss
        grad_x += box_coeff * total_box_grad_x
        grad_y += box_coeff * total_box_grad_y
        grad_theta += box_coeff * total_box_grad_theta
        grad_l += box_coeff * total_box_grad_l
        grad_w += box_coeff * total_box_grad_w

        

        # update parameters
        x -= lr * grad_x
        y -= lr * grad_y
        theta -= lr * grad_theta
        l -= lr * grad_l
        w -= lr * grad_w
        
        if prev_loss is not None:
            if abs(loss - prev_loss) < loss_threshold:
                print("optimize!")
                print(final_box_loss)
                break
        prev_loss = loss
    
    return {'x' : x, 'y' : y, 'theta' : theta, 'l' : l, 'w' : w, 'loss' : loss}, loss



# load dataset
pointclouds = []
labels = []
for seq in range(N):
    pointcloud = np.fromfile(bin_path + str(seq).zfill(7) + ".bin", dtype=np.float32).reshape(-1, 4)
    with open(label_path+str(seq).zfill(7)+".json", "r") as label_json:
        label = json.load(label_json)
    pointclouds.append(pointcloud)
    labels.append(label)

    

# for each data, calculate optimized parameters
loss_sum = 0.0
x_error = 0.0
y_error = 0.0
theta_error = 0.0
l_error = 0.0
w_error = 0.0
cnt = 0
iou_list = []
theta_diff = []
x_diff = []
for seq in range(N):
    # build grid map
    print("# of pointclouds :", pointclouds[seq].size)
    grid_map = build_grid_map(pointclouds[seq])

    # extract points and build convex hull
    pts = []
    for i in range(grid_x):
        for j in range(grid_y):
            if not grid_map[i][j]:
                continue
            x,y = pixel_to_xy(i,j)
            pts.append(Point(x+resolution/2, y+resolution/2))
            pts.append(Point(x+resolution/2, y+resolution/2))
            pts.append(Point(x+resolution/2, y+resolution/2))
            pts.append(Point(x+resolution/2, y+resolution/2))
    
    solution, loss = optimized_parameters(ConvexHull(pts))
    label = labels[seq]
    loss_sum += loss
    # label_cvh = box_convex_hull(label['x'], label['y'], label['theta'], label['l'], label['w'])
    # solution_cvh = box_convex_hull(solution['x'], solution['y'], solution['theta'], solution['l'], solution['w'])
    # iou = calculate_IOU(label_cvh, solution_cvh)
    # iou_list.append(iou)
    # if iou > 0.85:
    #     cnt += 1
    print("=======================================")
    print(seq)
    print(label)
    print(solution)
    # print(iou)
    print("=======================================")
    x_error += (label['x'] - solution['x']) ** 2
    y_error += (label['y'] - solution['y']) ** 2
    dtheta = abs(label['theta'] - solution['theta'])
    dtheta = min(dtheta, abs(dtheta-math.pi))
    theta_error += (dtheta) ** 2
    theta_diff.append(dtheta)
    x_diff.append(label['x'] - solution['x'])
    l_error += (label['l'] - solution['l']) ** 2
    w_error += (label['w'] - solution['w']) ** 2
    
loss_sum /= N
x_error /= N
y_error /= N
theta_error /= N
l_error /= N
w_error /= N

print("Mean loss : ", loss_sum)
print("MSE x : ", math.sqrt(x_error))
print("MSE y : ", math.sqrt(y_error))
print("MSE theta : ", math.sqrt(theta_error))
print("MSE l : ", math.sqrt(l_error))
print("MSE w : ", math.sqrt(w_error))

plt.hist(theta_diff)
plt.show()

plt.hist(x_diff)
plt.show()
# print("Mean IOU : ", sum(iou_list)/len(iou_list))
# print("AP 85% : ", 1.0*cnt / len(iou_list))

