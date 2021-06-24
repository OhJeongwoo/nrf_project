import numpy as np
import cv2
 
start = 12144
end = 13950
for index in range(start, end):
    pointcloud = np.fromfile('data/pc_example/bin/'+str(index).zfill(6)+'.bin', dtype=np.float32, count=-1).reshape([-1,4])
    
    print(pointcloud.shape)
    x = pointcloud[:, 0]  # x position of point
    y = pointcloud[:, 1]  # y position of point
    z = pointcloud[:, 2]  # z position of point
    r = pointcloud[:, 3]  # reflectance value of point
    d = np.sqrt(x ** 2 + y ** 2)  # Map Distance from sensor
    N = len(x)
    

    resolution = 0.1
    cpx = 300
    cpy = 300
    img = np.zeros((600, 600), np.float32)
    h, w= img.shape

    for i in range(N):
        px = int(x[i] / resolution + cpx)
        py = int(y[i] / resolution + cpy) 
        if px <0 or px>= h or py<0 or py>=w:
            continue
        img[px][py] = 1

    vis2 = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    cv2.imshow("WindowNameHere", vis2)
    cv2.waitKey(3)