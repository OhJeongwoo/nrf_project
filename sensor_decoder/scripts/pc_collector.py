import pypcd
import rospy
import os
import numpy as np
from sensor_msgs.msg import PointCloud2
import cv2

project_path = os.path.dirname(os.path.abspath(os.path.dirname(__file__)))
data_name = 'pc_example'
pcd_path = project_path + '/data/' + data_name + '/pointclouds/'
bin_path = project_path + '/data/' + data_name + '/bin/'


seq = 0
visualize_mode = False
save_mode = True

def callback(msg):
    seq = msg.header.seq
    # print(seq)
    # print(msg)
    pc = pypcd.PointCloud.from_msg(msg)
    x = pc.pc_data['x']
    y = pc.pc_data['y']
    z = pc.pc_data['z']
    intensity = pc.pc_data['intensity']
    # print(intensity)
    # print(x.shape)
    # print(np.max(intensity))
    arr = np.zeros(x.shape[0] + y.shape[0] + z.shape[0] + intensity.shape[0], dtype=np.float32)
    arr[::4] = x
    arr[1::4] = y
    arr[2::4] = z
    arr[3::4] = intensity
    if visualize_mode:
        N = x.shape[0]
        resolution = 0.1
        cx = 0.0
        cy = 0.0
        cpx = 300
        cpy = 300
        img = np.zeros((600, 600), np.float32)
        h, w= img.shape

        for i in range(N):
            px = int((x[i]-cx) / resolution + cpx)
            py = int((y[i]-cy) / resolution + cpy) 
            if px <0 or px>= h or py<0 or py>=w:
                continue
            img[px][py] = 1

        vis2 = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        cv2.imshow("Lidar_map", vis2)
        cv2.waitKey(1)
    if save_mode : 
        arr.astype('float32').tofile(bin_path + str(seq).zfill(6) + '.bin')
        pc.save_pcd(pcd_path + str(seq).zfill(6) + '.pcd', compression='binary_compressed')
        print("save file: ", bin_path + str(seq).zfill(6) + '.bin')
    

rospy.init_node('pypcd_node', anonymous=True)
sub = rospy.Subscriber('/points_raw', PointCloud2, callback)
rospy.spin()
