import numpy as np
import struct
import sys
import open3d as o3d
import os
import time
from seq import data_name_list
from seq import seq_list


def bin_to_pcd(binFileName, pcdFileName):
    size_float = 4
    list_pcd = []
    with open(binFileName, "rb") as f:
        byte = f.read(size_float * 4)
        while byte:
            x, y, z, intensity = struct.unpack("ffff", byte)
            list_pcd.append([x, y, z])
            byte = f.read(size_float * 4)
    np_pcd = np.asarray(list_pcd)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_pcd)
    o3d.io.write_point_cloud(pcdFileName, pcd)
    return pcd



N = len(data_name_list)
start = time.time()
end = time.time()
data_index_list = [i for i in range(101,382)]
for i in data_index_list:
    data_name = data_name_list[i]
    print("[%d / %d, %.2f] Start to convert point cloud files, data name is %s" %(i+1, N, end - start, data_name))
    data_path = "/home/jeongwoooh/catkin_ws/src/nrf_project/sensor_decoder/data/" + data_name + "/"
    # print('(0,'+str(len(os.listdir(data_path + "bin/"))) + ')')
    # bin_path = data_path + "bin/"
    # pcd_path = data_path + "pcd/"
    # print(str(len(os.listdir(data_path+"bin/"))-10)+',')
    bin_path = data_path + "bin/"
    pcd_path = data_path + "pcd/"
    st = seq_list[i][0]
    en = seq_list[i][1]
    M = en - st

    for j in range(1, M+1):
        seq = j + st
        bin_file = bin_path + str(seq).zfill(6) + ".bin"
        pcd_file = pcd_path + str(seq).zfill(6) + ".pcd"
        bin_to_pcd(bin_file, pcd_file)
        end = time.time()
        if j % 100 == 0:
            print("[%d / %d, %.2f] In progress to convert point cloud files" %(j, M, end-start))
# data_path = "/home/navi2/catkin_ws/src/nrf_project/sensor_decoder/data/" + data_name + "/"
# bin_path = data_path + "bin/"
# pcd_path = data_path + "pcd/"



# for seq in range(s_index, e_index):
#     print(seq)
#     bin_file = bin_path + str(seq).zfill(6) + ".bin"
#     pcd_file = pcd_path + str(seq).zfill(6) + ".pcd"
#     bin_to_pcd(bin_file, pcd_file)