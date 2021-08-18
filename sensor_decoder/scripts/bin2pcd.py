import numpy as np
import struct
import sys
import open3d as o3d
import os
import time


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


data_name_list = ['0729_exp_gunmin_FMTC'
                ,'0729_exp_gunmin_highway'
                ,'0729_exp_gunmin_road'
                ,'0729_exp_jeongwoo_FMTC'
                ,'0729_exp_sumin_FMTC'
                ,'0729_exp_sumin_highway'
                ,'0729_exp_sumin_road'
                ,'0729_exp_wooseok_FMTC'
                ,'0729_neg_gunmin_01_1'
                ,'0729_neg_gunmin_02_1'
                ,'0729_neg_gunmin_03_1'
                ,'0729_neg_gunmin_05_1'
                ,'0729_neg_gunmin_06_1'
                ,'0729_neg_gunmin_08_1'
                ,'0729_neg_gunmin_09_1'
                ,'0729_neg_gunmin_10_1'
                ,'0729_neg_gunmin_16_1'
                ,'0729_neg_gunmin_16_2'
                ,'0729_neg_gunmin_28_1'
                ,'0729_neg_gunmin_28_2'
                ,'0729_neg_gunmin_29_1'
                ,'0729_neg_gunmin_30_1'
                ,'0729_neg_gunmin_30_2'
                ,'0729_neg_gunmin_31_1'
                ,'0729_neg_gunmin_31_2'
                ,'0729_neg_gunmin_34_1'
                ,'0729_neg_gunmin_34_2'
                ,'0729_neg_gunmin_35_1'
                ,'0729_neg_gunmin_35_2'
                ,'0729_neg_gunmin_36_1'
                ,'0729_neg_gunmin_36_2'
                ,'0729_neg_gunmin_37_1'
                ,'0729_neg_gunmin_37_2'
                ,'0729_neg_gunmin_38_1'
                ,'0729_neg_gunmin_38_2'
                ,'0729_neg_gunmin_38_3'
                ,'0729_neg_gunmin_38_4'
                ,'0729_neg_gunmin_38_5'
                ,'0729_neg_gunmin_38_6'
                ,'0729_neg_gunmin_50_1'
                ,'0729_neg_gunmin_50_2'
                ,'0729_neg_jeongwoo_01_1'
                ,'0729_neg_jeongwoo_02_1'
                ,'0729_neg_jeongwoo_03_1'
                ,'0729_neg_jeongwoo_05_1'
                ,'0729_neg_jeongwoo_06_1'
                ,'0729_neg_jeongwoo_08_1'
                ,'0729_neg_jeongwoo_09_1'
                ,'0729_neg_jeongwoo_10_1'
                ,'0729_neg_jeongwoo_50_1'
                ,'0729_neg_jeongwoo_50_2'
                ,'0729_neg_sumin_01_1'
                ,'0729_neg_sumin_02_1'
                ,'0729_neg_sumin_03_1'
                ,'0729_neg_sumin_05_1'
                ,'0729_neg_sumin_06_1'
                ,'0729_neg_sumin_08_1'
                ,'0729_neg_sumin_09_1'
                ,'0729_neg_sumin_10_1'
                ,'0729_neg_sumin_38_1'
                ,'0729_neg_sumin_38_2'
                ,'0729_neg_sumin_38_3'
                ,'0729_neg_sumin_38_4'
                ,'0729_neg_sumin_42_1'
                ,'0729_neg_sumin_42_2'
                ,'0729_neg_sumin_42_3'
                ,'0729_neg_sumin_42_4'
                ,'0729_neg_sumin_50_1'
                ,'0729_neg_sumin_50_2'
                ,'0729_neg_wooseok_01_1'
                ,'0729_neg_wooseok_02_1'
                ,'0729_neg_wooseok_03_1'
                ,'0729_neg_wooseok_05_1'
                ,'0729_neg_wooseok_06_1'
                ,'0729_neg_wooseok_08_1'
                ,'0729_neg_wooseok_09_1'
                ,'0729_neg_wooseok_10_1'
                ,'0729_neg_wooseok_28'
                ,'0729_neg_wooseok_28_1'
                ,'0729_neg_wooseok_29_1'
                ,'0729_neg_wooseok_29_2'
                ,'0729_neg_wooseok_30_1'
                ,'0729_neg_wooseok_30_2'
                ,'0729_neg_wooseok_31_1'
                ,'0729_neg_wooseok_31_2'
                ,'0729_neg_wooseok_34_2'
                ,'0729_neg_wooseok_35_1'
                ,'0729_neg_wooseok_35_2'
                ,'0729_neg_wooseok_36_1'
                ,'0729_neg_wooseok_36_2'
                ,'0729_neg_wooseok_37_1'
                ,'0729_neg_wooseok_37_2'
                ,'0729_neg_wooseok_46'
                ,'0729_neg_wooseok_47'
                ,'0729_neg_wooseok_50_1'
                ,'0729_neg_wooseok_50_2'
                ,'0813_exp_jeongwoo_road_1'
                ,'0813_exp_jeongwoo_road_2'
                ,'0815_exp_jeongwoo_highway_1'
                ,'0815_exp_jeongwoo_highway_2']


seq_list = [(50,2450),
            (6000,9000),
            (5000,8000),
            (0,2200),
            (150,2550),
            (500,3500),
            (6000,9000),
            (50,1450),
            (40,140),
            (0,140),
            (0,160),
            (90,150),
            (90,170),
            (50,150),
            (90,190),
            (30,80),
            (80,180),
            (220,260),
            (200,220),
            (120,140),
            (120,140),
            (130,160),
            (180,200),
            (160,180),
            (145,165),
            (120,140),
            (90,110),
            (140,160),
            (180,200),
            (120,140),
            (180,200),
            (80,100),
            (120,130),
            (120,140),
            (150,170),
            (140,160),
            (150,170),
            (240,270),
            (180,200),
            (100,200),
            (80,200),
            (0,150),
            (0,150),
            (0,100),
            (80,190),
            (110,190),
            (50,150),
            (170,220),
            (70,100),
            (90,160),
            (90,290),
            (80,150),
            (70,160),
            (40,140),
            (40,80),
            (230,270),
            (50,150),
            (50,70),
            (130,150),
            (180,200),
            (120,140),
            (155,175),
            (120,140),
            (190,210),
            (140,160),
            (110,130),
            (130,150),
            (80,250),
            (80,180),
            (80,160),
            (50,150),
            (50,130),
            (120,190),
            (100,150),
            (40,110),
            (300,350),
            (360,400),
            (160,190),
            (125,155),
            (120,140),
            (100,110),
            (80,100),
            (90,110),
            (70,90),
            (110,130),
            (130,150),
            (90,110),
            (90,110),
            (230,250),
            (80,100),
            (100,130),
            (110,140),
            (170,190),
            (160,190),
            (30,130),
            (170,220),
            (1400,4400),
            (14000,17000),
            (1400,4400),
            (5500,8500)]

N = len(data_name_list)
start = time.time()
end = time.time()
for i, data_name in enumerate(data_name_list) :
    if data_name[1] == '7':
        continue
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