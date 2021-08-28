#include <iostream>
#include <fstream>
#include <iomanip>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <ctime>
#include <pthread.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include "sensor_decoder/Mobileye.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "inertiallabs_msgs/ins_data.h"

#include "ros/ros.h"
#include "ros/time.h"
#include "ros/package.h"

#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/ModelCoefficients.h>
#include <pcl-1.8/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.8/pcl/filters/extract_indices.h>
#include <pcl-1.8/pcl/io/pcd_io.h>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include <utils.h>

using namespace std;

typedef pair<int, int> pii;
typedef pair<double, double> pdd;
typedef pair<pii, int> piii;
typedef pair<double, int> pdi;


double minX = -20.0;
double maxX = 40.0;
double minY = -30.0;
double maxY = 30.0;
double minZ = 0.5;
double maxZ = 2.0;
double resolution = 1.0;

const int GRID = 60;

double cluster_threshold = 1.0;
double convex_hull_threshold = 0.3;
int R = int(cluster_threshold/resolution) + 1;

pii xy_to_pixel(point p){
    pii rt;
    rt.first = int((p.x - minX)/resolution);
    rt.second = int((p.y - minY)/resolution);
    return rt;
}

string zfill(int n){
    if(n==0) return "000000";
    int digit = log10(n) + 1;
    string rt = "";
    for(int i=0;i<6-digit;i++) rt += "0";
    return rt+to_string(n);
}


vector<string> data_name_list = {"0729_exp_gunmin_FMTC"
                                ,"0729_exp_gunmin_highway"
                                ,"0729_exp_gunmin_road"
                                ,"0729_exp_jeongwoo_FMTC"
                                ,"0729_exp_sumin_FMTC"
                                ,"0729_exp_sumin_highway"
                                ,"0729_exp_sumin_road"
                                ,"0729_exp_wooseok_FMTC"
                                ,"0729_neg_gunmin_01_1"
                                ,"0729_neg_gunmin_02_1"
                                ,"0729_neg_gunmin_03_1"
                                ,"0729_neg_gunmin_05_1"
                                ,"0729_neg_gunmin_06_1"
                                ,"0729_neg_gunmin_08_1"
                                ,"0729_neg_gunmin_09_1"
                                ,"0729_neg_gunmin_10_1"
                                ,"0729_neg_gunmin_16_1"
                                ,"0729_neg_gunmin_16_2"
                                ,"0729_neg_gunmin_28_1"
                                ,"0729_neg_gunmin_28_2"
                                ,"0729_neg_gunmin_29_1"
                                ,"0729_neg_gunmin_30_1"
                                ,"0729_neg_gunmin_30_2"
                                ,"0729_neg_gunmin_31_1"
                                ,"0729_neg_gunmin_31_2"
                                ,"0729_neg_gunmin_34_1"
                                ,"0729_neg_gunmin_34_2"
                                ,"0729_neg_gunmin_35_1"
                                ,"0729_neg_gunmin_35_2"
                                ,"0729_neg_gunmin_36_1"
                                ,"0729_neg_gunmin_36_2"
                                ,"0729_neg_gunmin_37_1"
                                ,"0729_neg_gunmin_37_2"
                                ,"0729_neg_gunmin_38_1"
                                ,"0729_neg_gunmin_38_2"
                                ,"0729_neg_gunmin_38_3"
                                ,"0729_neg_gunmin_38_4"
                                ,"0729_neg_gunmin_38_5"
                                ,"0729_neg_gunmin_38_6"
                                ,"0729_neg_gunmin_50_1"
                                ,"0729_neg_gunmin_50_2"
                                ,"0729_neg_jeongwoo_01_1"
                                ,"0729_neg_jeongwoo_02_1"
                                ,"0729_neg_jeongwoo_03_1"
                                ,"0729_neg_jeongwoo_05_1"
                                ,"0729_neg_jeongwoo_06_1"
                                ,"0729_neg_jeongwoo_08_1"
                                ,"0729_neg_jeongwoo_09_1"
                                ,"0729_neg_jeongwoo_10_1"
                                ,"0729_neg_jeongwoo_50_1"
                                ,"0729_neg_jeongwoo_50_2"
                                ,"0729_neg_sumin_01_1"
                                ,"0729_neg_sumin_02_1"
                                ,"0729_neg_sumin_03_1"
                                ,"0729_neg_sumin_05_1"
                                ,"0729_neg_sumin_06_1"
                                ,"0729_neg_sumin_08_1"
                                ,"0729_neg_sumin_09_1"
                                ,"0729_neg_sumin_10_1"
                                ,"0729_neg_sumin_38_1"
                                ,"0729_neg_sumin_38_2"
                                ,"0729_neg_sumin_38_3"
                                ,"0729_neg_sumin_38_4"
                                ,"0729_neg_sumin_42_1"
                                ,"0729_neg_sumin_42_2"
                                ,"0729_neg_sumin_42_3"
                                ,"0729_neg_sumin_42_4"
                                ,"0729_neg_sumin_50_1"
                                ,"0729_neg_sumin_50_2"
                                ,"0729_neg_wooseok_01_1"
                                ,"0729_neg_wooseok_02_1"
                                ,"0729_neg_wooseok_03_1"
                                ,"0729_neg_wooseok_05_1"
                                ,"0729_neg_wooseok_06_1"
                                ,"0729_neg_wooseok_08_1"
                                ,"0729_neg_wooseok_09_1"
                                ,"0729_neg_wooseok_10_1"
                                ,"0729_neg_wooseok_28"
                                ,"0729_neg_wooseok_28_1"
                                ,"0729_neg_wooseok_29_1"
                                ,"0729_neg_wooseok_29_2"
                                ,"0729_neg_wooseok_30_1"
                                ,"0729_neg_wooseok_30_2"
                                ,"0729_neg_wooseok_31_1"
                                ,"0729_neg_wooseok_31_2"
                                ,"0729_neg_wooseok_34_2"
                                ,"0729_neg_wooseok_35_1"
                                ,"0729_neg_wooseok_35_2"
                                ,"0729_neg_wooseok_36_1"
                                ,"0729_neg_wooseok_36_2"
                                ,"0729_neg_wooseok_37_1"
                                ,"0729_neg_wooseok_37_2"
                                ,"0729_neg_wooseok_46"
                                ,"0729_neg_wooseok_47"
                                ,"0729_neg_wooseok_50_1"
                                ,"0729_neg_wooseok_50_2"
                                ,"0813_exp_jeongwoo_road_1"
                                ,"0813_exp_jeongwoo_road_2"
                                ,"0815_exp_jeongwoo_highway_1"
                                ,"0815_exp_jeongwoo_highway_2"
                                ,"0826_exp_jeongwoo_FMTC"
                                ,"0823_neg_jeongwoo_01_01"
                                ,"0823_neg_jeongwoo_01_02"
                                ,"0823_neg_jeongwoo_01_03"
                                ,"0823_neg_jeongwoo_01_04"
                                ,"0823_neg_jeongwoo_01_05"
                                ,"0823_neg_jeongwoo_02_01"
                                ,"0823_neg_jeongwoo_02_02"
                                ,"0823_neg_jeongwoo_02_03"
                                ,"0823_neg_jeongwoo_02_04"
                                ,"0823_neg_jeongwoo_02_05"
                                ,"0823_neg_jeongwoo_02_06"
                                ,"0823_neg_jeongwoo_02_07"
                                ,"0823_neg_jeongwoo_02_08"
                                ,"0823_neg_jeongwoo_02_09"
                                ,"0823_neg_jeongwoo_02_10"
                                ,"0826_neg_jeongwoo_03_01"
                                ,"0826_neg_jeongwoo_03_02"
                                ,"0826_neg_jeongwoo_03_04"
                                ,"0826_neg_jeongwoo_03_05"
                                ,"0826_neg_jeongwoo_04_01"
                                ,"0826_neg_jeongwoo_04_02"
                                ,"0826_neg_jeongwoo_04_03"
                                ,"0826_neg_jeongwoo_04_04"
                                ,"0826_neg_jeongwoo_04_05"
                                ,"0826_neg_jeongwoo_04_06"
                                ,"0826_neg_jeongwoo_04_07"
                                ,"0826_neg_jeongwoo_04_08"
                                ,"0826_neg_jeongwoo_04_09"
                                ,"0826_neg_jeongwoo_04_10"
                                ,"0826_neg_jeongwoo_05_01"
                                ,"0826_neg_jeongwoo_05_02"
                                ,"0826_neg_jeongwoo_05_03"
                                ,"0826_neg_jeongwoo_05_04"
                                ,"0826_neg_jeongwoo_05_05"
                                ,"0826_neg_jeongwoo_05_06"
                                ,"0826_neg_jeongwoo_05_07"
                                ,"0826_neg_jeongwoo_05_08"
                                ,"0826_neg_jeongwoo_05_09"
                                ,"0826_neg_jeongwoo_05_10"
                                ,"0826_neg_jeongwoo_06_02"
                                ,"0826_neg_jeongwoo_06_03"
                                ,"0826_neg_jeongwoo_06_04"
                                ,"0826_neg_jeongwoo_06_05"
                                ,"0823_neg_jeongwoo_09_01"
                                ,"0823_neg_jeongwoo_09_02"
                                ,"0823_neg_jeongwoo_09_03"
                                ,"0823_neg_jeongwoo_09_04"
                                ,"0823_neg_jeongwoo_09_05"
                                ,"0823_neg_jeongwoo_09_06"
                                ,"0823_neg_jeongwoo_09_07"
                                ,"0823_neg_jeongwoo_09_08"
                                ,"0823_neg_jeongwoo_09_09"
                                ,"0823_neg_jeongwoo_09_10"
                                ,"0826_neg_jeongwoo_12_01"
                                ,"0826_neg_jeongwoo_12_02"
                                ,"0826_neg_jeongwoo_12_03"
                                ,"0826_neg_jeongwoo_12_04"
                                ,"0826_neg_jeongwoo_12_05"
                                ,"0826_neg_jeongwoo_13_01"
                                ,"0826_neg_jeongwoo_13_02"
                                ,"0826_neg_jeongwoo_13_03"
                                ,"0826_neg_jeongwoo_13_04"
                                ,"0826_neg_jeongwoo_13_05"
                                ,"0826_neg_jeongwoo_14_01"
                                ,"0826_neg_jeongwoo_14_02"
                                ,"0826_neg_jeongwoo_14_03"
                                ,"0826_neg_jeongwoo_14_04"
                                ,"0826_neg_jeongwoo_14_05"
                                ,"0826_neg_jeongwoo_15_01"
                                ,"0826_neg_jeongwoo_15_02"
                                ,"0826_neg_jeongwoo_15_03"
                                ,"0826_neg_jeongwoo_15_04"
                                ,"0826_neg_jeongwoo_15_05"
                                ,"0826_neg_jeongwoo_16_01"
                                ,"0826_neg_jeongwoo_16_02"
                                ,"0826_neg_jeongwoo_16_03"
                                ,"0826_neg_jeongwoo_16_04"
                                ,"0826_neg_jeongwoo_16_05"
                                ,"0826_neg_jeongwoo_16_06"
                                ,"0826_neg_jeongwoo_16_07"
                                ,"0826_neg_jeongwoo_16_08"
                                ,"0826_neg_jeongwoo_16_09"
                                ,"0826_neg_jeongwoo_16_10"
                                ,"0826_neg_jeongwoo_17_01"
                                ,"0826_neg_jeongwoo_17_02"
                                ,"0826_neg_jeongwoo_17_03"
                                ,"0826_neg_jeongwoo_17_04"
                                ,"0826_neg_jeongwoo_17_05"
                                ,"0826_neg_jeongwoo_18_01"
                                ,"0826_neg_jeongwoo_18_02"
                                ,"0826_neg_jeongwoo_18_03"
                                ,"0826_neg_jeongwoo_18_04"
                                ,"0826_neg_jeongwoo_18_05"
                                ,"0826_neg_jeongwoo_18_06"
                                ,"0826_neg_jeongwoo_19_01"
                                ,"0826_neg_jeongwoo_19_02"
                                ,"0826_neg_jeongwoo_19_03"
                                ,"0826_neg_jeongwoo_19_04"
                                ,"0826_neg_jeongwoo_19_05"
                                ,"0826_neg_jeongwoo_20_01"
                                ,"0826_neg_jeongwoo_20_02"
                                ,"0826_neg_jeongwoo_20_03"
                                ,"0826_neg_jeongwoo_20_04"
                                ,"0826_neg_jeongwoo_20_05"
                                ,"0826_neg_jeongwoo_21_01"
                                ,"0826_neg_jeongwoo_21_02"
                                ,"0826_neg_jeongwoo_21_03"
                                ,"0826_neg_jeongwoo_21_04"
                                ,"0826_neg_jeongwoo_21_05"
                                ,"0826_neg_jeongwoo_23_01"
                                ,"0826_neg_jeongwoo_23_02"
                                ,"0826_neg_jeongwoo_23_03"
                                ,"0826_neg_jeongwoo_23_04"
                                ,"0826_neg_jeongwoo_23_05"
                                ,"0826_neg_jeongwoo_24_01"
                                ,"0826_neg_jeongwoo_24_02"
                                ,"0826_neg_jeongwoo_24_03"
                                ,"0826_neg_jeongwoo_24_04"
                                ,"0826_neg_jeongwoo_24_05"
                                ,"0826_neg_jeongwoo_24_06"
                                ,"0826_neg_jeongwoo_24_07"
                                ,"0826_neg_jeongwoo_25_01"
                                ,"0826_neg_jeongwoo_25_02"
                                ,"0826_neg_jeongwoo_25_03"
                                ,"0826_neg_jeongwoo_25_04"
                                ,"0826_neg_jeongwoo_25_05"
                                ,"0826_neg_jeongwoo_26_01"
                                ,"0826_neg_jeongwoo_26_02"
                                ,"0826_neg_jeongwoo_26_03"
                                ,"0826_neg_jeongwoo_26_04"
                                ,"0826_neg_jeongwoo_26_05"
                                ,"0823_neg_jeongwoo_27_01"
                                ,"0823_neg_jeongwoo_27_02"
                                ,"0823_neg_jeongwoo_27_03"
                                ,"0823_neg_jeongwoo_27_04"
                                ,"0823_neg_jeongwoo_27_05"
                                ,"0823_neg_jeongwoo_27_06"
                                ,"0823_neg_jeongwoo_27_07"
                                ,"0823_neg_jeongwoo_27_08"
                                ,"0823_neg_jeongwoo_27_09"
                                ,"0823_neg_jeongwoo_27_10"
                                ,"0823_neg_jeongwoo_28_01"
                                ,"0823_neg_jeongwoo_28_02"
                                ,"0823_neg_jeongwoo_28_03"
                                ,"0823_neg_jeongwoo_28_04"
                                ,"0823_neg_jeongwoo_28_05"
                                ,"0823_neg_jeongwoo_28_06"
                                ,"0823_neg_jeongwoo_28_07"
                                ,"0823_neg_jeongwoo_28_08"
                                ,"0823_neg_jeongwoo_28_09"
                                ,"0823_neg_jeongwoo_28_10"
                                ,"0823_neg_jeongwoo_29_01"
                                ,"0823_neg_jeongwoo_29_02"
                                ,"0823_neg_jeongwoo_29_03"
                                ,"0823_neg_jeongwoo_29_04"
                                ,"0823_neg_jeongwoo_29_05"
                                ,"0823_neg_jeongwoo_29_06"
                                ,"0823_neg_jeongwoo_29_07"
                                ,"0823_neg_jeongwoo_29_08"
                                ,"0823_neg_jeongwoo_29_09"
                                ,"0823_neg_jeongwoo_29_10"
                                ,"0823_neg_jeongwoo_30_01"
                                ,"0823_neg_jeongwoo_30_02"
                                ,"0823_neg_jeongwoo_30_03"
                                ,"0823_neg_jeongwoo_30_04"
                                ,"0823_neg_jeongwoo_30_05"
                                ,"0823_neg_jeongwoo_30_06"
                                ,"0823_neg_jeongwoo_30_07"
                                ,"0823_neg_jeongwoo_30_08"
                                ,"0823_neg_jeongwoo_30_09"
                                ,"0823_neg_jeongwoo_30_10"
                                ,"0823_neg_jeongwoo_32_01"
                                ,"0823_neg_jeongwoo_32_02"
                                ,"0823_neg_jeongwoo_32_03"
                                ,"0823_neg_jeongwoo_32_04"
                                ,"0823_neg_jeongwoo_32_05"
                                ,"0823_neg_jeongwoo_32_06"
                                ,"0823_neg_jeongwoo_32_07"
                                ,"0823_neg_jeongwoo_32_08"
                                ,"0823_neg_jeongwoo_32_09"
                                ,"0823_neg_jeongwoo_32_10"
                                ,"0823_neg_jeongwoo_33_01"
                                ,"0823_neg_jeongwoo_33_02"
                                ,"0823_neg_jeongwoo_33_03"
                                ,"0823_neg_jeongwoo_33_04"
                                ,"0823_neg_jeongwoo_33_05"
                                ,"0823_neg_jeongwoo_33_06"
                                ,"0823_neg_jeongwoo_33_07"
                                ,"0823_neg_jeongwoo_33_08"
                                ,"0823_neg_jeongwoo_33_09"
                                ,"0823_neg_jeongwoo_33_10"
                                ,"0823_neg_jeongwoo_34_01"
                                ,"0823_neg_jeongwoo_34_02"
                                ,"0823_neg_jeongwoo_34_03"
                                ,"0823_neg_jeongwoo_34_04"
                                ,"0823_neg_jeongwoo_34_05"
                                ,"0823_neg_jeongwoo_34_06"
                                ,"0823_neg_jeongwoo_34_07"
                                ,"0823_neg_jeongwoo_34_08"
                                ,"0823_neg_jeongwoo_34_09"
                                ,"0823_neg_jeongwoo_34_10"
                                ,"0823_neg_jeongwoo_35_01"
                                ,"0823_neg_jeongwoo_35_02"
                                ,"0823_neg_jeongwoo_35_03"
                                ,"0823_neg_jeongwoo_35_04"
                                ,"0823_neg_jeongwoo_35_05"
                                ,"0823_neg_jeongwoo_35_06"
                                ,"0823_neg_jeongwoo_35_07"
                                ,"0823_neg_jeongwoo_35_08"
                                ,"0823_neg_jeongwoo_35_09"
                                ,"0823_neg_jeongwoo_35_10"
                                ,"0823_neg_jeongwoo_36_01"
                                ,"0823_neg_jeongwoo_36_02"
                                ,"0823_neg_jeongwoo_36_03"
                                ,"0823_neg_jeongwoo_36_04"
                                ,"0823_neg_jeongwoo_36_05"
                                ,"0823_neg_jeongwoo_36_06"
                                ,"0823_neg_jeongwoo_36_07"
                                ,"0823_neg_jeongwoo_36_08"
                                ,"0823_neg_jeongwoo_36_09"
                                ,"0823_neg_jeongwoo_36_10"
                                ,"0823_neg_jeongwoo_37_01"
                                ,"0823_neg_jeongwoo_37_02"
                                ,"0823_neg_jeongwoo_37_03"
                                ,"0823_neg_jeongwoo_37_04"
                                ,"0823_neg_jeongwoo_37_05"
                                ,"0823_neg_jeongwoo_37_06"
                                ,"0823_neg_jeongwoo_37_07"
                                ,"0823_neg_jeongwoo_37_08"
                                ,"0823_neg_jeongwoo_37_09"
                                ,"0823_neg_jeongwoo_37_10"
                                ,"0823_neg_jeongwoo_38_01"
                                ,"0823_neg_jeongwoo_38_02"
                                ,"0823_neg_jeongwoo_38_03"
                                ,"0823_neg_jeongwoo_38_04"
                                ,"0823_neg_jeongwoo_38_05"
                                ,"0823_neg_jeongwoo_38_06"
                                ,"0823_neg_jeongwoo_38_07"
                                ,"0823_neg_jeongwoo_38_08"
                                ,"0823_neg_jeongwoo_38_09"
                                ,"0823_neg_jeongwoo_38_10"
                                ,"0823_neg_jeongwoo_41_01"
                                ,"0823_neg_jeongwoo_41_02"
                                ,"0823_neg_jeongwoo_41_03"
                                ,"0823_neg_jeongwoo_41_04"
                                ,"0823_neg_jeongwoo_41_05"
                                ,"0823_neg_jeongwoo_41_06"
                                ,"0823_neg_jeongwoo_41_07"
                                ,"0823_neg_jeongwoo_41_08"
                                ,"0823_neg_jeongwoo_41_09"
                                ,"0823_neg_jeongwoo_41_10"
                                ,"0823_neg_jeongwoo_42_01"
                                ,"0823_neg_jeongwoo_42_02"
                                ,"0823_neg_jeongwoo_42_03"
                                ,"0823_neg_jeongwoo_42_04"
                                ,"0823_neg_jeongwoo_42_05"
                                ,"0823_neg_jeongwoo_42_06"
                                ,"0823_neg_jeongwoo_42_07"
                                ,"0823_neg_jeongwoo_42_08"
                                ,"0823_neg_jeongwoo_42_09"
                                ,"0823_neg_jeongwoo_42_10"
                                ,"0823_neg_jeongwoo_43_01"
                                ,"0823_neg_jeongwoo_43_02"
                                ,"0823_neg_jeongwoo_43_03"
                                ,"0823_neg_jeongwoo_43_04"
                                ,"0823_neg_jeongwoo_43_05"
                                ,"0823_neg_jeongwoo_43_06"
                                ,"0823_neg_jeongwoo_43_07"
                                ,"0823_neg_jeongwoo_43_08"
                                ,"0823_neg_jeongwoo_43_09"
                                ,"0823_neg_jeongwoo_43_10"
                                ,"0823_neg_jeongwoo_44_01"
                                ,"0823_neg_jeongwoo_44_02"
                                ,"0823_neg_jeongwoo_44_03"
                                ,"0823_neg_jeongwoo_44_04"
                                ,"0823_neg_jeongwoo_44_05"
                                ,"0823_neg_jeongwoo_44_06"
                                ,"0823_neg_jeongwoo_44_07"
                                ,"0823_neg_jeongwoo_44_08"
                                ,"0823_neg_jeongwoo_44_09"
                                ,"0823_neg_jeongwoo_44_10"};


vector<pii> seq_list = {{50,2450},
                        {6000,9000},
                        {5000,8000},
                        {0,2200},
                        {150,2550},
                        {500,3500},
                        {6000,9000},
                        {50,1450},
                        {40,140},
                        {0,140},
                        {0,160},
                        {90,150},
                        {90,170},
                        {50,150},
                        {90,190},
                        {30,80},
                        {80,180},
                        {220,260},
                        {200,220},
                        {120,140},
                        {120,140},
                        {130,160},
                        {180,200},
                        {160,180},
                        {145,165},
                        {120,140},
                        {90,110},
                        {140,160},
                        {180,200},
                        {120,140},
                        {180,200},
                        {80,100},
                        {120,130},
                        {120,140},
                        {150,170},
                        {140,160},
                        {150,170},
                        {240,270},
                        {180,200},
                        {100,200},
                        {80,200},
                        {0,150},
                        {0,150},
                        {0,100},
                        {80,190},
                        {110,190},
                        {50,150},
                        {170,220},
                        {70,100},
                        {90,160},
                        {90,290},
                        {80,150},
                        {70,160},
                        {40,140},
                        {40,80},
                        {230,270},
                        {50,150},
                        {50,70},
                        {130,150},
                        {180,200},
                        {120,140},
                        {155,175},
                        {120,140},
                        {190,210},
                        {140,160},
                        {110,130},
                        {130,150},
                        {80,250},
                        {80,180},
                        {80,160},
                        {50,150},
                        {50,130},
                        {120,190},
                        {100,150},
                        {40,110},
                        {300,350},
                        {360,400},
                        {160,190},
                        {125,155},
                        {120,140},
                        {100,110},
                        {80,100},
                        {90,110},
                        {70,90},
                        {110,130},
                        {130,150},
                        {90,110},
                        {90,110},
                        {230,250},
                        {80,100},
                        {100,130},
                        {110,140},
                        {170,190},
                        {160,190},
                        {30,130},
                        {170,220},
                        {1400,4400},
                        {14000,17000},
                        {1400,4400},
                        {5500,8500},
                        {500, 10500},
                        {130,330},
                        {0,200},
                        {0,250},
                        {0,200},
                        {0,250},
                        {0,130},
                        {0,100},
                        {0,130},
                        {0,100},
                        {0,150},
                        {0,100},
                        {0,130},
                        {0,100},
                        {0,130},
                        {0,110},
                        {40,180},
                        {30,160},
                        {30,140},
                        {30,180},
                        {60,100},
                        {55,95},
                        {50,110},
                        {50,100},
                        {90,150},
                        {50,100},
                        {40,120},
                        {40,100},
                        {90,140},
                        {60,100},
                        {50,100},
                        {40,70},
                        {90,140},
                        {50,100},
                        {70,130},
                        {100,150},
                        {70,120},
                        {60,110},
                        {90,140},
                        {50,100},
                        {50,210},
                        {30,260},
                        {30,140},
                        {0,160},
                        {20,170},
                        {0,170},
                        {0,170},
                        {0,170},
                        {0,160},
                        {0,160},
                        {0,150},
                        {0,160},
                        {0,150},
                        {0,170},
                        {55,100},
                        {90,150},
                        {90,140},
                        {90,130},
                        {95,145},
                        {70,110},
                        {90,130},
                        {90,120},
                        {75,125},
                        {75,120},
                        {100,140},
                        {90,110},
                        {70,100},
                        {60,90},
                        {45,80},
                        {110,140},
                        {130,155},
                        {90,120},
                        {100,130},
                        {100,125},
                        {80,110},
                        {40,80},
                        {100,150},
                        {100,140},
                        {75,105},
                        {100,150},
                        {60,95},
                        {60,100},
                        {80,130},
                        {40,90},
                        {100,140},
                        {70,110},
                        {60,95},
                        {80,130},
                        {70,120},
                        {0,250},
                        {10,350},
                        {10,200},
                        {10,240},
                        {10,260},
                        {10,240},
                        {10,300},
                        {60,330},
                        {30,310},
                        {0,300},
                        {0,300},
                        {95,120},
                        {60,90},
                        {100,140},
                        {110,140},
                        {130,160},
                        {80,120},
                        {100,130},
                        {195,220},
                        {120,150},
                        {140,175},
                        {60,100},
                        {60,110},
                        {60,100},
                        {60,100},
                        {80,110},
                        {90,110},
                        {100,130},
                        {120,140},
                        {105,125},
                        {95,110},
                        {115,135},
                        {150,160},
                        {80,105},
                        {65,95},
                        {100,120},
                        {70,90},
                        {75,95},
                        {75,90},
                        {60,70},
                        {70,80},
                        {55,70},
                        {30,50},
                        {25,55},
                        {45,65},
                        {35,55},
                        {40,70},
                        {40,60},
                        {30,50},
                        {0,30},
                        {40,60},
                        {40,60},
                        {150,170},
                        {20,50},
                        {40,70},
                        {50,85},
                        {90,120},
                        {90,110},
                        {80,105},
                        {95,120},
                        {105,130},
                        {90,110},
                        {90,120},
                        {15,45},
                        {20,50},
                        {30,60},
                        {10,50},
                        {20,50},
                        {35,70},
                        {25,55},
                        {30,60},
                        {30,60},
                        {15,55},
                        {25,55},
                        {25,55},
                        {25,50},
                        {15,60},
                        {0,45},
                        {20,35},
                        {50,75},
                        {35,60},
                        {35,65},
                        {30,65},
                        {30,60},
                        {40,75},
                        {25,50},
                        {55,80},
                        {35,65},
                        {35,65},
                        {35,60},
                        {50,85},
                        {25,55},
                        {30,65},
                        {25,50},
                        {50,85},
                        {20,60},
                        {20,50},
                        {35,65},
                        {20,55},
                        {20,50},
                        {20,50},
                        {30,60},
                        {25,60},
                        {30,70},
                        {30,60},
                        {50,90},
                        {30,50},
                        {60,100},
                        {20,50},
                        {25,55},
                        {50,85},
                        {20,50},
                        {20,50},
                        {70,105},
                        {70,115},
                        {90,125},
                        {70,90},
                        {60,90},
                        {60,90},
                        {100,130},
                        {55,85},
                        {70,110},
                        {55,90},
                        {0,40},
                        {20,50},
                        {20,60},
                        {30,70},
                        {25,60},
                        {25,65},
                        {25,60},
                        {35,80},
                        {0,30},
                        {40,65},
                        {30,60},
                        {10,35},
                        {20,50},
                        {20,50},
                        {20,55},
                        {15,40},
                        {10,40},
                        {15,45},
                        {10,40},
                        {10,40},
                        {15,50},
                        {15,70},
                        {20,50},
                        {20,50},
                        {0,35},
                        {55,85},
                        {20,50},
                        {15,40},
                        {40,70},
                        {40,65},
                        {20,40},
                        {20,40},
                        {20,40},
                        {15,40},
                        {20,50},
                        {20,45},
                        {20,40},
                        {20,40},
                        {20,45},
                        {0,35},
                        {20,60},
                        {40,80},
                        {20,40},
                        {20,50},
                        {20,45},
                        {20,60},
                        {20,55},
                        {20,50},
                        {10,40},
                        {15,45},
                        {65,85},
                        {40,80},
                        {55,90},
                        {40,80},
                        {40,80},
                        {35,60},
                        {55,95},
                        {50,65},
                        {80,120},
                        {55,100},
                        {20,45},
                        {30,50},
                        {20,40},
                        {10,40},
                        {35,55},
                        {40,65},
                        {30,50},
                        {25,50},
                        {30,55},
                        {30,50}};



int main(int argc, char **argv){
    ros::init(argc, argv, "object_detector");
    int NN = data_name_list.size();
    clock_t begin = clock();
    clock_t end = clock();
    for(int i = 0; i<NN;i++){
        if (i<101) continue;
        string data_name = data_name_list[i];
        stringstream data_path;
        data_path << ros::package::getPath("sensor_decoder") << "/data/" << data_name << "/";
        cout << "[" << i + 1 << "/" << NN << ", " << double(end - begin) / CLOCKS_PER_SEC << "] Start to object detection for data named " << data_name << endl; 
        
        string pcd_path = data_path.str() + "pcd/";
        string obj_path = data_path.str() + "object/";
        int st = seq_list[i].first;
        int en = seq_list[i].second;
        int MM = en - st;

        for(int load_seq = 1; load_seq < MM + 1; load_seq++){
            int seq = load_seq + st;
            string pcd_file = pcd_path + zfill(seq) + ".pcd";
            string obj_file = obj_path + zfill(seq) + ".txt";
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file, *cloud) == -1) {
                PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
                return (-1);
            }

            vector<point> grid_map[GRID][GRID];
            for(const auto& p : *cloud) {
                double x = p.x;
                double y = p.y;
                double z = p.z;
                if(x < minX + EPS || x > maxX - EPS || y < minY + EPS || y > maxY - EPS || z < minZ || z > maxZ) continue;
                point t;
                t.x = x;
                t.y = y;
                t.valid = false;
                t.occluded = false;
                pii pixel = xy_to_pixel(t);
                grid_map[pixel.first][pixel.second].push_back(t);
            }
            // clustering
            vector<vector<point>> clusters;
            for(int i = 0; i < GRID; i++){
                for(int j = 0; j < GRID; j++){
                    int sz = grid_map[i][j].size();
                    for(int k = 0; k < sz; k++){
                        if(grid_map[i][j][k].valid) continue;
                        double point_threshold = max(10.0, 50 - 1.5 * (abs(grid_map[i][j][k].x) + abs(grid_map[i][j][k].y)));
                        vector<point> cluster;
                        queue<piii> q;
                        q.push({{i,j},k});
                        while(!q.empty()){
                            int px = q.front().first.first;
                            int py = q.front().first.second;
                            int id = q.front().second;
                            point cur = grid_map[px][py][id];
                            q.pop();
                            if(grid_map[px][py][id].valid) continue;
                            grid_map[px][py][id].valid = true;
                            double dist_threshold = min(1.0, (abs(cur.x) + abs(cur.y))/30.0) * cluster_threshold;
                            cluster.push_back(grid_map[px][py][id]);
                            for(int dx = -R; dx <= R; dx++){
                                for(int dy = -R; dy <= R; dy++){
                                    int nx = px + dx;
                                    int ny = py + dy;
                                    if(nx < 0 || nx >= GRID || ny < 0 || ny >= GRID) continue;
                                    int nsz = grid_map[nx][ny].size();
                                    for(int nk = 0; nk < nsz; nk++){
                                        if(grid_map[nx][ny][nk].valid) continue;
                                        if(dist(cur, grid_map[nx][ny][nk]) < dist_threshold) q.push({{nx, ny}, nk});
                                    }
                                }
                            }
                        }
                        if(cluster.size() > point_threshold) {
                            // cout << cluster.size() << endl;
                            clusters.push_back(cluster);
                        }
                    }
                }
            }
            //build convex hull for each cluster
            vector<ConvexHull> cvh_list;
            for(const vector<point>& v : clusters) cvh_list.push_back(ConvexHull(v));

            // build adj matrix with gjk algorithm
            int N = cvh_list.size();
            // cout << "# of cvh before merge: " << N << endl;
            vector<vector<int>> adj(N);
            for(int i = 0; i < N; i++){
                for(int j = i+1; j < N; j++){
                    if(gjk(cvh_list[i].p, cvh_list[j].p) < convex_hull_threshold){
                        adj[i].push_back(j);
                        adj[j].push_back(i);
                    }
                }
            }
            // merge convex hull
            vector<bool> convex_visited(N, false);
            vector<ConvexHull> obstacles;
            for(int i=0;i<N;i++){
                if(convex_visited[i]) continue;
                vector<ConvexHull> cluster;
                queue<int> q;
                q.push(i);
                while(!q.empty()){
                    int cur = q.front();
                    q.pop();
                    if(convex_visited[cur]) continue;
                    cluster.push_back(cvh_list[cur]);
                    convex_visited[cur] =  true;
                    for(int next : adj[cur]){
                        if(convex_visited[next]) continue;
                        q.push(next);
                    }
                }
                obstacles.push_back(UnionConvexHull(cluster));
            }

            // cout << "# of cvh after merge : " << obstacles.size() << endl;

            // for each cluster, find optimal solution
            ofstream out(obj_file.c_str());
            vector<point> origin;
            origin.push_back(point());
            vector<BoundingBox> boxes;
            for(int i=0;i<obstacles.size();i++) {
                vector<pdi> pa;
                for(int j=0;j<obstacles[i].size;j++) pa.push_back({atan2(obstacles[i].p[i].y, obstacles[i].p[i].x), j});
                sort(pa.begin(), pa.end());
                pdi s = pa[0];
                pdi e = pa[obstacles[i].size-1];
                for(int j=0;j<obstacles[i].size-1;j++){
                    if(pa[j+1].first - pa[j].first > M_PI){
                        s = pa[j];
                        e = pa[j+1];
                        break;
                    }
                }
                ConvexHull c = ConvexHull({point(0,0), obstacles[i].p[s.second], obstacles[i].p[e.second]});
                for(int j=0;j<obstacles[i].size;j++){
                    if(j==s.second || j == e.second) obstacles[i].p[j].occluded = false;
                    else if(isIncludeConvexHull(c, obstacles[i].p[j])) obstacles[i].p[j].occluded = false;
                    else obstacles[i].p[j].occluded = true;
                }
            }
            for(const ConvexHull& cvh : obstacles){
                BoundingBox b = BoundingBox(cvh.center.x, cvh.center.y, 0.0, 5.0, 2.5);
                b.optimize(cvh.p, gjk(cvh.p, origin));
                boxes.push_back(b);
            }
            sort(boxes.begin(), boxes.end());
            for(const BoundingBox& b : boxes){
                out << to_string(b.loss) << " " << to_string(b.box_loss) << " " << to_string(b.x) << " " << to_string(b.y) << " " << to_string(b.theta) << " " << to_string(b.l) << " " << to_string(b.w) << endl;
            }
            out.close();
            end = clock();
            if (load_seq % 100 == 0) cout << "[" << load_seq << "/" << MM << ", " << double(end - begin) / CLOCKS_PER_SEC << "] In progress to object detection" << endl;
        }

    }
        

  return (0);
}
