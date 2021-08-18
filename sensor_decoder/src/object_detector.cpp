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
                ,"0815_exp_jeongwoo_highway_2"};

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
            {5500,8500}};

int main(int argc, char **argv){
    ros::init(argc, argv, "object_detector");
    int NN = data_name_list.size();
    clock_t begin = clock();
    clock_t end = clock();
    for(int i = 0; i<NN;i++){
        if (i<96) continue;
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
