#include <iostream>

#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
#include "sensor_decoder/Data.h"
#include "sensor_decoder/Object.h"
#include "sensor_decoder/Label.h"

#include "ros/ros.h"
#include "ros/time.h"
#include "ros/package.h"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>


using namespace std;
using namespace cv;

typedef pair<double, double> pdd;


string zfill(int n){
    if(n==0) return "000000";
    int digit = log10(n) + 1;
    string rt = "";
    for(int i=0;i<6-digit;i++) rt += "0";
    return rt+to_string(n);
}

class QueryVisualizer{
    public:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_data_;
    ros::Subscriber sub_label_;

    cv_bridge::CvImage bridge;
    
    std_msgs::Header header;

    string data_name;
    stringstream data_path;
    
    string img_path;
    string pcd_path;

    bool valid_data;
    bool valid_label;
    int seq;

    Mat bev_map;
    Mat img_raw;

    int height = 600;
    int width = 600;
    double resolution = 0.1;

    double minX = -10.0;
    double maxX = 20.0;
    double minY = -10.0;
    double maxY = 10.0;
    double minZ = 0.5;
    double maxZ = 2.0;

    
    QueryVisualizer(){
        data_name = "";
        data_path << ros::package::getPath("sensor_decoder") << "/data/" << data_name << "/";
        valid_data = false;
        valid_label = false;

        img_path = data_path.str() + "image_raw/";
        pcd_path = data_path.str() + "local_map/";

        sub_data_ = nh_.subscribe("/query_data", 1, &QueryVisualizer::callback_data, this);
        sub_label_ = nh_.subscribe("/query_label", 1, &QueryVisualizer::callback_label, this);
        pub_ = nh_.advertise<sensor_msgs::Image>("query_image", 10);
        // query();
    }

    cv::Point xy_to_pixel(pdd pos){
        return cv::Point(width/2 + int(pos.first/resolution), height - int(pos.second/resolution));
    }

    void draw_object(const sensor_decoder::Object& obj){
        
    }

    void callback_data(const sensor_decoder::Data::ConstPtr& msg){
        data_name = msg->name;
        data_path << ros::package::getPath("sensor_decoder") << "/data" << data_name << "/";

        img_path = data_path.str() + "image_raw/";
        pcd_path = data_path.str() + "pcd/";

        string cur_image_raw_path = image_raw_path + zfill(seq) + ".png";
        img_raw = cv::imread(cur_image_raw_path);

        bev_map = Mat(height, width, CV_8UC3, Scalar(255,255,255));

        string pcd_file = pcd_path + zfill(seq) + ".pcd";

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file, *cloud) == -1) {
            PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
            return (-1);
        }

        for(const auto& p : *cloud) {
            double x = p.x;
            double y = p.y;
            double z = p.z;
            if(x < minX + EPS || x > maxX - EPS || y < minY + EPS || y > maxY - EPS || z < minZ || z > maxZ) continue;
            
            cv::Point pixel = xy_to_pixel({x, y});
            bev_map.at<cv::Vec3b>(pixel[1], pixel[0])[0] = 0;
            bev_map.at<cv::Vec3b>(pixel[1], pixel[0])[1] = 0;
            bev_map.at<cv::Vec3b>(pixel[1], pixel[0])[2] = 0;
        }


        valid = true;
    }

    void callback_label(const sensor_decoder::Label::ConstPtr& msg){
        

        return;
    }



    void query(){
        if(!valid_data || !valid_label) return;

        

        // Mat local_map = pc_map;

        
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "query_viz_tool");
    QueryVisualizer q;
    ros::Rate loop_rate(10);
    while(ros::ok()){
        // q.query();
        // if(!q.valid) continue;
        // int seq = q.start + q.N % q.n_data;
        // string cur_image_raw_path = q.image_raw_path + zfill(seq) + ".png";
        // string cur_local_map_path = q.local_map_path + zfill(seq) + ".png";

        // Mat image_raw = cv::imread(cur_image_raw_path);
        // Mat local_map = Mat(img_height_, img_width_, CV_8UC3, Scalar(255,255,255));
        // Mat local_map = cv::imread(cur_local_map_path);
        // if(image_raw.size().height == 0 || local_map.size().height == 0) continue;
        // Mat out;

        // cv::resize(image_raw, image_raw, Size(300,300), 0, 0);
        // cv::resize(local_map, local_map, Size(300,300), 0, 0);

        // Mat arr[2] = {image_raw, local_map};
        // cv::hconcat(arr,2,out);
        // sensor_msgs::Image rt;
        // q.bridge = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, out);
        // q.bridge.toImageMsg(rt);
        // q.pub_.publish(rt);
        // q.N++;
        // ros::spinOnce();
        // loop_rate.sleep();
    }
    ros::spin();
}
