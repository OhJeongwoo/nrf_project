#include <iostream>

#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"

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
    ros::Subscriber sub_;

    cv_bridge::CvImage bridge;
    
    std_msgs::Header header;

    string data_name;
    stringstream data_path;
    
    string image_raw_path;
    string local_map_path;

    bool valid = true;
    int start = 500;
    int n_data = 100;
    int N = 0;


    
    QueryVisualizer(){
        data_name = "sumin_highway";
        data_path << ros::package::getPath("sensor_decoder") << "/data/" << data_name << "/";
        
        image_raw_path = data_path.str() + "image_raw/";
        local_map_path = data_path.str() + "local_map/";

        sub_ = nh_.subscribe("/query", 1, &QueryVisualizer::callback, this);
        pub_ = nh_.advertise<sensor_msgs::Image>("query_image", 10);
        // query();
    }

    void callback(const std_msgs::Int32MultiArray::ConstPtr& msg){
        cout << "debug" << endl;
        cout << msg->data[0] << endl;
        cout << msg->data[1] << endl;
        start = msg->data[0];
        n_data = msg->data[1];
        N = 0;
        valid = true;
    }

    void query(){
        while(1){
            if (N>10000) break;
            if(!valid) continue;
            int seq = start + N % n_data;
            string cur_image_raw_path = image_raw_path + zfill(seq) + ".png";
            string cur_local_map_path = local_map_path + zfill(seq) + ".png";

            Mat image_raw = cv::imread(cur_image_raw_path);
            Mat local_map = cv::imread(cur_local_map_path);
            if(image_raw.size().height == 0 || local_map.size().height == 0) continue;
            Mat out;

            cv::resize(image_raw, image_raw, Size(300,300), 0, 0);
            cv::resize(local_map, local_map, Size(300,300), 0, 0);

            Mat arr[2] = {image_raw, local_map};
            cv::hconcat(arr,2,out);
            sensor_msgs::Image rt;
            bridge = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, out);
            bridge.toImageMsg(rt);
            pub_.publish(rt);
            N++;
            sleep(0.5);
        }
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "query_viz_tool");
    QueryVisualizer q;
    ros::Rate loop_rate(10);
    while(ros::ok()){
        if(!q.valid) continue;
        int seq = q.start + q.N % q.n_data;
        string cur_image_raw_path = q.image_raw_path + zfill(seq) + ".png";
        string cur_local_map_path = q.local_map_path + zfill(seq) + ".png";

        Mat image_raw = cv::imread(cur_image_raw_path);
        Mat local_map = cv::imread(cur_local_map_path);
        if(image_raw.size().height == 0 || local_map.size().height == 0) continue;
        Mat out;

        cv::resize(image_raw, image_raw, Size(300,300), 0, 0);
        cv::resize(local_map, local_map, Size(300,300), 0, 0);

        Mat arr[2] = {image_raw, local_map};
        cv::hconcat(arr,2,out);
        sensor_msgs::Image rt;
        q.bridge = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, out);
        q.bridge.toImageMsg(rt);
        q.pub_.publish(rt);
        q.N++;
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
}
