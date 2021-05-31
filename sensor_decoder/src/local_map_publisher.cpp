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

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

const double PI = 3.14159265359;

typedef pair<double, double> pdd;

struct Lane{
    double c0,c1,c2,c3;

    Lane(){
        this->c0 = 0;
        this->c1 = 0;
        this->c2 = 0;
        this->c3 = 0;
    }

    Lane(double c0, double c1, double c2, double c3){
        this->c0 = c0;
        this->c1 = c1;
        this->c2 = c2;
        this->c3 = c3;
    }

    double get_x(double z){
        return c0 + z*c1 + z*z*c2 + z*z*z*c3;
    }
};

string zfill(int n){
    if(n==0) return "000000";
    int digit = log10(n) + 1;
    string rt = "";
    for(int i=0;i<6-digit;i++) rt += "0";
    return rt+to_string(n);
}

double clip(double value, double limit, int type){
    limit = abs(limit);
    if(abs(value)>limit) value = value / abs(value);
    else value = value / limit;
    if(type == 0) return abs(value);
    else return value / 2 + 0.5;
}

class LocalMapPublisher{
    private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    // ros::Subscriber sub_imu_;
    ros::Subscriber sub_img_;
    double width_;
    double height_;
    double resolution_;
    int img_width_;
    int img_height_;
    cv_bridge::CvImage img_bridge;
    Mat local_map_;
    // double heading_;
    // sensor_msgs::Image cur_image_;
    cv_bridge::CvImagePtr cv_ptr_;
    stringstream result_path_;
    string data_name_;

    public:
    LocalMapPublisher(){
        data_name_ = "gunmin_04";
        //result_path_ << "/media/jeongwoooh/40820CAB820CA80C" << "/result/" << data_name_;
	result_path_ << ros::package::getPath("sensor_decoder") <<"/result/"<<data_name_;
        width_ = 60.0;
        height_ = 60.0;
        resolution_ = 0.1;
        img_width_ = int(width_ / resolution_);
        img_height_ = int(height_ / resolution_);

        sub_img_ = nh_.subscribe("/camera/color/image_raw", 1, &LocalMapPublisher::callback_image, this);
        
        sleep(1);
        // ROS_INFO("DEBUG1");
        
        pub_ = nh_.advertise<sensor_msgs::Image>("/local_map/image_raw", 10);
        // ROS_INFO("DEBUG2");
        sub_ = nh_.subscribe("/mobileye", 1, &LocalMapPublisher::callback, this);
        // ROS_INFO("DEBUG3");
        // ROS_INFO("DEBUG4");
    }

    cv::Point xy_to_pixel(pdd pos){
        return cv::Point(img_width_/2 + int(pos.first/resolution_), img_height_ - int(pos.second/resolution_));
    }

    void draw_lane(const sensor_decoder::LaneMsg& msg){
        vector<pdd> points;
        Lane lane = Lane(msg.c[0], msg.c[1], msg.c[2], msg.c[3]);
        for(int i = 0; i < 500; i++) cv::circle(local_map_, xy_to_pixel({lane.get_x(0.1*i), 0.1*i}), 1.0, Scalar(0, 255, 0), -1);
    }

    void draw_obstacle(const sensor_decoder::ObstacleMsg& msg){
        cv::RotatedRect obstacle = cv::RotatedRect(xy_to_pixel({-msg.y, msg.x}), cv::Size2f(msg.width / resolution_, msg.width / resolution_), msg.theta);
        // cout << msg.width << " " << msg.length << " " << msg.theta << endl;
        // cout << msg.x << " " << msg.y << endl;
        cv::Point2f vertices[4];
        obstacle.points(vertices);
        for (int i = 0; i < 4; i++) cv::line(local_map_, vertices[i], vertices[(i+1)%4], cv::Scalar(0, 0, 255), 3);
    }

    void callback_image(const sensor_msgs::Image::ConstPtr& msg){
        cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }

    
    void callback(const sensor_decoder::Mobileye::ConstPtr& msg){
        if(!msg->valid) return;

        local_map_ = Mat(img_height_, img_width_, CV_8UC3, Scalar(255,255,255));

        // dotted line for distance intuitive
        for(int i = -4; i < 4; i++) cv::line(local_map_, xy_to_pixel({10.0 * i, 0}), xy_to_pixel({10.0 * i, height_}), cv::Scalar(0,0,0), 1);
        for(int i = 0; i < 6; i++) cv::line(local_map_, xy_to_pixel({-width_/2, 10.0 * i}), xy_to_pixel({width_/2, 10.0 * i}), cv::Scalar(0,0,0), 1);

        // draw circle for heading
        cv::circle(local_map_, Point(100,100), 50.0, Scalar(0,0,0), 2);

        // draw rectangle for indication of velocity, acceleration x-axis, acceleration y-axis, angular velocity
        cv::rectangle(local_map_, Point(50,180), Point(150,200), Scalar(0,0,0), 2);
        cv::rectangle(local_map_, Point(50,220), Point(150,240), Scalar(0,0,0), 2);
        cv::rectangle(local_map_, Point(50,260), Point(150,280), Scalar(0,0,0), 2);
        cv::rectangle(local_map_, Point(50,300), Point(150,320), Scalar(0,0,0), 2);
                

        // draw heading direction
        cv::line(local_map_, Point(100,100), Point(int(100+30.0*cos(msg->theta)), int(100+30.0*sin(msg->theta))), cv::Scalar(255, 0, 0), 1);

        // draw velocity line
        cv::line(local_map_, Point(50 + int(100*clip(msg->v, 20, 0)), 180), Point(50 + int(100*clip(msg->v, 20, 0)), 200), cv::Scalar(255, 0, 0), 2);

        // draw acceleration x-axis line
        cv::line(local_map_, Point(50 + int(100*clip(msg->ax, 5, 1)), 220), Point(50 + int(100*clip(msg->ax, 5, 1)), 240), cv::Scalar(255, 0, 0), 2);

        // draw acceleration y-axis line
        cv::line(local_map_, Point(50 + int(100*clip(msg->ay, 5, 1)), 260), Point(50 + int(100*clip(msg->ay, 5, 1)), 280), cv::Scalar(255, 0, 0), 2);

        // draw angular velocity line
        cv::line(local_map_, Point(50 + int(100*clip(msg->omega, 1, 1)), 300), Point(50 + int(100*clip(msg->omega, 1, 1)), 320), cv::Scalar(255, 0, 0), 2);

        // our position
        cv::circle(local_map_, xy_to_pixel({0,0}), 5.0, Scalar(255, 0, 0), -1);

        // left lane
        draw_lane(msg->left_lane);

        // right lane
        draw_lane(msg->right_lane);

        // next lane
        for(int i = 0; i < msg->n_next_lanes; i++) draw_lane(msg->next_lanes[i]);

        // obstacles
        // cout << msg->n_obstacles << endl;
        for(int i = 0; i < msg->n_obstacles; i++) draw_obstacle(msg->obstacles[i]);

        sensor_msgs::Image rt;
        img_bridge = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::RGB8, local_map_);
        img_bridge.toImageMsg(rt);

        string raw_image_save_path = result_path_.str() + "/image_raw/" + zfill(msg->header.seq) + ".png";
        string local_map_save_path = result_path_.str() + "/local_map/" + zfill(msg->header.seq) + ".png"; 
        cv::imwrite(raw_image_save_path, cv_ptr_->image);
        cv::imwrite(local_map_save_path, local_map_);

        pub_.publish(rt);
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "local_map_publisher");
    LocalMapPublisher local_map_publisher;
    ros::spin();
}
