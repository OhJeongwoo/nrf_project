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

#include "nrf_project/Mobileye.h"
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

string zfill(int n){
    if(n==0) return "000000";
    int digit = log10(n) + 1;
    string rt = "";
    for(int i=0;i<6-digit;i++) rt += "0";
    return rt+to_string(n);
}


class ImageSaver{
    private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    cv_bridge::CvImagePtr cv_ptr_;
    stringstream result_path_;
    string data_name_;
    int seq = 0;

    public:
    ImageSaver(){
        sub_ = nh_.subscribe("/camera/color/image_raw", 1, &ImageSaver::callback, this);
        data_name_ = "test01";
        result_path_ << ros::package::getPath("nrf_project") << "/result/" << data_name_;
    }

    void callback(const sensor_msgs::Image::ConstPtr& msg){
        seq++;
        cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        string raw_image_save_path = result_path_.str() + "/image_raw/" + zfill(seq) + ".png";
        cv::imwrite(raw_image_save_path, cv_ptr_->image);
    }
};


int main(int argc, char **argv){
    ros::init(argc, argv, "test_image_save");
    ImageSaver test_image_save;
    ros::spin();
}