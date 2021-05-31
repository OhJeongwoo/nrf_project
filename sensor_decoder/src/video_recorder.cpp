#include <iostream>

#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

#include "ros/ros.h"
#include "ros/time.h"
#include "ros/package.h"

#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include "cv_bridge/cv_bridge.h"

using namespace std;
using namespace cv;


class VideoRecoder{
    private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    cv_bridge::CvImagePtr cv_ptr_;
    stringstream result_path_;
    string data_name_;
    int seq;
    VideoWriter writer;
    double fps = 15.0; // framerate of the created video stream

    public:
    VideoRecoder(){
        sub_ = nh_.subscribe("/camera/color/image_raw", 1, &VideoRecoder::callback, this);
        data_name_ = "walking_6km_01";
        result_path_ << ros::package::getPath("sensor_decoder") << "/result/graphics_project/";

        seq = 0;
        int codec = VideoWriter::fourcc('M', 'J', 'P', 'G');  // select desired codec (must be available at runtime)
        string filename = result_path_.str() + data_name_ + ".avi";             // name of the output video file
        writer.open(filename, codec, fps, cv::Size(640,480));
    }

    void callback(const sensor_msgs::Image::ConstPtr& msg){
        seq++;
        if(seq > 1100) return;
        cout << seq << endl;
        if (seq >= 150 && seq < 1050){
            cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            writer.write(cv_ptr_->image);
        }
        if (seq == 1100) {
            writer.release();
            cout << "save done" << endl;
        }
    }

};

int main(int argc, char **argv){
    ros::init(argc, argv, "video_recorder");
    VideoRecoder video_recorder;
    ros::spin();
}
