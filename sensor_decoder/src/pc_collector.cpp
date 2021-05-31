#include <iostream>
#include <fstream>
#include <iomanip>
#include <string.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
 
using namespace std;


string zfill(int n){
  if(n==0) return "000000";
  int digit = log10(n) + 1;
  string rt = "";
  for(int i=0;i<6-digit;i++) rt+="0";
  return rt+to_string(n);
}

class PCCollector{
  private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  string data_name_;
  stringstream result_path_;
  int seq;

  public:
  PCCollector(){
    sub_ = nh_.subscribe("/points_raw", 1, &PCCollector::callback, this);
    data_name_ = "pc";
    result_path_ << ros::package::getPath("sensor_decoder") << "/result/" << data_name_ << "/point_clouds/";
    seq = 0;
  }

  void callback(const sensor_msgs::PointCloud2ConstPtr& msg){
    seq ++;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    string save_path = result_path_.str() + zfill(seq) + ".pcd";
    if(pcl::io::savePCDFileASCII(save_path, cloud) >= 0) cout << "Saved " << save_path << endl;
  }

};
 
int main(int argc,char** argv){
  ros::init(argc, argv, "pc_collector");
  
  PCCollector pc_collector;
  ros::spin();
}

