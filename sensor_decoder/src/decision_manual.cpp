#include <iostream>
#include <stdlib.h>

#include <std_msgs/Int32.h>


#include "ros/ros.h"
#include "ros/time.h"
#include "ros/package.h"

using namespace std;

string get_mode(int x){
    if (x==1) return "KEEPING LANE";
    if (x==2) return "CHANGING LEFT";
    if (x==3) return "CHANGING RIGHT";
    if (x==4) return "STOP";
    cout << "[ERROR] WRONG MODE" << endl;
    return "error";
}

int main(int argc, char **argv){
    ros::init(argc, argv, "local_map_publisher");
    
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Int32>("/decision", 1); 
    std_msgs::Int32 rt;
    int signal = 1;
    while(true){
        for(int i=1;i<=4;i++) cout << to_string(i) << " : " << get_mode(i) << endl;
        cout << "current mode is " << "\033[1;31m" << get_mode(signal)<< "\033[0m" << endl;
        cout << "PLEASE INPUT MODE..." << endl;
        cin >> signal;
        if(get_mode(signal) == "error") continue;
        rt.data = signal;
        pub.publish(rt);
    }
    
    ros::spin();
}