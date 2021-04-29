#include <iostream>
#include <fstream>
#include <iomanip>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <ctime>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include "nrf_project/PCAN.h"

#include "ros/ros.h"
#include "ros/time.h"
#include "ros/package.h"

using namespace std;


int main(int argc, char **argv){
    ros::init(argc, argv, "pcan_topic_generator");
    ros::NodeHandle nh_;
    ros::Publisher pub_ = nh_.advertise<nrf_project::PCAN>("pcan_data", 1);

    stringstream data_path_;
    data_path_ << ros::package::getPath("nrf_project") << "/data/trace.trc";
    vector<nrf_project::PCAN> topic_array;
    const char delimiter = ' ';
    string in_line;
    ifstream in(data_path_.str());
    int seq = 0;
    while(getline(in, in_line)){
        stringstream ss(in_line);
        string token;
        vector<string> tokens;
        while(getline(ss,token,delimiter)) tokens.push_back(token);
        seq ++;
        if(seq > 100000) break;
        nrf_project::PCAN msg;
        msg.header.seq = seq;
        double time = stod(tokens[1])/1000.0;
        msg.header.stamp.sec = int(time);
        msg.header.stamp.nsec = int(1e9*(time-int(time)));
        msg.type = stoul(tokens[3], nullptr, 16);
        msg.size = stoi(tokens[5]);
        vector<int> data;
        for(int i=0;i<msg.size;i++) data.push_back(stoul(tokens[6+i], nullptr, 16));
        msg.data = data;
        topic_array.push_back(msg);
    }
    cout << "complete to pre-setup" << endl;

    double interval = 0.001;
    clock_t cur_time = clock();
    int index = 0;
    while(1){
        if(index == topic_array.size()) break;
        if((double)(clock()-cur_time)/CLOCKS_PER_SEC < interval) continue;
        pub_.publish(topic_array[index]);
        index++;
        cur_time = clock();
    }
    return 0;
}