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

#include "nrf_project/PCAN.h"

#include "ros/ros.h"
#include "ros/time.h"
#include "ros/package.h"

#include <net/if.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/ip.h>
#include <netinet/in.h>
#include <netdb.h>

#include <linux/can.h>
#include <linux/can/raw.h>

using namespace std;


//CAN VARIABLES
int socket_CAN_CH0;
int socket_CAN_CH1; 
int CAN_READ_BYTES_CH0;
int CAN_READ_BYTES_CH1;
struct sockaddr_can CAN_ADDR_CH0;
struct sockaddr_can CAN_ADDR_CH1;
struct ifreq CAN_IFR_CH0;
struct ifreq CAN_IFR_CH1;
struct can_frame CAN_FRAME_CH0;
struct can_frame CAN_FRAME_CH1; 


int main(int argc, char **argv){
    ros::init(argc, argv, "pcan_reader");
    ros::NodeHandle nh_;
    ros::Publisher pub_ = nh_.advertise<nrf_project::PCAN>("pcan_data", 1);

    ros::Time init_time = ros::Time::now();
    int seq = 0;

    if ((socket_CAN_CH0 = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) perror("Socket");
		
	strcpy(CAN_IFR_CH0.ifr_name, "can0" );
	ioctl(socket_CAN_CH0, SIOCGIFINDEX, &CAN_IFR_CH0);

	memset(&CAN_ADDR_CH0, 0, sizeof(CAN_ADDR_CH0));
	CAN_ADDR_CH0.can_family = AF_CAN;
	CAN_ADDR_CH0.can_ifindex = CAN_IFR_CH0.ifr_ifindex;

	if (bind(socket_CAN_CH0, (struct sockaddr *)&CAN_ADDR_CH0, sizeof(CAN_ADDR_CH0)) < 0) perror("Bind");

    while (1) {
        //CAN READ
		CAN_READ_BYTES_CH0 = read(socket_CAN_CH0, &CAN_FRAME_CH0, sizeof(struct can_frame));

        if (CAN_READ_BYTES_CH0 > 0){
            seq++;
            nrf_project::PCAN rt;

            rt.header.stamp = ros::Time::now();
            rt.header.seq = seq;

            rt.type = (int)CAN_FRAME_CH0.can_id;
            rt.size = (int)CAN_FRAME_CH0.can_dlc;

            vector<int> data_array;
            for(int i = 0; i < rt.size; i++) data_array.push_back(CAN_FRAME_CH0.data[i]);
            rt.data = data_array;

            pub_.publish(rt);
        }
        else ROS_ERROR("NO DATA");
    }

    return 0;
}