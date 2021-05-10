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

#include "sensor_decoder/PCAN.h"
#include "sensor_decoder/PCANArray.h"

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

typedef struct can_frame CAN;

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
int ID;

vector<struct can_frame> DATA;

int main(int argc, char **argv){
    ros::init(argc, argv, "pcan_reader");
    ros::NodeHandle nh_;
    ros::Publisher pub_ = nh_.advertise<sensor_decoder::PCANArray>("pcan_data", 1);

    ros::Time init_time = ros::Time::now();
    sensor_decoder::PCANArray rt;
    rt.header.seq = 0;

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
            
            ID = CAN_FRAME_CH0.can_id;
            if (ID == 0x700){
                //reset
                rt.header.seq ++;
                rt.header.stamp = ros::Time::now();
                DATA.clear();
            }
            else if(ID == 0x728){
                if(rt.header.seq == 0) continue;
                rt.size = DATA.size();
                vector<sensor_decoder::PCAN> pcan_array;
                for(struct can_frame data : DATA){
                    sensor_decoder::PCAN tmp;
                    tmp.type = data.can_id;
                    tmp.size = data.can_dlc;
                    for(int i = 0; i < data.can_dlc; i++) tmp.data.push_back(data.data[i]); 
                    pcan_array.push_back(tmp);
                }
                rt.messages = pcan_array;
                pub_.publish(rt);
            }
            DATA.push_back(CAN_FRAME_CH0);
        // ID = CAN_FRAME_CH0.can_id;
	    // if((ID >> 8) - ((ID >> 12) << 4) != 7) continue;
   	    // cout << ID << endl;
	    // sensor_decoder::PCAN rt;

        //     //rt.header.stamp = ros::Time::now();
        //     //rt.header.seq = seq;

        //     rt.type = ID;
        //     rt.size = CAN_FRAME_CH0.can_dlc;

        //     //vector<int> data_array;
        //     //for(int i = 0; i < rt.size; i++) data_array.push_back(CAN_FRAME_CH0.data[i]);
        //     //rt.data = CAN_FRAME_CH0.data;
	    // copy(begin(CAN_FRAME_CH0.data), end(CAN_FRAME_CH0.data), back_inserter(rt.data));

            // pub_.publish(rt);
        }
        else ROS_ERROR("NO DATA");
    }

    return 0;
}
