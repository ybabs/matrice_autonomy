#ifndef MOBILE_COMM_H
#define MOBILE_COMM_H

// DJI SDK INCLUDE
#include <dji_sdk/dji_sdk.h>
#include <dji_sdk/MobileData.h>
#include <dji_sdk/SendMobileData.h>
#include <iostream>
#include <cstring>
#include <algorithm>
#include <iomanip>

//DJI SDK DRONE LIBRARY
#include <djiosdk/dji_vehicle.hpp>

// ROS Include
#include <ros/ros.h>

#pragma pack(1)
typedef struct AckReturnToMobile{
  uint16_t cmdID;
  uint16_t ack;
} AckReturnToMobile;
#pragma pack()


class MobileComm
{

    private:
    ros::ServiceClient mobile_data_service;        // Services
    ros::Subscriber mobile_data_subscriber; // Data Subscribe
    std::string string_to_mobile; 
    dji_sdk::MobileData data_from_mobile;
   



    public:
    ros::NodeHandle nh;
    MobileComm();
    void SendDataToMobile(unsigned char* data_to_mobile);
    void SendDataToMobile(AckReturnToMobile returnMobileAck);
    void SendText(std::string data_to_mobile);
   
    

};



#endif
