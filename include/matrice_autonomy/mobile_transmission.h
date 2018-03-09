#ifndef MOBILE_TRANSMISSION_H
#define MOBILE_TRANSMISSION_H

// msgs
#include <dji_sdk/MobileData.h>
#include <sensor_msgs/Joy.h>
// DJI SDK includes
#include <dji_sdk/Activation.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/MissionWpAction.h>
#include <dji_sdk/MissionHpAction.h>
#include <dji_sdk/MissionWpUpload.h>
#include <dji_sdk/MissionHpUpload.h>
#include <dji_sdk/MissionHpUpdateRadius.h>
#include <dji_sdk/MissionHpUpdateYawRate.h>
#include <dji_sdk/DroneArmControl.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SendMobileData.h>
#include <dji_sdk/QueryDroneVersion.h>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/UInt8.h>
#include <tf/tf.h>
// SDK library
#include <djiosdk/dji_vehicle.hpp>



#pragma pack(1)

typedef struct AckSendToMobile
{
uint16_t cmdID;
uint16_t ack;
}AckSendToMobile;


typedef struct AckStringToMobile
{
std::string text;
uint16_t ack;
}AckStringToMobile;

#pragma pack()

class MobileTransmission
{ 
    public:
     MobileTransmission();
     void SendToMobile(AckSendToMobile sendToMobile); 
     void MobileDataSubscriberCallback(const dji_sdk::MobileData::ConstPtr& mobile_data);
     void SendText(unsigned char* dataToMobile);


     private:
      ros::NodeHandle mnh;
      ros::Subscriber MobileDataSubscriber;
      ros::ServiceClient mobile_data_service;
      dji_sdk::MobileData mobile_data;
};


































#endif