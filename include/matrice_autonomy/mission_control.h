#ifndef MISSION_CONTROL_H
#define MISSION_CONTROL_H

#include "matrice_autonomy/mobile_comm.h"
#include <iostream>
#include <dji_sdk/dji_sdk.h>
#include <dji_sdk/MobileData.h>
#include <dji_sdk/SendMobileData.h>
#include "matrice_autonomy/flight_data.h"
#include <iostream>
#include <cstring>
#include <algorithm>
#include <iomanip>

//DJI SDK DRONE LIBRARY
#include <djiosdk/dji_vehicle.hpp>

// ROS Include
#include <ros/ros.h>

#define PI (double) 3.141592653589793


// Include other services.

static double Deg_To_Rad( double degree)
{
    return degree * (PI/180.0);
};

static double Rad_To_Deg(double rad)
{
  return rad * (180.0/PI);  
};


// Define a service acknowledgement mission here..


class MissionControl
{
public:
    MissionControl();
    void Activate();
    void TakeOff();
    void Land();
    void GoHome();
    void MonitoredTakeoff();
    void ObtainControl();
    bool InitWayPointMission(dji_sdk::MissionWaypointTask& waypointTask);

    void MissionAction(DJI::OSDK::DJI_MISSION_TYPE type, DJI::OSDK::MISSION_ACTION action);

    void SetWayPointDefaults(DJI::OSDK::WayPointSettings* m_waypoint);

    void SetWayPointInitDefaults(dji_sdk::MissionWaypointTask& wayPointTask);

    void UploadWaypoints(std::vector<DJI::OSDK::WayPointSettings>& wayPoint_list, int responseTimeout, dji_sdk::MissionWaypointTask& waypointTask);

    bool RunWaypointMission(int responseTimeout);

    void SetGPSPosition(sensor_msgs::NavSatFix);

    void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);

    // void SendDataToMobile(unsigned char* data_to_mobile);

    // void SendDataToMobile(AckReturnToMobile returnMobileAck);

    // void SendText(std::string data_to_mobile);

   void MobileDataSubscriberCallback(const dji_sdk::MobileData::ConstPtr& from_mobile_data);

    
    

    
 private:
    ros::NodeHandle nh;
    ros::ServiceClient ctrl_authority_service;
    ros::ServiceClient drone_task_service;
    ros::ServiceClient query_version_service;
    ros::ServiceClient waypoint_upload_service;
    ros::ServiceClient waypoint_action_service;
    ros::ServiceClient drone_activation_service;
    ros::Subscriber gps_sub;
    ros::ServiceClient mobile_data_service;        // Services
    ros::Subscriber mobile_data_subscriber; // Data Subscribe


    std::string string_to_mobile; 
    dji_sdk::MobileData data_from_mobile;
    sensor_msgs::NavSatFix gps_pos;
    unsigned char data_to_mobile[10];
    int waypoint_index;

// timeout of response for running mission;
    int responseTimeout;

    // Vector to store the waypoints
    std::vector<DJI::OSDK::WayPointSettings> flightWaypointList;

    // Waypoint task to set Waypoint options for the drone
     dji_sdk::MissionWaypointTask waypointTask;

    double latitude;
    double longitude;
    float altitude;
    float speed;

    unsigned char latitude_array[8] = {0};
    unsigned char longitude_array[8] = {0};
    unsigned char altitude_array[4] = {0};
    unsigned char orientation;
    unsigned char speed_array[4] = {0};
    unsigned char missionEnd = 0;

    

    bool CheckM100();  // checks if drone is Matrice 100
    

};







#endif