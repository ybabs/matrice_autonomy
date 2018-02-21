#ifndef MISSION_CONTROL_H
#define MISSION_CONTROL_H

#include "matrice_autonomy/flight_data.h"
#include <iostream>

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

    std::vector<DJI::OSDK::WayPointSettings> CreateWaypoints(int numWaypoints, DJI::OSDK::float64_t distanceIncrement, DJI::OSDK::float32_t start_alt);

    std::vector<DJI::OSDK::WayPointSettings> GeneratePolygon(DJI::OSDK::WayPointSettings* start_data, DJI::OSDK::float64_t distanceIncrement, int numberOfWaypoints);

    void UploadWaypoints(std::vector<DJI::OSDK::WayPointSettings>& wayPoint_list, int responseTimeout, dji_sdk::MissionWaypointTask& waypointTask);

    bool RunWaypointMission(uint8_t numWaypoints, int responseTimeout);

    FlightData flightData;


 private:
    ros::NodeHandle nh;
    ros::ServiceClient ctrl_authority_service;
    ros::ServiceClient drone_task_service;
    ros::ServiceClient query_version_service;
    ros::ServiceClient waypoint_upload_service;
    ros::ServiceClient waypoint_action_service;
    ros::ServiceClient drone_activation_service;


    

    bool CheckM100();  // checks if drone is Matrice 100
    

};







#endif