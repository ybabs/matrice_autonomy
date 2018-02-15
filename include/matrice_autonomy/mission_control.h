#ifndef MISSION_CONTROL_H
#define MISSION_CONTROL_H

#include "flight_data.h"
static double Deg_To_Rad( double degree)
{
    return degree * (PI/180.0);
};

static double Rad_To_Deg(double rad)
{
  return rad * (180.0/PI);  
};

class MissionControl
{
public:
    MissionControl();
    void TakeOff();
    void Land();
    void GoHome();
    void MonitoredTakeoff();
    void ObtainControl();

 private:
    ros::NodeHandle nh;
    ros::ServiceClient ctrl_authority_service;
    ros::ServiceClient drone_task_service;
    ros::ServiceClient query_version_service;

    bool CheckM100();  // checks if drone is Matrice 100
    

};







#endif