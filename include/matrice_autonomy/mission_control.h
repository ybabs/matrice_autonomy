#ifndef MISSION_CONTROL_H
#define MISSION_CONTROL_H

#include "sensors_process.h"

static double Deg_To_Rad( double degree)
{
    return degree * (PI/180.0);
};

class MissionControl
{
public:
    MissionControl();
    void TakeOff();
    void Land();
    void GoHome();
    void MonitoredTakeoff();



 private:

};







#endif