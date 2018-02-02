#include "mission_control.h"

class XboxControl
{
public:
     XboxControl();

private:
    void ControlCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh;








};
