#ifndef MISSION_CONTROL_H
#define MISSION_CONTROL_H

// ROS Includes
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>


// Controller Includes
#include <sensor_msgs/Joy.h>
#include <tf/tf.h>

//DJI SDK Includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>

#define EARTH_RADIUS (double)6378137.0
#define PI (double) 3.141592653589793

static double Deg_To_Rad( double degree)
{
    return degree * (PI/180.0);
};

class MissionControl
{

};







#endif