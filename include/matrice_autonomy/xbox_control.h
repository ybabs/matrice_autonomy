#ifndef XBOX_CONTROL_H
#define XBOX_CONTROL_H

// #include "mission_control.h"
// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>

// DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>

#include <tf/tf.h>
#include <sensor_msgs/Joy.h>
#include "dji_sdk/dji_sdk.h"

class XboxControl
{
public:
     XboxControl();
     void Takeoff();
     void Land();
     void ObtainControl();

private:
    void ControlCallback(const sensor_msgs::Joy::ConstPtr& joy); //Callback to use Xbox360 Controller to control Matrice 100
    void RC_Callback(const sensor_msgs::Joy::ConstPtr& joy); // Subscribe to Lightbridge Data
    bool CheckM100();  // checks if drone is Matrice 100
    
    ros::NodeHandle nh;
    ros::Subscriber joy_sub; // Subscriber to the Joy Package
    ros::Publisher  vel_pub; // Velocity Publisher
    ros::Publisher takeoff_pub; // publish takeoff message
    ros::Publisher land_pub; // Publish landing message
    ros::Subscriber rc_sub; // Subscribing to the RC Channel

    ros::ServiceClient ctrl_authority_service;
    ros::ServiceClient drone_task_service;
    ros::ServiceClient query_version_service;
    

  /// Axes to control the Matrice
    int linear, angular;
    double linear_scale, angular_scale;

    // Lightbridge 2 Accessible channels
    double roll_channel;
    double pitch_channel;
    double yaw_channel;
    double throttle_channel;
    double mode_switch;
    double landing_gear_switch;

    




};

#endif