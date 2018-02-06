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

#define PI (double) 3.141592653589793

// Add the source of this from Zurich ASL..
struct Axes{

  int roll;
  int pitch;
  int throttle;
  int roll_direction;
  int pitch_direction;
  int throttle_direction;

};

struct MaxControlValues
{
  double velocity_xy;
  double roll;
  double pitch;
  double yaw_rate;
  double throttle;

};

struct Buttons
{
  int takeoff;
  int land;
  int obtainControl;
  int current_mode;
  int yaw_left;
  int yaw_right;

};

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


    
   // sensor_msgs::Joy controlVelYawRate;

    //????

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
     Axes axes;

  // Declaration of maximum control values
  MaxControlValues maxValues;

  //offset values
  double current_yaw_velocity;
  double yaw_velocity_delta;



    // Lightbridge 2 Accessible channels
    int roll_channel;
    int pitch_channel;
    int yaw_channel;
    int throttle_channel;
    int mode_switch;
    int landing_gear_switch;

    




};

#endif