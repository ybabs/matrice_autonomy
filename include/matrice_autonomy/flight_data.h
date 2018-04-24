#ifndef SENSORS_PROCESS_H
#define SENSORS_PROCESS_H

// ROS Includes
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include "matrice_autonomy/mobile_comm.h"

// Controller Includes
#include <sensor_msgs/Joy.h>
#include <tf/tf.h>

//DJI SDK Includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/MissionWpAction.h>
#include <dji_sdk/MissionWpUpload.h>
#include <dji_sdk/Activation.h>


//DJI OSDK Includes
#include <djiosdk/dji_vehicle.hpp>


#include "dji_sdk/dji_sdk.h"

#define EARTH_RADIUS (double)6378137.0
#define PI (double) 3.141592653589793


  // Lightbridge 2 Accessible channels
  struct Lightbridge
  {
    int roll_channel;
    int pitch_channel;
    int yaw_channel;
    int throttle_channel;
    int mode_switch;
    int landing_gear_switch;

    enum{MODE_P = -8000, MODE_A = 0, MODE_F = 8000} ModeToggle;
  
  };

class FlightData
{

  public:
  FlightData();
   //sensor_msgs::NavSatFix current_gps_location; 

  // accessor to return GPS Position
  sensor_msgs::NavSatFix GetGPSPosition();

  private:
  // global position of MAtrice M100 based on WGS84 System (Lat, Lon, Alt (m))
  sensor_msgs::NavSatFix current_gps_location; 
  // GPS Signal Health between 0 and 5 being the best
  uint8_t gps_health;
  //IMU Data in FLU(Forward Left Up Frame) (raw gyro, Raw Acc, and attitude estimation at 100Hz)
  sensor_msgs::Imu imu_data;
  // Flight status of the UAV 
  uint8_t flight_status;


  // Drone Attitude as a quaternion
  geometry_msgs::Quaternion attitude_data;
  // Battery State
  int batteryLeft;

  // Velocity Data
  geometry_msgs::Vector3Stamped velocity_data;

    // Height above takeoff
   float takeoff_height;
  

    // Callbacks to return flight data
    //return attitude data
  void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);
  // returg GPS data
  void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
 // Return GPS Health data
  void gps_health_callback(const std_msgs::UInt8::ConstPtr& msg);
// Return IMU data
  void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
 // return flight status data
  void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);
  // return value of RC Channelss
  void lightbridge_callback(const sensor_msgs::Joy::ConstPtr& joy); // Subscribe to Lightbridge Data
// convert Attitude data from quaternion to Euler angles
 geometry_msgs::Vector3 attitudeEuler(geometry_msgs::Quaternion attitude_quat);
// Check Lightbridge control mode
 void checkLightbridgeControlMode();
 // battery state callback
 void batteryState_callback(const sensor_msgs::BatteryState::ConstPtr& msg);

void height_callback(const std_msgs::Float32::ConstPtr& msg);

void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
// To send FLight Data To the Tablet.
 MobileComm mobileCommManager;


 /* Subscribers */
   ros::Subscriber attitude_sub;
   ros::Subscriber gps_sub;
   ros::Subscriber flightStatus_sub;
   ros::Subscriber gpsHealth_sub;
   ros::Subscriber imu_sub;
   ros::Subscriber lightbridge_sub;
   ros::Subscriber batteryState_sub;
   ros::Subscriber velocity_sub;
   ros::Subscriber height_sub;
   



   // RC Controller Data
   Lightbridge lightbridge;

   // NodeHandle
   ros::NodeHandle fnh;


  

};

#endif