#include "flight_data.h"


FlightData::FlightData()
{
   attitude_sub = fnh.subscribe("dji_sdk/attitude", 10, &attitude_callback, this);
   gps_sub = fnh.subscribe("dji_sdk/gps_position", 10, &gps_callback, this);
   flightStatus_sub = fnh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback, this);
   gpsHealth_sub =fnh.subscribe("dji_sdk/gps_health", 10, &gps_health_callback, this);
   imu_sub = fnh.subscribe("dji_sdk/imu", 10, &imu_callback, this);
   batteryState_sub = fnh.subscribe("dji_sdk/battery_state", 10, &batteryState_callback, this);
   lightbridge_sub = fnh.subscribe("/dji_sdk/rc", 10, &lightbridge_callback, this);
}

void FlightData::attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  attitude_data = msg->quaternion;
}

void FlightData::batteryState_callback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
  
}

void FlightData::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  current_gps_location.latitude = msg->latitude;
  current_gps_location.longitude = msg->longitude;
  current_gps_location.altitude = msg->altitude;

}

void FlightData::gps_health_callback(const std_msgs::UInt8::ConstPtr& msg)
{
   gps_health = msg->data;
}

void FlightData::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{

}

void FlightData::flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
   flight_status = msg->data;
}
  // return value of RC Channels
void FlightData::lightbridge_callback(const sensor_msgs::Joy::ConstPtr& joy)
{
    lightbridge.roll_channel = joy->axes[0];
    lightbridge.pitch_channel = joy->axes[1];
    lightbridge.yaw_channel = joy->axes[2];
    lightbridge.throttle_channel = joy->axes[3];
    lightbridge.mode_switch = joy->axes[4];
    lightbridge.landing_gear_switch = joy->axes[5];

}

geometry_msgs::Vector3 FlightData::attitudeEuler(geometry_msgs::Quaternion attitude_quat)
{
    geometry_msgs::Vector3 eulerValues;

    tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(attitude_quat.x,attitude_quat.y,attitude_quat.z,attitude_quat.w))
    R_FLU2ENU.getRPY(eulerValues.x, eulerValues.y, eulerValues.z);

    return attitudeEuler;

}

void FlightData::checkLightbridgeControlMode()
{
 
  if (lightbridge.mode_switch == lightbridge.ModeToggle.MODE_F)
  {
      ROS_INFO_ONCE("Mode switch is in F mode");
  }
   

}