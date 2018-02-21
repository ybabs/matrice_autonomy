#include "matrice_autonomy/flight_data.h"


FlightData::FlightData()
{
   attitude_sub = fnh.subscribe("dji_sdk/attitude", 10, &FlightData::attitude_callback, this);
   gps_sub = fnh.subscribe("dji_sdk/gps_position", 10, &FlightData::gps_callback, this);
   flightStatus_sub = fnh.subscribe("dji_sdk/flight_status", 10, &FlightData::flight_status_callback, this);
   gpsHealth_sub =fnh.subscribe("dji_sdk/gps_health", 10, &FlightData::gps_health_callback, this);
   imu_sub = fnh.subscribe("dji_sdk/imu", 10, &FlightData::imu_callback, this);
   batteryState_sub = fnh.subscribe("dji_sdk/battery_state", 10, &FlightData::batteryState_callback, this);
   lightbridge_sub = fnh.subscribe("/dji_sdk/rc", 10, &FlightData::lightbridge_callback, this);
}

void FlightData::attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  attitude_data = msg->quaternion;
  ROS_INFO_ONCE("Quaternion Data %f, %f, %f, %f", attitude_data.x, attitude_data.y, attitude_data.z, attitude_data.w);
    
}

sensor_msgs::NavSatFix FlightData::GetGPSPosition()
{

    return current_gps_location;
  
}

void FlightData::batteryState_callback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
  batteryLeft = msg->percentage;
  ROS_INFO("Battery Life: %d %%", batteryLeft);
}

void FlightData::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  current_gps_location.latitude = msg->latitude;
  current_gps_location.longitude = msg->longitude;
  current_gps_location.altitude = msg->altitude;

  ROS_INFO("GPS Location %f , %f , %f",  current_gps_location.latitude,  current_gps_location.longitude, current_gps_location.altitude);

}

void FlightData::gps_health_callback(const std_msgs::UInt8::ConstPtr& msg)
{
   gps_health = msg->data;
   ROS_INFO_ONCE ("GPS Health: %i", gps_health);
}

void FlightData::imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{

// Retrieve orientation data in quaternion
    imu_data.orientation.x = msg->orientation.x;
    imu_data.orientation.y = msg->orientation.y;
    imu_data.orientation.z = msg->orientation.z;
    imu_data.orientation.w = msg->orientation.w;

// Angular velocity 
    imu_data.angular_velocity.x = msg->angular_velocity.x;
    imu_data.angular_velocity.y = msg->angular_velocity.y;
    imu_data.angular_velocity.z = msg->angular_velocity.z;

    imu_data.linear_acceleration.x = msg->linear_acceleration.x;
    imu_data.linear_acceleration.y = msg->linear_acceleration.y;
    imu_data.linear_acceleration.z = msg->linear_acceleration.z;

}

void FlightData::flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
   flight_status = msg->data;
   ROS_INFO_ONCE("FLight Status: %i", flight_status);
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

    tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(attitude_quat.x,attitude_quat.y,attitude_quat.z,attitude_quat.w));
    R_FLU2ENU.getRPY(eulerValues.x, eulerValues.y, eulerValues.z);

    ROS_INFO_ONCE("Euler angles: x = %f y = %f z = %f ", eulerValues.x, eulerValues.y, eulerValues.z);
    return eulerValues;

}

void FlightData::checkLightbridgeControlMode()
{
 
  if (lightbridge.mode_switch == Lightbridge::MODE_F)
  {
      ROS_INFO_ONCE("Mode switch is in F mode");
  }
   

}

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "flight_collect");
//     FlightData flightdata;

//     ros::spin();
//     return 0;

// }