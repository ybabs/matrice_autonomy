#include "matrice_autonomy/xbox_control.h"

XboxControl::XboxControl()
{
    // Set axes for roll, pitch, yaw
    nh.param("roll_axis", axes.roll, 0);
    nh.param("pitch_axis", axes.pitch, 1);
    nh.param("throttle_axes", axes.throttle, 3);

    // Direction paramters
    nh.param("roll_axis_direction", axes.roll_direction, -1);
    nh.param("pitch_axis_direction", axes.pitch_direction, 1);
    nh.param("throttle_axes_direction", axes.throttle_direction, 1);
    
    //define  maximum performance parameters.. Gimped for testing purposes
    /* Real performance according to DJI Datasheet:

    Max Angular Pitch Velocity = 5.23598 rad/s
    Max Angular Yaw Velocity = 2.61799 rad/s
    Max tilt angle = 35 degress
    Max Ascent Speed = 5 m/s
    Max Descent Speed = 4 m/s
    Max Wind Resistance = 10 m/s
    Max Speed 22/17 ms (ATTI/GPS) with no payload and no wind
    
    */
    
    nh.param("max_xy_velocity", maxValues.velocity_xy, 1.0);
    nh.param("max_roll", maxValues.roll, 20 * PI / 180.0);
    nh.param("max_pitch", maxValues.pitch, 20 * PI / 180.0);
    nh.param("max_yaw_rate", maxValues.yaw_rate, 20 * PI / 180.0);

    nh.param("yaw_delta", yaw_velocity_delta, 0.05); // rad/s

    /*Define buttons here later on */
    takeoff_pub = nh.advertise<std_msgs::String>("Takeoff", 1);
    land_pub = nh.advertise<std_msgs::String>("Landing", 1);

    joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &XboxControl::ControlCallback, this);
    vel_pub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_generic", 10);
    rc_sub = nh.subscribe("/dji_sdk/rc", 100, &XboxControl::RC_Callback, this );
   // Start Basic Services
    ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
    drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
    query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");

}

void XboxControl::ControlCallback(const sensor_msgs::JoyConstPtr& joy)
{

  //  controlVelYawRate = *joy;
    sensor_msgs::Joy controlVelYawRate;


    double joy_roll = joy->axes[axes.roll] * maxValues.roll * axes.roll_direction;
    double joy_pitch = joy->axes[axes.pitch] * maxValues.pitch * axes.pitch_direction;
     // Add yaw later
    double joy_throttle = joy->axes[axes.throttle] * axes.throttle_direction;

     uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
                DJISDK::HORIZONTAL_VELOCITY |
                DJISDK::YAW_RATE            |
                DJISDK::HORIZONTAL_GROUND   |
                DJISDK::STABLE_ENABLE);

    controlVelYawRate.axes.push_back(joy_roll);
    controlVelYawRate.axes.push_back(joy_pitch);
    controlVelYawRate.axes.push_back(joy->axes[2]);
    controlVelYawRate.axes.push_back(joy_throttle);
    controlVelYawRate.axes.push_back(flag);

   vel_pub.publish(controlVelYawRate);
}

bool XboxControl::CheckM100()
{
    dji_sdk::QueryDroneVersion query;
    query_version_service.call(query);

    if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
    {
        return true;
    }

    else
    {
        return false;
    }


}

void XboxControl::Takeoff()
{
    dji_sdk::DroneTaskControl droneTaskControl;

    droneTaskControl.request.task = dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF;

    drone_task_service.call(droneTaskControl);
    ROS_INFO("DRONE TAKING OFF!");

    if(!droneTaskControl.response.result)
    {
        ROS_ERROR("Drone failed to take off");
    }


}

void XboxControl::Land()
{
   dji_sdk::DroneTaskControl droneTaskControl;

    droneTaskControl.request.task = dji_sdk::DroneTaskControl::Request::TASK_LAND;

    drone_task_service.call(droneTaskControl);

    if(!droneTaskControl.response.result)
    {
        ROS_ERROR("Drone failed to land");
    }

}

void XboxControl::ObtainControl()
{
    dji_sdk::SDKControlAuthority authority;
    authority.request.control_enable = 1;
    ctrl_authority_service.call(authority);
    ROS_INFO("Program has obtained control");
}

void XboxControl::RC_Callback(const sensor_msgs::JoyConstPtr& joy)
{
    roll_channel = joy->axes[0];
    pitch_channel = joy->axes[1];
    yaw_channel = joy->axes[2];
    throttle_channel = joy->axes[3];
    mode_switch = joy->axes[4];
    landing_gear_switch = joy->axes[5];

   // ROS_INFO("Roll = %d, Pitch = %d, Yaw = %d, Throttle = %d, Mode = %d, landing = %d", roll_channel, 
  //  pitch_channel, yaw_channel, throttle_channel, mode_switch, landing_gear_switch);
     

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_matrice");
    XboxControl xboxControl;

    xboxControl.ObtainControl();
    xboxControl.Takeoff();

    ros::spin();
    return 0;

}