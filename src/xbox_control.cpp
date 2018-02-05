#include "matrice_autonomy/xbox_control.h"

XboxControl::XboxControl():
linear(1),
angular(2)
{
    nh.param("Linear_Axis", linear, linear);
    nh.param("Angular_Axis", angular, angular);
    nh.param("Linear_Scale", linear_scale, linear_scale);
    nh.param("Angular_Scale", angular_scale, angular_scale);

    takeoff_pub = nh.advertise<std_msgs::String>("Takeoff", 1);
    land_pub = nh.advertise<std_msgs::String>("Landing", 1);

    joy_sub = nh.subscribe("Controller", 10, &XboxControl::ControlCallback, this);
    vel_pub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_generic", 10);

   // Start Basic Services
    ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
    drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
    query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");

}

void XboxControl::ControlCallback(const sensor_msgs::JoyConstPtr& joy)
{

    sensor_msgs::Joy controlVelYawRate;
     uint8_t flag = (DJISDK::VERTICAL_VELOCITY   |
                DJISDK::HORIZONTAL_VELOCITY |
                DJISDK::YAW_RATE            |
                DJISDK::HORIZONTAL_GROUND   |
                DJISDK::STABLE_ENABLE);

     controlVelYawRate.axes.push_back(joy->axes[0]);
    controlVelYawRate.axes.push_back(joy->axes[1]);
    controlVelYawRate.axes.push_back(joy->axes[2]);
    controlVelYawRate.axes.push_back(joy->axes[3]);
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


int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_matrice");
    XboxControl xboxControl;

    xboxControl.ObtainControl();
    xboxControl.Takeoff();

    ros::spin();
    return 0;

}