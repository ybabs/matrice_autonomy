#include "mission_control.h"

MissionControl::MissionControl()
{
    // Start Basic Services
    ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
    drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
    query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");

}

bool MissionControl::CheckM100()
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

void MissionControl::Takeoff()
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

void MissionControl::Land()
{
   dji_sdk::DroneTaskControl droneTaskControl;

    droneTaskControl.request.task = dji_sdk::DroneTaskControl::Request::TASK_LAND;

    drone_task_service.call(droneTaskControl);

    if(!droneTaskControl.response.result)
    {
        ROS_ERROR("Drone failed to land");
    }

}

void MissionControl::ObtainControl()
{
    dji_sdk::SDKControlAuthority authority;
    authority.request.control_enable = 1;
    ctrl_authority_service.call(authority);
    ROS_INFO("Program has obtained control");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_matrice");

    ros::spin();
    return 0;

}