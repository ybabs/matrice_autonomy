#include "mission_control.h"

using namespace DJI::OSDK;



MissionControl::MissionControl()
{
    // Start Basic Services
    ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
    drone_task_service     = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
    query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
    waypoint_upload_service = nh.serviceClient<dji_sdk::MissionWpUpload>("dji_sdk/mission_waypoint_upload");
    waypoint_action_service  = nh.serviceClient<dji_sdk::MissionWpAction>("dji_sdk/mission_waypoint_action");
    drone_activation_service = nh.serviceClient<dji_sdk::Activation>("dji_sdk/activation");

}

void MissionControl::Activate()
{
   dji_sdk::Activation activation;
   drone_activation_service.call(activation);
   ROS_INFO("Activation successful");

   if(!activation.response.result)
   {
       ROS_ERROR("Drone app not activated. Please check your app key");
   }

   
}

void::MissionControl::SetWayPointDefaults(WayPointSettings* waypoint)
{
    waypoint->damping = 0;
    waypoint->yaw = 0;
    waypoint->gimbalPitch = 0;
    waypoint->turnMode = 0;
    waypoint->hasAction =  0;
    waypoint->actionTimeLimit = 100; // in seconds????
    waypoint->actionNumber = 0;
    waypoint->actionRepeat = 0;

    for (int i = 0; i < 16; i ++)
    {
        waypoint->commandList[i] = 0;
        waypoint->commandParameter[i] = 0; 
    }

}

void MissionControl::SetWayPointInitDefaults(dji_sdk::MissionWaypointTask& waypointTask)
{
    waypointTask.velocity_range = 10;
    waypointTask.idle_velocity = 2;
    waypointTask.action_on_finish = dji_sdk::MissionWaypointTask::FINISH_NO_ACTION;
    waypointTask.mission_exec_times = 1;
    waypointTask.yaw_mode = 1;
    waypointTask.trace_mode = dji_sdk::MissionWaypointTask::TRACE_POINT;
    waypointTask.action_on_rc_lost = dji_sdk::MissionWaypointTask::ACTION_AUTO;
    waypointTask.gimbal_pitch_mode = dji_sdk::MissionWaypointTask::GIMBAL_PITCH_FREE;

}

std::vector<DJI::OSDK::WayPointSettings> MissionControl::GeneratePolygon(WayPointSettings* startData, float64_t increment, int numWaypoints)
{
    // Circular waypoint Polygon could be drawn here as well

    // vector to stoe waypoints in 
    std::vector<DJI::OSDK::WayPointSettings> waypointList;

    // Create equal number of angles
    float64_t angle = 2 * PI / numWaypoints;
    
    //first waypoint
    startData->index = 0;
    waypointList.push_back(*startData);

    // Generate points here. Integrate the Aviation formulary method here later on
    for(int i = 0; i < numWaypoints; i++)
    {
        WayPointSettings current_waypoint;

        WayPointSettings* previous_waypoint = &waypointList[i - 1];
        SetWayPointDefaults(&current_waypoint);

        current_waypoint.index = i;
        current_waypoint.latitude = (previous_waypoint->latitude + (increment * cos(i * angle)));
        current_waypoint.longitude = (previous_waypoint->longitude + (increment * sin(i * angle)));
        current_waypoint.altitude = (previous_waypoint->altitude + 1);
        
        waypointList.push_back(current_waypoint);           
    
    }

    startData->index = numWaypoints;
    waypointList.push_back(*startData);

    return waypointList;
    
}    

void MissionControl::UploadWaypoints(std::vector<DJI::OSDK::WayPointSettings>& waypointList, int responseTimeout, dji_sdk::MissionWaypointTask& waypointTask)
{
    dji_sdk::MissionWaypoint waypoint;

    for( std::vector<WayPointSettings>::iterator wp = waypointList.begin(); wp != waypointList.end(); ++wp)
    {
        ROS_INFO("Waypoint created at Lat: %f , Lon: %f , Alt: %f \n", wp->latitude, wp->longitude, wp->altitude );

        waypoint.latitude = wp->latitude;
        waypoint.longitude = wp->longitude;
        waypoint.altitude = wp->altitude;
        waypoint.damping_distance = 0;
        waypoint.target_yaw = 0;
        waypoint.turn_mode = 0;
        waypoint.target_gimbal_pitch = 0;
        waypoint.has_action = 0;

        waypointTask.mission_waypoint.push_back(waypoint);
    }
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

void MissionControl::TakeOff()
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

void MissionControl::GoHome()
{
    dji_sdk::DroneTaskControl droneTaskControl;

    droneTaskControl.request.task = dji_sdk::DroneTaskControl::Request::TASK_GOHOME;

    drone_task_service.call(droneTaskControl);
    ROS_INFO("DRONE Going Home!");

    if(!droneTaskControl.response.result)
    {
        ROS_ERROR("Drone failed to go home");
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