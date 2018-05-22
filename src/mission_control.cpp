#include "matrice_autonomy/mission_control.h"

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
    mobile_data_service = nh.serviceClient<dji_sdk::SendMobileData>("dji_sdk/send_data_to_mobile");

    mobile_data_subscriber = nh.subscribe<dji_sdk::MobileData>("dji_sdk/from_mobile_data", 10, &MissionControl::MobileDataSubscriberCallback, this);
    gps_sub = nh.subscribe("dji_sdk/gps_position", 10, &MissionControl::gps_callback, this);

    data_to_mobile[10] = {0};
    waypoint_index = 1;

     latitude = 0;
     longitude = 0;
     altitude = 0;
     speed = 0;

     latitude_array[8] = {0};
     longitude_array[8] = {0};
     altitude_array[4] = {0};
     orientation = 0;
     speed_array[4] = {0};
     missionEnd = 0;


    
}

void MissionControl::MobileDataSubscriberCallback(const dji_sdk::MobileData::ConstPtr& mobile_data)
{
    data_from_mobile = *mobile_data;
    MissionControl missionControlManager;

    unsigned char data;
    memcpy(&data, &data_from_mobile.data[0], 10);

    unsigned char flight_data[28] = {0};

    ROS_INFO_STREAM ("Receieved from mobile: " <<std::hex << data_from_mobile );

    unsigned char CMD = data_from_mobile.data[0];

      switch(CMD)
    {

        case 0x01:
        {
           TakeOff();
             break;

        }

       /* Abort Button pressed on the app runs here as well. 
       Default behaviour when user presses abort is to go back home. */
        case 0x02:
        {
            GoHome();
            break;
            
        }

        case 0x03:
        {
            Land();
            ROS_INFO("Aircraft Landing");
            break;
        }

        case 0x4d:
        {
            ROS_INFO("Flight Parameters received");

            for(int i = 0; i < sizeof(speed_array); i++)
            {
               speed_array[i] = data_from_mobile.data[i + 1];
            }
          
            orientation = data_from_mobile.data[5];

            missionEnd = data_from_mobile.data[6];
 
            std::reverse(std::begin(speed_array), std::end(speed_array));
           
            std::memcpy(&speed, speed_array, sizeof(float));
            // Set cruising speed of drone (between 2 and 15m/s)
            waypointTask.idle_velocity = speed;

             switch(missionEnd)
            {
                case 1:  // NO ACTION
                waypointTask.action_on_finish = dji_sdk::MissionWaypointTask::FINISH_NO_ACTION;
                std::cout << "NOTHING ACTION" << std::endl;
                break;

                case 2: // WAYPOINT // Drone will go  to first waypoint
                 waypointTask.action_on_finish = dji_sdk::MissionWaypointTask::FINISH_RETURN_TO_POINT;
                 std::cout << "FIRST WAYPOINT ACTION" << std::endl;

                break;

                case 3:  // RETURN HOME
                 waypointTask.action_on_finish = dji_sdk::MissionWaypointTask::FINISH_RETURN_TO_HOME;
                 std::cout << "RETURN HOME ACTION" << std::endl;
                break;

                case 4: // AUTOLAND
                waypointTask.action_on_finish = dji_sdk::MissionWaypointTask::FINISH_AUTO_LANDING;
                std::cout << "LAND ACTION" << std::endl;
                break;

                default:
                break;
            }

            switch(orientation)
            {
                case 1: // Lock as Initial value
                waypointTask.yaw_mode = dji_sdk::MissionWaypointTask::YAW_MODE_LOCK;
                std::cout << "YAW MODE" << std::endl;
                break;

                case 2: // Auto Mode.. Point to next waypoint
                waypointTask.yaw_mode = dji_sdk::MissionWaypointTask::YAW_MODE_AUTO;
                std::cout << "NEXT MODE" << std::endl;
                break;

                case 3: // Use Waypoint's Yaw
                 waypointTask.yaw_mode = dji_sdk::MissionWaypointTask::YAW_MODE_WAYPOINT;
                 std::cout << "WAYPOINT MODE" << std::endl;
                break;

                case 4: // Use RC to control Yaw at waypoint
                std::cout << "RC MODE" << std::endl;
                 waypointTask.yaw_mode = dji_sdk::MissionWaypointTask::YAW_MODE_RC;

                break;

                default:
                break;

            }

            std::cout << std::fixed << speed << std::endl;
            std::cout << static_cast<unsigned>(orientation) << std::endl;
            std::cout << static_cast<unsigned>(missionEnd) << std::endl;
            break;

        }

        case 0x2f:
        { 
            ROS_INFO("Waypoints Recieved");


            WayPointSettings current_waypoint;
            SetWayPointDefaults(&current_waypoint);


            for(int i = 0; i < sizeof(latitude_array); i ++)
            {
                 latitude_array [i] = data_from_mobile.data[i + 1];  
          
            }

            for(int i = 0; i < sizeof(longitude_array); i ++)
            {
                longitude_array [i] = data_from_mobile.data[i + 9] ;
            }   

            for(int i = 0; i < sizeof(altitude_array); i ++)
            {
                altitude_array [i] = data_from_mobile.data[i + 17] ;
            }
   


            std::reverse(std::begin(latitude_array), std::end(latitude_array));
            std::reverse(std::begin(longitude_array), std::end(longitude_array));
            std::reverse(std::begin(altitude_array), std::end(altitude_array));

            std::memcpy(&latitude, latitude_array, sizeof (double));
            std::memcpy(&longitude, longitude_array, sizeof (double));
            std::memcpy(&altitude, altitude_array, sizeof(float));
           
            
            current_waypoint.latitude = latitude;
            current_waypoint.longitude = longitude;
            current_waypoint.altitude = altitude;
            
            current_waypoint.index = waypoint_index;
            SetWayPointDefaults(&current_waypoint);
            flightWaypointList.push_back(current_waypoint);

            waypoint_index++;
            

           

             std::cout <<std::fixed << latitude <<std::endl;
             std::cout <<std::fixed << longitude << std::endl;
             std::cout <<std::fixed << altitude << std::endl;

          
            break;
        }

      // Start Mission when Start Button is pressed on Drone
        case 0x1A:
        {
            RunWaypointMission(responseTimeout);
        }

        case 0x3F:
        {
            // Clear the vector storing waypoints.
            flightWaypointList.clear();
            waypointTask.mission_waypoint.empty();
            
            ROS_INFO("Waypoints Cleared");
        }


       default:
       {
           break;

       }
        

    }



}


void MissionControl::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  gps_pos.latitude = msg->latitude;
  gps_pos.longitude = msg->longitude;
  gps_pos.altitude = msg->altitude;

  ROS_INFO_ONCE("GPS Location %f , %f , %f",  gps_pos.latitude,  gps_pos.longitude, gps_pos.altitude);

}

void MissionControl::SetGPSPosition(sensor_msgs::NavSatFix set_gps_pos)
{
    gps_pos = set_gps_pos;
  
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

   else
   {
       ROS_INFO("Activation Successful");
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

    for (int i = 0; i < 16; ++i)
    {
        waypoint->commandList[i] = 0;
        waypoint->commandParameter[i] = 0; 
    }

}

void MissionControl::SetWayPointInitDefaults(dji_sdk::MissionWaypointTask& waypointTask)
{
    // Heading and Yaw mode are set by the Application. EVerything else is set to default here.
    waypointTask.velocity_range = 15;
    //waypointTask.idle_velocity = 5;
   // waypointTask.action_on_finish = dji_sdk::MissionWaypointTask::FINISH_NO_ACTION;
    waypointTask.mission_exec_times = 1;
    //waypointTask.yaw_mode = dji_sdk::MissionWaypointTask::YAW_MODE_AUTO;
    waypointTask.trace_mode = dji_sdk::MissionWaypointTask::TRACE_POINT;
    waypointTask.action_on_rc_lost = dji_sdk::MissionWaypointTask::ACTION_AUTO;
    waypointTask.gimbal_pitch_mode = dji_sdk::MissionWaypointTask::GIMBAL_PITCH_FREE;

}


void MissionControl::UploadWaypoints(std::vector<DJI::OSDK::WayPointSettings>& waypointList, int responseTimeout, dji_sdk::MissionWaypointTask& waypointTask)
{
    dji_sdk::MissionWaypoint waypoint;

    for( std::vector<WayPointSettings>::iterator wp = waypointList.begin(); wp != waypointList.end(); ++wp)
    {
        ROS_INFO("Waypoint created at Lat Upload: %f , Lon: %f , Alt: %f \n", wp->latitude, wp->longitude, wp->altitude );

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

bool MissionControl::RunWaypointMission(int responseTimeout)
{

    ros::spinOnce();
    SetWayPointInitDefaults(waypointTask);

  ROS_INFO("Number of Waypoints created: %lu", flightWaypointList.size());
    /// Upload Waypoints;
    ROS_INFO("Uploading Waypoints ... \n");
    UploadWaypoints(flightWaypointList, responseTimeout, waypointTask);

    // initialise Mission:
    ROS_INFO("Initialising Mission ... \n");
    if(InitWayPointMission(waypointTask) != true)
    {
       return false;
    }

    // Start Waypoint Mission
    MissionAction(DJI_MISSION_TYPE::WAYPOINT, MISSION_ACTION::START);

    return true;


}

void MissionControl:: MissionAction(DJI::OSDK::DJI_MISSION_TYPE type, DJI::OSDK::MISSION_ACTION action)
{
    dji_sdk::MissionWpAction waypointMissionAction;

    if (type == DJI::OSDK::WAYPOINT)
    {
        waypointMissionAction.request.action = action;
        waypoint_action_service.call(waypointMissionAction);

        if (!waypointMissionAction.response.result)
        {
            ROS_WARN( "Acknowledgment Info Set = %i id = %i", waypointMissionAction.response.cmd_set, waypointMissionAction.response.cmd_id);

             ROS_INFO("ACK.data: %i", waypointMissionAction.response.ack_data);

             ROS_ERROR("Mission start command not sent");

        }

        else
        {
            ROS_INFO("Action set");
        }

    }

    else
    {
        ROS_ERROR("Mission Type not supported");
    }

}


bool MissionControl::InitWayPointMission(dji_sdk::MissionWaypointTask& waypointTask)
{

   dji_sdk::MissionWpUpload uploadWaypointMission;
   uploadWaypointMission.request.waypoint_task = waypointTask;

   waypoint_upload_service.call(uploadWaypointMission);
   if(!uploadWaypointMission.response.result)
   {
       ROS_WARN ( "Acknowledgment Info Set = %i id = %i", uploadWaypointMission.response.cmd_set, uploadWaypointMission.response.cmd_id);

       ROS_INFO("ACK.data: %i", uploadWaypointMission.response.ack_data);

        ROS_ERROR("Couldn't upload waypoints");

        return false;

   }

   else
   {
        ROS_INFO("Waypoints uploaded successfully");
        return true;
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
    if(authority.response.result)
    {
    ROS_INFO("Program has obtained control");
    }

    else
    {
        if(authority.response.ack_data == 3 && authority.response.cmd_set == 1 && authority.response.cmd_id == 0)
        {
            // Make this recursive?
            // Possibly call obtainControl again
            ROS_INFO("Call control Authority again");
        }

        else
        {
            ROS_ERROR("Failed to obtain control authority");
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Mission Control");
     
    FlightData flightData;
         
    MissionControl missionControl;

    // ros::spinOnce();

    missionControl.SetGPSPosition(flightData.GetGPSPosition());

    missionControl.Activate();

    missionControl.ObtainControl();
       
    //uint8_t waypointSides = 6;
   // int responseTimeout = 1;
    //missionControl.RunWaypointMission(waypointSides, responseTimeout);
    
    ros::spin();

   
    return 0;

}