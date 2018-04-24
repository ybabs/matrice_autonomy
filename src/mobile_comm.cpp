#include "matrice_autonomy/mobile_comm.h"

using namespace DJI::OSDK;



MobileComm::MobileComm()
{
    mobile_data_service = nh.serviceClient<dji_sdk::SendMobileData>("dji_sdk/send_data_to_mobile");

    mobile_data_subscriber = nh.subscribe<dji_sdk::MobileData>("dji_sdk/from_mobile_data", 10, &MobileComm::MobileDataSubscriberCallback, this);

    data_to_mobile[10] = {0};

 

   
    
}


void MobileComm::SendDataToMobile(unsigned char* data_to_mobile)
{
    dji_sdk::SendMobileData mobile_data;

    mobile_data.request.data.resize(10);
    memcpy(&mobile_data.request.data[0], data_to_mobile, 10 );

    mobile_data_service.call(mobile_data);

    bool mobile_response = mobile_data.response.result;
   

    ROS_INFO_STREAM("Sent: " <<  mobile_data.request.data[0]<< mobile_data.request.data[1]);
   
    ROS_INFO("SEND DATA TO MOBILE RESPONSE %i", mobile_response);

}

// Overloaded Function

void MobileComm::SendDataToMobile(AckReturnToMobile returnMobileAck)
{
    dji_sdk::SendMobileData mobile_data;

    mobile_data.request.data.resize(sizeof(AckReturnToMobile));
    memcpy(&mobile_data.request.data[0], (uint8_t *)(&returnMobileAck), sizeof(AckReturnToMobile));

    mobile_data_service.call(mobile_data);

    bool mobile_response = mobile_data.response.result;
      
    ROS_INFO("SEND DATA TO MOBILE RESPONSE %i", mobile_response);

}

void MobileComm::MobileDataSubscriberCallback(const dji_sdk::MobileData::ConstPtr& mobile_data)
{
    data_from_mobile = *mobile_data;
    MissionControl missionControlManager;

    unsigned char data;
    memcpy(&data, &data_from_mobile.data[0], 10);

    unsigned char flight_data[28] = {0};

    ROS_INFO_STREAM ("Receieved from mobile: " <<std::hex << data_from_mobile );

    unsigned char CMD = data_from_mobile.data[0];

    std::vector<DJI::OSDK::WayPointSettings> flightWaypointList;

    switch(CMD)
    {

        case 0x01:
        {
            missionControlManager.TakeOff();
             break;

        }

        case 0x02:
        {
            missionControlManager.GoHome();
            break;
            
        }

        case 0x03:
        {
            missionControlManager.Land();
            ROS_INFO("Aircraft Landing");
            break;
        }

        case 0x2f:
        { 
            ROS_INFO("Waypoints Recieved");
            double latitude;
            double longitude;
            float altitude;
            float speed;

            WayPointSettings current_waypoint;
            missionControlManager.SetWayPointDefaults(&current_waypoint);

            unsigned char latitude_array[8] = {0};
            unsigned char longitude_array[8] = {0};
            unsigned char altitude_array[4] = {0};
            unsigned char orientation;
            unsigned char speed_array[4] = {0};
            unsigned char missionEnd = 0;

            memcpy(&flight_data, &data_from_mobile.data, sizeof(data_from_mobile));

            for(int i = 0; i < sizeof(longitude_array); i ++)
            {
                longitude_array [i] = flight_data[i + 9] ;
            }

            for(int i = 0; i < sizeof(altitude_array); i ++)
            {
                altitude_array [i] = flight_data[i + 17] ;
            }

            orientation = flight_data[21];

            for(int i = 0; i < sizeof(speed_array); i++)
            {
                speed_array [i + 22];
            }

            missionEnd = flight_data[26];


            std::reverse(std::begin(latitude_array), std::end(latitude_array));
            std::reverse(std::begin(longitude_array), std::end(longitude_array));
            std::reverse(std::begin(altitude_array), std::end(altitude_array));
            std::reverse(std::begin(speed_array), std::end(speed_array));

            std::memcpy(&latitude, latitude_array, sizeof (double));
            std::memcpy(&longitude, longitude_array, sizeof (double));
            std::memcpy(&altitude, altitude_array, sizeof(float));
            
            current_waypoint.latitude = latitude;
            current_waypoint.longitude = longitude;
            current_waypoint.altitude = altitude;

           
            
            break;
        }


       default:
       {
           break;

       }
        

    }



}

void MobileComm:: SendText(std::string string_to_mobile)
{
    dji_sdk::SendMobileData mobile_data;
    mobile_data.request.data.resize(sizeof(string_to_mobile));

    memcpy(&mobile_data.request.data[0], &string_to_mobile, sizeof(string_to_mobile));
    mobile_data_service.call(mobile_data);

    bool mobile_string_response = mobile_data.response.result;

   // ROS_INFO("Sent: %s ",  mobile_data.request.data);
   
    ROS_INFO("SEND DATA TO MOBILE RESPONSE %i", mobile_string_response);


}





