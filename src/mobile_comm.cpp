#include "matrice_autonomy/mobile_comm.h"


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

    unsigned char data;
    memcpy(&data, &data_from_mobile.data[0], 10);

    ROS_INFO_STREAM ("Receieved from mobile: " <<std::hex << data_from_mobile );
    
   unsigned char data_to_mobile [10] = {0};

   data_to_mobile[0] = 0x02;
   data_to_mobile[1] = 0x05;

  //SendDataToMobile(data_to_mobile);

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





