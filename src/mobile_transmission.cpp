
#include "matrice_autonomy/mobile_transmission.h"


MobileTransmission::MobileTransmission()
{
   mobile_data_service = mnh.serviceClient<dji_sdk::SendMobileData>("dji_sdk/send_data_to_mobile");

   MobileDataSubscriber = mnh.subscribe<dji_sdk::MobileData>("dji_sdk/from_mobile_data", 10, &MobileTransmission::MobileDataSubscriberCallback, this);

}


void MobileTransmission::SendToMobile(AckSendToMobile ackSendToMobile)
{
    dji_sdk::SendMobileData mobile_data;
    mobile_data.request.data.resize(sizeof(ackSendToMobile));
    memcpy(&mobile_data.request.data[0], (uint8_t *)(&ackSendToMobile), sizeof(ackSendToMobile) );
    mobile_data_service.call(mobile_data);


}


void MobileTransmission::SendText(unsigned char* dataToMobile)
{
    dji_sdk::SendMobileData mobile_data;
    
    mobile_data.request.data.resize (10);
    memcpy(&mobile_data.request.data[0], dataToMobile, 10);
    bool resp = mobile_data.response.result;

    ROS_INFO( "Mobile Data Response %i", resp);

    mobile_data_service.call(mobile_data);


}




void MobileTransmission::MobileDataSubscriberCallback(const dji_sdk::MobileData::ConstPtr& from_mobile_data)
{
   mobile_data = *from_mobile_data;

   unsigned char data_to_mobile[10] = {0};

   data_to_mobile[0] = 1;
   data_to_mobile[1] = 0;
   data_to_mobile[2] = 2;
   data_to_mobile[3] = 3;
   data_to_mobile[4] = 4;
   data_to_mobile[5] = 5;
   data_to_mobile[6] = 5;
   data_to_mobile[7] = 6;
   data_to_mobile[8] = 7;
   data_to_mobile[9] = 8;

   
   SendText(data_to_mobile);

} 


