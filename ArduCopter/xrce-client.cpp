#include "Copter.h"
#include "Copter.h"

void Copter::init_client()
{
    if(client.init()){
        if(client.create()){
            hal.scheduler->register_io_process(FUNCTOR_BIND(&client,&AP_XRCE_Client::write,void));
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,"Client: Initialization passed");
        }
        else{
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,"Client: Creation Requests failed");
        }
    }
    else{
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"Client: Initialization failed");
    }

}

void Copter::update_topics()
{
    client.updateINSTopic(copter.ins);
}
