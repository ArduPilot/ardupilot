#include "AP_XRCE_Client.h"

AP_HAL::Semaphore* AP_XRCE_Client::get_clientsemaphore()
{
    return &(AP::ahrs().get_semaphore());
}

bool AP_XRCE_Client::init()
{
    
    _csem = AP_XRCE_Client::get_clientsemaphore();
    if (!_csem){
        hal.console->printf("No semaphore");
        return false;
    }
 
    if (this->use_serial) {
        this->fd=open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NONBLOCK);
        if(!uxr_init_serial_transport(&this->serial_transport,this->fd,this->relativeSerialAgentAddr,this->relativeSerialClientAddr)) {
            return false;
        }
        uxr_init_session(&this->session, &this->serial_transport.comm, 0xAAAABBBB);
    }
    else {
        if(!uxr_init_udp_transport(&this->udp_transport,UXR_IPv4,ip,port)) {
            return false;
        }
        uxr_init_session(&this->session, &this->udp_transport.comm, 0xAAAABBBB);
    }

    if (!uxr_create_session(&this->session)) {
        return false;
    }
    
    if (this->use_serial) {
        this->reliable_in=uxr_create_input_reliable_stream(&this->session,this->input_reliable_stream,BUFFER_SIZE_SERIAL,STREAM_HISTORY);
        this->reliable_out=uxr_create_output_reliable_stream(&this->session,this->output_reliable_stream,BUFFER_SIZE_SERIAL,STREAM_HISTORY);
    }
    else {
        this->reliable_in=uxr_create_input_reliable_stream(&this->session,this->input_reliable_stream,BUFFER_SIZE_UDP,STREAM_HISTORY);
        this->reliable_out=uxr_create_output_reliable_stream(&this->session,this->output_reliable_stream,BUFFER_SIZE_UDP,STREAM_HISTORY);
    }

    return true;
}
bool AP_XRCE_Client::create()
{

    if(_csem->take(5)){
        this->participant_id=uxr_object_id(0x01,UXR_PARTICIPANT_ID);
        const char* participant_xml = "<dds>"
                                        "<participant>"
                                            "<rtps>"
                                                "<name>AP_XRCE_Client</name>"
                                            "</rtps>"
                                        "</participant>"
                                      "</dds>";
        this->participant_req=uxr_buffer_create_participant_xml(&this->session,this->reliable_out,this->participant_id,0,participant_xml,UXR_REPLACE);

        this->topic_id=uxr_object_id(0x01,UXR_TOPIC_ID);
        const char* topic_xml = "<dds>"
                                    "<topic>"
                                            "<name>AP_INSTopic</name>"
                                            "<dataType>AP_INS</dataType>"
                                    "</topic>"
                                "</dds>";
        this->topic_req=uxr_buffer_create_topic_xml(&this->session,this->reliable_out,this->topic_id,this->participant_id,topic_xml,UXR_REPLACE);
    
        this->pub_id=uxr_object_id(0x01,UXR_PUBLISHER_ID);
        const char* pub_xml = "";
        this->pub_req = uxr_buffer_create_publisher_xml(&this->session,this->reliable_out,this->pub_id,this->participant_id,pub_xml,UXR_REPLACE);

        this->dwriter_id = uxr_object_id(0x01,UXR_DATAWRITER_ID);
        const char* dwriter_xml = "<dds>"
                                    "<data_writer>"
                                        "<topic>"
                                            "<kind>NO_KEY</kind>"
                                            "<name>AP_INSTopic</name>"
                                            "<dataType>AP_INS</dataType>"
                                        "</topic>"
                                    "</data_writer>"
                                "</dds>";
        this->dwriter_req = uxr_buffer_create_datawriter_xml(&this->session,this->reliable_out,this->dwriter_id,this->pub_id,dwriter_xml,UXR_REPLACE);

        uint16_t requests[4] = {this->participant_req,this->topic_req,this->pub_req,this->dwriter_req};
    
        if(!uxr_run_session_until_all_status(&this->session,1000,requests,this->status,4)){
            return false;
        }
        _csem->give();
        return true;
    }
    return false;
}

void AP_XRCE_Client::write()
{
    if(_csem->take(5)){
        if(this->connected)
        {
            ucdrBuffer ub;
            uint32_t topic_size = AP_INS_size_of_topic(&this->ins_topic,0);
            uxr_prepare_output_stream(&this->session,this->reliable_out,this->dwriter_id,&ub,topic_size);
            AP_INS_serialize_topic(&ub,&this->ins_topic);
            _csem->give();
        }
    }
}

void AP_XRCE_Client::updateINSTopic(AP_InertialSensor &ins)
{
    if(_csem->take(5)){
        this->ins_topic.accel_count=ins.get_accel_count();
        this->ins_topic.gyro_count=ins.get_gyro_count();
        this->connected = uxr_run_session_time(&this->session,1000);
        _csem->give();
    }
}