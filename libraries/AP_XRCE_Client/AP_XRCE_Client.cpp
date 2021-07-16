#include "AP_XRCE_Client.h"

#if AP_XRCE_ENABLED

AP_HAL::UARTDriver *xrce_port;

AP_HAL::Semaphore *AP_XRCE_Client::get_clientsemaphore()
{
    return &(AP::ahrs().get_semaphore());
}

bool AP_XRCE_Client::init()
{
    xrce_port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_ROS2, 0);
    if (xrce_port == nullptr) {
        return false;
    }

    _csem = AP_XRCE_Client::get_clientsemaphore();
    if (!_csem){
        hal.console->printf("No semaphore");
        return false;
    }

    if (!uxr_init_serial_transport(&serial_transport,0,relativeSerialAgentAddr,relativeSerialClientAddr)) {
        return false;
    }
    uxr_init_session(&session, &serial_transport.comm, 0xAAAABBBB);

    if (!uxr_create_session(&session)) {
        return false;
    }
    
    reliable_in=uxr_create_input_reliable_stream(&session,input_reliable_stream,BUFFER_SIZE_SERIAL,STREAM_HISTORY);
    reliable_out=uxr_create_output_reliable_stream(&session,output_reliable_stream,BUFFER_SIZE_SERIAL,STREAM_HISTORY);
    
    return true;
}
bool AP_XRCE_Client::create()
{
    WITH_SEMAPHORE(_csem);
    participant_id=uxr_object_id(0x01,UXR_PARTICIPANT_ID);
    const char* participant_xml = "<dds>"
                                    "<participant>"
                                        "<rtps>"
                                        "<name>AP_XRCE_Client</name>"
                                        "</rtps>"
                                    "</participant>"
                                "</dds>";
    participant_req=uxr_buffer_create_participant_xml(&session,reliable_out,participant_id,0,participant_xml,UXR_REPLACE);

    topic_id=uxr_object_id(0x01,UXR_TOPIC_ID);
    const char* topic_xml = "<dds>"
                                "<topic>"
                                    "<name>AP_INSTopic</name>"
                                    "<dataType>AP_INS</dataType>"
                                "</topic>"
                            "</dds>";
    topic_req=uxr_buffer_create_topic_xml(&session,reliable_out,topic_id,participant_id,topic_xml,UXR_REPLACE);
    
    pub_id=uxr_object_id(0x01,UXR_PUBLISHER_ID);
    const char* pub_xml = "";
    pub_req = uxr_buffer_create_publisher_xml(&session,reliable_out,pub_id,participant_id,pub_xml,UXR_REPLACE);

    dwriter_id = uxr_object_id(0x01,UXR_DATAWRITER_ID);
    const char* dwriter_xml = "<dds>"
                                "<data_writer>"
                                    "<topic>"
                                        "<kind>NO_KEY</kind>"
                                        "<name>AP_INSTopic</name>"
                                        "<dataType>AP_INS</dataType>"
                                    "</topic>"
                                "</data_writer>"
                            "</dds>";
    dwriter_req = uxr_buffer_create_datawriter_xml(&session,reliable_out,dwriter_id,pub_id,dwriter_xml,UXR_REPLACE);

    uint16_t requests[4] = {participant_req,topic_req,pub_req,dwriter_req};
    
    if (!uxr_run_session_until_all_status(&session,1000,requests,status,4)) {
        return false;
    }

    return true;
}

void AP_XRCE_Client::write()
{
    WITH_SEMAPHORE(_csem);
    if(connected)
    {
        ucdrBuffer ub;
        uint32_t topic_size = AP_INS_size_of_topic(&ins_topic,0);
        uxr_prepare_output_stream(&session,reliable_out,dwriter_id,&ub,topic_size);
        AP_INS_serialize_topic(&ub,&ins_topic);
    }
    
}

void AP_XRCE_Client::updateINSTopic(AP_InertialSensor &ins)
{
    if (xrce_port == nullptr) {
        return;
    }
    WITH_SEMAPHORE(_csem);
    ins_topic.accel_count=ins.get_accel_count();
    ins_topic.gyro_count=ins.get_gyro_count();
    connected = uxr_run_session_time(&session,1000);
        
}

/*
  implement C functions for serial transport
 */
extern "C" {
#include <uxr/client/profile/transport/serial/serial_transport_platform.h>
}

bool uxr_init_serial_platform(void* args, int fd, uint8_t remote_addr, uint8_t local_addr)
{
    return true;
}

bool uxr_close_serial_platform(void* args)
{
    return true;
}

size_t uxr_write_serial_data_platform(void* args, const uint8_t* buf, size_t len, uint8_t* errcode)
{
    if (xrce_port == nullptr) {
        *errcode = 1;
        return 0;
    }
    size_t bytes_written = xrce_port->write(buf, (size_t)len);
    if (bytes_written == 0) {
        *errcode = 1;
        return 0;
    }
    *errcode = 0;
    return bytes_written;
}

size_t uxr_read_serial_data_platform(void* args, uint8_t* buf, size_t len, int timeout, uint8_t* errcode)
{
    if (xrce_port == nullptr) {
        *errcode = 1;
        return 0;
    }
    size_t bytes_read = xrce_port->read(buf, (size_t)len);
    if (bytes_read <= 0) {
        *errcode = 1;
        return 0;
    }
    *errcode = 0;
    return bytes_read;
}

#endif // AP_XRCE_ENABLED


