#include <AP_HAL/AP_HAL_Boards.h>

#if AP_DDS_ENABLED

#include <AP_RTC/AP_RTC.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

#include "AP_DDS_Client.h"
#include "generated/Time.h"

AP_HAL::UARTDriver *dds_port;


const AP_Param::GroupInfo AP_DDS_Client::var_info[]= {
    //! @todo Params go here

    AP_GROUPEND
};

#include "AP_DDS_Topic_Table.h"

void AP_DDS_Client::update_topic(builtin_interfaces_msg_Time& msg)
{
    uint64_t utc_usec;
    if (!AP::rtc().get_utc_usec(utc_usec)) {
        utc_usec = AP_HAL::micros64();
    }
    msg.sec = utc_usec / 1000000ULL;
    msg.nanosec = (utc_usec % 1000000ULL) * 1000UL;

}


/*
  class constructor
 */
AP_DDS_Client::AP_DDS_Client(void)
{
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_DDS_Client::main_loop, void),
                                      "DDS",
                                      8192, AP_HAL::Scheduler::PRIORITY_IO, 1)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"DDS Client: thread create failed");
    }
}

/*
  main loop for DDS thread
 */
void AP_DDS_Client::main_loop(void)
{
    if (!init() || !create()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"DDS Client: Creation Requests failed");
        return;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO,"DDS Client: Initialization passed");
    while (true) {
        hal.scheduler->delay(1);
        update();
    }
}


bool AP_DDS_Client::init()
{
    AP_SerialManager *serial_manager = AP_SerialManager::get_singleton();
    dds_port = serial_manager->find_serial(AP_SerialManager::SerialProtocol_DDS_XRCE, 0);
    if (dds_port == nullptr) {
        return false;
    }

    // ensure we own the UART
    dds_port->begin(0);

    constexpr uint8_t fd = 0;
    constexpr uint8_t relativeSerialAgentAddr = 0;
    constexpr uint8_t relativeSerialClientAddr = 1;
    if (!uxr_init_serial_transport(&serial_transport,fd,relativeSerialAgentAddr,relativeSerialClientAddr)) {
        return false;
    }

    constexpr uint32_t uniqueClientKey = 0xAAAABBBB;
    //TODO does this need to be inside the loop to handle reconnect?
    uxr_init_session(&session, &serial_transport.comm, uniqueClientKey);
    while (!uxr_create_session(&session)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"DDS Client: Initialization waiting...");
        hal.scheduler->delay(1000);
    }

    reliable_in = uxr_create_input_reliable_stream(&session,input_reliable_stream,BUFFER_SIZE_SERIAL,STREAM_HISTORY);
    reliable_out = uxr_create_output_reliable_stream(&session,output_reliable_stream,BUFFER_SIZE_SERIAL,STREAM_HISTORY);

    GCS_SEND_TEXT(MAV_SEVERITY_INFO,"DDS Client: Init Complete");

    return true;
}

bool AP_DDS_Client::create()
{
    WITH_SEMAPHORE(csem);

    // Participant
    const uxrObjectId participant_id = {
        .id = 0x01,
        .type = UXR_PARTICIPANT_ID
    };
    const char* participant_ref = "participant_profile";
    const auto participant_req_id = uxr_buffer_create_participant_ref(&session, reliable_out, participant_id,0,participant_ref,UXR_REPLACE);

    // Topic
    const uxrObjectId topic_id = {
        .id = topics[0].topic_id,
        .type = UXR_TOPIC_ID
    };
    const char* topic_ref = topics[0].topic_profile_label;
    const auto topic_req_id = uxr_buffer_create_topic_ref(&session,reliable_out,topic_id,participant_id,topic_ref,UXR_REPLACE);

    // Publisher
    const uxrObjectId pub_id = {
        .id = topics[0].pub_id,
        .type = UXR_PUBLISHER_ID
    };
    const char* pub_xml = "";
    const auto pub_req_id = uxr_buffer_create_publisher_xml(&session,reliable_out,pub_id,participant_id,pub_xml,UXR_REPLACE);

    // Data Writer
    const char* data_writer_ref = topics[0].dw_profile_label;
    const auto dwriter_req_id = uxr_buffer_create_datawriter_ref(&session,reliable_out,dwriter_id,pub_id,data_writer_ref,UXR_REPLACE);

    //Status requests
    constexpr uint8_t nRequests = 4;
    const uint16_t requests[nRequests] = {participant_req_id, topic_req_id, pub_req_id, dwriter_req_id};

    constexpr int maxTimeMsPerRequestMs = 250;
    constexpr int requestTimeoutMs = nRequests * maxTimeMsPerRequestMs;
    uint8_t status[nRequests];
    if (!uxr_run_session_until_all_status(&session, requestTimeoutMs, requests, status, nRequests)) {
        // TODO add a failure log message sharing the status results
        return false;
    }

    return true;
}

void AP_DDS_Client::write()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        ucdrBuffer ub;
        uint32_t topic_size = builtin_interfaces_msg_Time_size_of_topic(&time_topic, 0);
        uxr_prepare_output_stream(&session,reliable_out,dwriter_id,&ub,topic_size);
        const bool success = builtin_interfaces_msg_Time_serialize_topic(&ub, &time_topic);
        if (!success) {
            // TODO sometimes serialization fails on bootup. Determine why.
            // AP_HAL::panic("FATAL: DDS_Client failed to serialize\n");
        }

    }

}

void AP_DDS_Client::update()
{
    WITH_SEMAPHORE(csem);

    update_topic(time_topic);
    write();
    connected = uxr_run_session_time(&session, 1);
}

/*
  implement C functions for serial transport
 */
extern "C" {
#include <uxr/client/profile/transport/serial/serial_transport_platform.h>
}

bool uxr_init_serial_platform(void* args, int fd, uint8_t remote_addr, uint8_t local_addr)
{
    //! @todo Add error reporting
    return true;
}

bool uxr_close_serial_platform(void* args)
{
    //! @todo Add error reporting
    return true;
}

size_t uxr_write_serial_data_platform(void* args, const uint8_t* buf, size_t len, uint8_t* errcode)
{
    if (dds_port == nullptr) {
        *errcode = 1;
        return 0;
    }
    ssize_t bytes_written = dds_port->write(buf, len);
    if (bytes_written <= 0) {
        *errcode = 1;
        return 0;
    }
    //! @todo Add populate the error code correctly
    *errcode = 0;
    return bytes_written;
}

size_t uxr_read_serial_data_platform(void* args, uint8_t* buf, size_t len, int timeout, uint8_t* errcode)
{
    if (dds_port == nullptr) {
        *errcode = 1;
        return 0;
    }
    while (timeout > 0 && dds_port->available() < len) {
        hal.scheduler->delay(1); // TODO select or poll this is limiting speed (1mS)
        timeout--;
    }
    ssize_t bytes_read = dds_port->read(buf, len);
    if (bytes_read <= 0) {
        *errcode = 1;
        return 0;
    }
    //! @todo Add error reporting
    *errcode = 0;
    return bytes_read;
}

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
extern "C" {
    int clock_gettime(clockid_t clockid, struct timespec *ts);
}

int clock_gettime(clockid_t clockid, struct timespec *ts)
{
    //! @todo the value of clockid is ignored here.
    //! A fallback mechanism is employed against the caller's choice of clock.
    uint64_t utc_usec;
    if (!AP::rtc().get_utc_usec(utc_usec)) {
        utc_usec = AP_HAL::micros64();
    }
    ts->tv_sec = utc_usec / 1000000ULL;
    ts->tv_nsec = (utc_usec % 1000000ULL) * 1000UL;
    return 0;
}
#endif // CONFIG_HAL_BOARD

#endif // AP_DDS_ENABLED


