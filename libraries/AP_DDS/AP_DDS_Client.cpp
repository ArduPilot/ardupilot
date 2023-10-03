#include <AP_HAL/AP_HAL_Boards.h>

#if AP_DDS_ENABLED

#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_ExternalControl/AP_ExternalControl_config.h>

#include <AP_ROS/AP_ROS_Client.h>

#if AP_EXTERNAL_CONTROL_ENABLED
#include "AP_DDS_ExternalControl.h"
#endif
// #include "AP_DDS_Frames.h"

#include "AP_DDS_Client.h"
#include "AP_DDS_Topic_Table.h"
#include "AP_DDS_Service_Table.h"
#include "AP_DDS_External_Odom.h"
#include "AP_DDS_Type_Conversions.h"

// Enable DDS at runtime by default
static constexpr uint8_t ENABLED_BY_DEFAULT = 1;
static constexpr uint16_t DELAY_TIME_TOPIC_MS = 10;
static constexpr uint16_t DELAY_BATTERY_STATE_TOPIC_MS = 1000;
static constexpr uint16_t DELAY_LOCAL_POSE_TOPIC_MS = 33;
static constexpr uint16_t DELAY_LOCAL_VELOCITY_TOPIC_MS = 33;
static constexpr uint16_t DELAY_GEO_POSE_TOPIC_MS = 33;
static constexpr uint16_t DELAY_CLOCK_TOPIC_MS = 10;

// Define the subscriber data members, which are static class scope.
// If these are created on the stack in the subscriber,
// the AP_DDS_Client::on_topic frame size is exceeded.
sensor_msgs_msg_Joy AP_DDS_Client::rx_joy_topic {};
tf2_msgs_msg_TFMessage AP_DDS_Client::rx_dynamic_transforms_topic {};
geometry_msgs_msg_TwistStamped AP_DDS_Client::rx_velocity_control_topic {};


const AP_Param::GroupInfo AP_DDS_Client::var_info[] {

    // @Param: _ENABLE
    // @DisplayName: DDS enable
    // @Description: Enable DDS subsystem
    // @Values: 0:Disabled,1:Enabled
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_ENABLE", 1, AP_DDS_Client, enabled, ENABLED_BY_DEFAULT, AP_PARAM_FLAG_ENABLE),

#if AP_DDS_UDP_ENABLED
    // @Param: _UDP_PORT
    // @DisplayName: DDS UDP port
    // @Description: UDP port number for DDS
    // @Range: 1 65535
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("_PORT", 2, AP_DDS_Client, udp.port, 2019),

#endif

    AP_GROUPEND
};

void AP_DDS_Client::update_topic(builtin_interfaces_msg_Time& msg)
{
    AP_ROS_Client::update_time(msg);
}

bool AP_DDS_Client::update_topic(sensor_msgs_msg_NavSatFix& msg, const uint8_t instance)
{
    return AP_ROS_Client::update_nav_sat_fix(msg, instance, last_nav_sat_fix_time_ms);
}

void AP_DDS_Client::populate_static_transforms(tf2_msgs_msg_TFMessage& msg)
{
    AP_ROS_Client::update_static_transforms(msg);
}

void AP_DDS_Client::update_topic(sensor_msgs_msg_BatteryState& msg, const uint8_t instance)
{
    AP_ROS_Client::update_battery_state(msg, instance);
}

void AP_DDS_Client::update_topic(geometry_msgs_msg_PoseStamped& msg)
{
    AP_ROS_Client::update_pose_stamped(msg);
}

void AP_DDS_Client::update_topic(geometry_msgs_msg_TwistStamped& msg)
{
    AP_ROS_Client::update_twist_stamped(msg);
}

void AP_DDS_Client::update_topic(geographic_msgs_msg_GeoPoseStamped& msg)
{
    AP_ROS_Client::update_geopose_stamped(msg);
}

void AP_DDS_Client::update_topic(rosgraph_msgs_msg_Clock& msg)
{
    AP_ROS_Client::update_clock(msg);
}

/*
  start the DDS thread
 */
bool AP_DDS_Client::start(void)
{
    AP_Param::setup_object_defaults(this, var_info);
    AP_Param::load_object_from_eeprom(this, var_info);

    if (enabled == 0) {
        return true;
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_DDS_Client::main_loop, void),
                                      "DDS",
                                      8192, AP_HAL::Scheduler::PRIORITY_IO, 1)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"DDS Client: thread create failed");
        return false;
    }
    return true;
}

// read function triggered at every subscription callback
void AP_DDS_Client::on_topic_trampoline(uxrSession* uxr_session, uxrObjectId object_id, uint16_t request_id, uxrStreamId stream_id, struct ucdrBuffer* ub, uint16_t length,
                                        void* args)
{
    AP_DDS_Client *dds = (AP_DDS_Client *)args;
    dds->on_topic(uxr_session, object_id, request_id, stream_id, ub, length);
}

void AP_DDS_Client::on_topic(uxrSession* uxr_session, uxrObjectId object_id, uint16_t request_id, uxrStreamId stream_id, struct ucdrBuffer* ub, uint16_t length)
{
    /*
    TEMPLATE for reading to the subscribed topics
    1) Store the read contents into the ucdr buffer
    2) Deserialize the said contents into the topic instance
    */
    (void) uxr_session;
    (void) request_id;
    (void) stream_id;
    (void) length;
    switch (object_id.id) {
    case topics[to_underlying(TopicIndex::JOY_SUB)].dr_id.id: {
        const bool success = sensor_msgs_msg_Joy_deserialize_topic(ub, &rx_joy_topic);

        if (success == false) {
            break;
        }

        if (rx_joy_topic.axes_size >= 4) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,"Received sensor_msgs/Joy: %f, %f, %f, %f",
                          rx_joy_topic.axes[0], rx_joy_topic.axes[1], rx_joy_topic.axes[2], rx_joy_topic.axes[3]);
            // TODO implement joystick RC control to AP
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,"Received sensor_msgs/Joy: Insufficient axes size ");
        }
        break;
    }
    case topics[to_underlying(TopicIndex::DYNAMIC_TRANSFORMS_SUB)].dr_id.id: {
        const bool success = tf2_msgs_msg_TFMessage_deserialize_topic(ub, &rx_dynamic_transforms_topic);
        if (success == false) {
            break;
        }

        if (rx_dynamic_transforms_topic.transforms_size > 0) {
#if AP_DDS_VISUALODOM_ENABLED
            AP_DDS_External_Odom::handle_external_odom(rx_dynamic_transforms_topic);
#endif // AP_DDS_VISUALODOM_ENABLED

        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,"Received tf2_msgs/TFMessage: Insufficient size ");
        }
        break;
    }
    case topics[to_underlying(TopicIndex::VELOCITY_CONTROL_SUB)].dr_id.id: {
        const bool success = geometry_msgs_msg_TwistStamped_deserialize_topic(ub, &rx_velocity_control_topic);
        if (success == false) {
            break;
        }

#if AP_EXTERNAL_CONTROL_ENABLED
        if (!AP_DDS_External_Control::handle_velocity_control(rx_velocity_control_topic)) {
            // TODO #23430 handle velocity control failure through rosout, throttled.
        }
#endif // AP_EXTERNAL_CONTROL_ENABLED
        break;
    }
    }

}

/*
  callback on request completion
 */
void AP_DDS_Client::on_request_trampoline(uxrSession* uxr_session, uxrObjectId object_id, uint16_t request_id, SampleIdentity* sample_id, ucdrBuffer* ub, uint16_t length, void* args)
{
    AP_DDS_Client *dds = (AP_DDS_Client *)args;
    dds->on_request(uxr_session, object_id, request_id, sample_id, ub, length);
}

void AP_DDS_Client::on_request(uxrSession* uxr_session, uxrObjectId object_id, uint16_t request_id, SampleIdentity* sample_id, ucdrBuffer* ub, uint16_t length)
{
    (void) request_id;
    (void) length;
    switch (object_id.id) {
    case services[to_underlying(ServiceIndex::ARMING_MOTORS)].rep_id: {
        bool arm;
        bool result;
        const bool deserialize_success = ucdr_deserialize_bool(ub,&arm);
        if (deserialize_success == false) {
            break;
        }

        if (arm) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,"Request for arming received");
            result = AP::arming().arm(AP_Arming::Method::DDS);
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,"Request for disarming received");
            result = AP::arming().disarm(AP_Arming::Method::DDS);
        }

        const uxrObjectId replier_id = {
            .id = services[to_underlying(ServiceIndex::ARMING_MOTORS)].rep_id,
            .type = UXR_REPLIER_ID
        };

        //Todo : Fix the size-handling of services with the help of the functions autogenerated via Micro-XRCE-DDS Gen
        uint8_t reply_buffer[8] {};
        ucdrBuffer reply_ub;

        ucdr_init_buffer(&reply_ub, reply_buffer, sizeof(reply_buffer));
        const bool serialize_success = ucdr_serialize_bool(&reply_ub,result);
        if (serialize_success == false) {
            break;
        }

        uxr_buffer_reply(uxr_session, reliable_out, replier_id, sample_id, reply_buffer, ucdr_buffer_length(&reply_ub));
        if (result) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,"DDS Client: Request for Arming/Disarming : SUCCESS");
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,"DDS Client: Request for Arming/Disarming : FAIL");
        }
        break;
    }
    case services[to_underlying(ServiceIndex::MODE_SWITCH)].rep_id: {
        uint8_t mode;
        const bool deserialize_success = ucdr_deserialize_uint8_t(ub,&mode);
        if (deserialize_success == false) {
            break;
        }
        bool status = AP::vehicle()->set_mode(mode, ModeReason::DDS_COMMAND);
        uint8_t curr_mode = AP::vehicle()->get_mode();

        const uxrObjectId replier_id = {
            .id = services[to_underlying(ServiceIndex::MODE_SWITCH)].rep_id,
            .type = UXR_REPLIER_ID
        };

        //Todo : Fix the size-handling of services with the help of the functions autogenerated via Micro-XRCE-DDS Gen
        uint8_t reply_buffer[8] {};
        ucdrBuffer reply_ub;

        ucdr_init_buffer(&reply_ub, reply_buffer, sizeof(reply_buffer));
        bool serialize_success = true;
        serialize_success &= ucdr_serialize_bool(&reply_ub, status);
        serialize_success &= ucdr_serialize_uint8_t(&reply_ub, curr_mode);
        if (serialize_success == false || reply_ub.error) {
            break;
        }

        uxr_buffer_reply(uxr_session, reliable_out, replier_id, sample_id, reply_buffer, ucdr_buffer_length(&reply_ub));
        if (status) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,"DDS Client: Request for Mode Switch : SUCCESS");
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,"DDS Client: Request for Mode Switch : FAIL");
        }
        break;
    }
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

    populate_static_transforms(tx_static_transforms_topic);
    write_static_transforms();

    while (true) {
        hal.scheduler->delay(1);
        update();
    }
}

bool AP_DDS_Client::init()
{
    // serial init will fail if the SERIALn_PROTOCOL is not setup
    bool initTransportStatus = ddsSerialInit();
    is_using_serial = initTransportStatus;

#if AP_DDS_UDP_ENABLED
    // fallback to UDP if available
    if (!initTransportStatus) {
        initTransportStatus = ddsUdpInit();
    }
#endif

    if (!initTransportStatus) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"DDS Client: Transport Initialization failed");
        return false;
    }

    // Register topic callbacks
    uxr_set_topic_callback(&session, AP_DDS_Client::on_topic_trampoline, this);

    // ROS-2 Service : Register service request callbacks
    uxr_set_request_callback(&session, AP_DDS_Client::on_request_trampoline, this);

    while (!uxr_create_session(&session)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"DDS Client: Initialization waiting...");
        hal.scheduler->delay(1000);
    }

    // setup reliable stream buffers
    input_reliable_stream = new uint8_t[DDS_BUFFER_SIZE];
    if (input_reliable_stream == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"DDS Client: allocation failed");
        return false;
    }
    output_reliable_stream = new uint8_t[DDS_BUFFER_SIZE];
    if (output_reliable_stream == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"DDS Client: allocation failed");
        return false;
    }

    reliable_in = uxr_create_input_reliable_stream(&session, input_reliable_stream, DDS_BUFFER_SIZE, DDS_STREAM_HISTORY);
    reliable_out = uxr_create_output_reliable_stream(&session, output_reliable_stream, DDS_BUFFER_SIZE, DDS_STREAM_HISTORY);

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

    //Participant requests
    constexpr uint8_t nRequestsParticipant = 1;
    const uint16_t requestsParticipant[nRequestsParticipant] = {participant_req_id};

    constexpr uint8_t maxTimeMsPerRequestMs = 250;
    constexpr uint16_t requestTimeoutParticipantMs = (uint16_t) nRequestsParticipant * maxTimeMsPerRequestMs;
    uint8_t statusParticipant[nRequestsParticipant];
    if (!uxr_run_session_until_all_status(&session, requestTimeoutParticipantMs, requestsParticipant, statusParticipant, nRequestsParticipant)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"XRCE Client: Participant session request failure");
        // TODO add a failure log message sharing the status results
        return false;
    }

    for (uint16_t i = 0 ; i < ARRAY_SIZE(topics); i++) {
        // Topic
        const uxrObjectId topic_id = {
            .id = topics[i].topic_id,
            .type = UXR_TOPIC_ID
        };
        const char* topic_ref = topics[i].topic_profile_label;
        const auto topic_req_id = uxr_buffer_create_topic_ref(&session,reliable_out,topic_id,participant_id,topic_ref,UXR_REPLACE);

        // Status requests
        constexpr uint8_t nRequests = 3;
        uint16_t requests[nRequests];
        constexpr uint16_t requestTimeoutMs = nRequests * maxTimeMsPerRequestMs;
        uint8_t status[nRequests];

        if (strlen(topics[i].dw_profile_label) > 0) {
            // Publisher
            const uxrObjectId pub_id = {
                .id = topics[i].pub_id,
                .type = UXR_PUBLISHER_ID
            };
            const char* pub_xml = "";
            const auto pub_req_id = uxr_buffer_create_publisher_xml(&session,reliable_out,pub_id,participant_id,pub_xml,UXR_REPLACE);

            // Data Writer
            const char* data_writer_ref = topics[i].dw_profile_label;
            const auto dwriter_req_id = uxr_buffer_create_datawriter_ref(&session,reliable_out,topics[i].dw_id,pub_id,data_writer_ref,UXR_REPLACE);

            // save the request statuses
            requests[0] = topic_req_id;
            requests[1] = pub_req_id;
            requests[2] = dwriter_req_id;

            if (!uxr_run_session_until_all_status(&session, requestTimeoutMs, requests, status, nRequests)) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"XRCE Client: Topic/Pub/Writer session request failure for index '%u'",i);
                for (uint8_t s = 0 ; s < nRequests; s++) {
                    GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"XRCE Client: Status '%d' result '%u'", s, status[s]);
                }
                // TODO add a failure log message sharing the status results
                return false;
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO,"XRCE Client: Topic/Pub/Writer session pass for index '%u'",i);
            }
        } else if (strlen(topics[i].dr_profile_label) > 0) {
            // Subscriber
            const uxrObjectId sub_id = {
                .id = topics[i].sub_id,
                .type = UXR_SUBSCRIBER_ID
            };
            const char* sub_xml = "";
            const auto sub_req_id = uxr_buffer_create_subscriber_xml(&session,reliable_out,sub_id,participant_id,sub_xml,UXR_REPLACE);

            // Data Reader
            const char* data_reader_ref = topics[i].dr_profile_label;
            const auto dreader_req_id = uxr_buffer_create_datareader_ref(&session,reliable_out,topics[i].dr_id,sub_id,data_reader_ref,UXR_REPLACE);

            // save the request statuses
            requests[0] = topic_req_id;
            requests[1] = sub_req_id;
            requests[2] = dreader_req_id;

            if (!uxr_run_session_until_all_status(&session, requestTimeoutMs, requests, status, nRequests)) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"XRCE Client: Topic/Sub/Reader session request failure for index '%u'",i);
                for (uint8_t s = 0 ; s < nRequests; s++) {
                    GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"XRCE Client: Status '%d' result '%u'", s, status[s]);
                }
                // TODO add a failure log message sharing the status results
                return false;
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO,"XRCE Client: Topic/Sub/Reader session pass for index '%u'",i);
                uxr_buffer_request_data(&session, reliable_out, topics[i].dr_id, reliable_in, &delivery_control);
            }
        }
    }

    // ROS-2 Service : else case for service requests

    for (uint16_t i = 0; i < ARRAY_SIZE(services); i++) {

        constexpr uint16_t requestTimeoutMs = maxTimeMsPerRequestMs;

        if (strlen(services[i].rep_profile_label) > 0) {
            const uxrObjectId rep_id = {
                .id = services[i].rep_id,
                .type = UXR_REPLIER_ID
            };
            const char* replier_ref = services[i].rep_profile_label;
            const auto  replier_req_id = uxr_buffer_create_replier_ref(&session, reliable_out, rep_id, participant_id, replier_ref, UXR_REPLACE);

            uint16_t request = replier_req_id;
            uint8_t status;

            if (!uxr_run_session_until_all_status(&session, requestTimeoutMs, &request, &status, 1)) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"XRCE Client: Service/Replier session request failure for index '%u'",i);
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"XRCE Client: Status result '%u'", status);
                // TODO add a failure log message sharing the status results
                return false;
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO,"XRCE Client: Service/Replier session pass for index '%u'",i);
                uxr_buffer_request_data(&session, reliable_out, rep_id, reliable_in, &delivery_control);
            }

        } else if (strlen(services[i].req_profile_label) > 0) {
            // TODO : Add Similar Code for Requester Profile
        }
    }

    return true;
}

void AP_DDS_Client::write_time_topic()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        ucdrBuffer ub {};
        const uint32_t topic_size = builtin_interfaces_msg_Time_size_of_topic(&time_topic, 0);
        uxr_prepare_output_stream(&session,reliable_out,topics[0].dw_id,&ub,topic_size);
        const bool success = builtin_interfaces_msg_Time_serialize_topic(&ub, &time_topic);
        if (!success) {
            // TODO sometimes serialization fails on bootup. Determine why.
            // AP_HAL::panic("FATAL: XRCE_Client failed to serialize\n");
        }
    }
}

void AP_DDS_Client::write_nav_sat_fix_topic()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        ucdrBuffer ub {};
        const uint32_t topic_size = sensor_msgs_msg_NavSatFix_size_of_topic(&nav_sat_fix_topic, 0);
        uxr_prepare_output_stream(&session,reliable_out,topics[1].dw_id,&ub,topic_size);
        const bool success = sensor_msgs_msg_NavSatFix_serialize_topic(&ub, &nav_sat_fix_topic);
        if (!success) {
            // TODO sometimes serialization fails on bootup. Determine why.
            // AP_HAL::panic("FATAL: DDS_Client failed to serialize\n");
        }
    }
}

void AP_DDS_Client::write_static_transforms()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        ucdrBuffer ub {};
        const uint32_t topic_size = tf2_msgs_msg_TFMessage_size_of_topic(&tx_static_transforms_topic, 0);
        uxr_prepare_output_stream(&session,reliable_out,topics[2].dw_id,&ub,topic_size);
        const bool success = tf2_msgs_msg_TFMessage_serialize_topic(&ub, &tx_static_transforms_topic);
        if (!success) {
            // TODO sometimes serialization fails on bootup. Determine why.
            // AP_HAL::panic("FATAL: DDS_Client failed to serialize\n");
        }
    }
}

void AP_DDS_Client::write_battery_state_topic()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        ucdrBuffer ub {};
        const uint32_t topic_size = sensor_msgs_msg_BatteryState_size_of_topic(&battery_state_topic, 0);
        uxr_prepare_output_stream(&session,reliable_out,topics[3].dw_id,&ub,topic_size);
        const bool success = sensor_msgs_msg_BatteryState_serialize_topic(&ub, &battery_state_topic);
        if (!success) {
            // TODO sometimes serialization fails on bootup. Determine why.
            // AP_HAL::panic("FATAL: DDS_Client failed to serialize\n");
        }
    }
}

void AP_DDS_Client::write_local_pose_topic()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        ucdrBuffer ub {};
        const uint32_t topic_size = geometry_msgs_msg_PoseStamped_size_of_topic(&local_pose_topic, 0);
        uxr_prepare_output_stream(&session,reliable_out,topics[4].dw_id,&ub,topic_size);
        const bool success = geometry_msgs_msg_PoseStamped_serialize_topic(&ub, &local_pose_topic);
        if (!success) {
            // TODO sometimes serialization fails on bootup. Determine why.
            // AP_HAL::panic("FATAL: DDS_Client failed to serialize\n");
        }
    }
}

void AP_DDS_Client::write_tx_local_velocity_topic()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        ucdrBuffer ub {};
        const uint32_t topic_size = geometry_msgs_msg_TwistStamped_size_of_topic(&tx_local_velocity_topic, 0);
        uxr_prepare_output_stream(&session,reliable_out,topics[5].dw_id,&ub,topic_size);
        const bool success = geometry_msgs_msg_TwistStamped_serialize_topic(&ub, &tx_local_velocity_topic);
        if (!success) {
            // TODO sometimes serialization fails on bootup. Determine why.
            // AP_HAL::panic("FATAL: DDS_Client failed to serialize\n");
        }
    }
}

void AP_DDS_Client::write_geo_pose_topic()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        ucdrBuffer ub {};
        const uint32_t topic_size = geographic_msgs_msg_GeoPoseStamped_size_of_topic(&geo_pose_topic, 0);
        uxr_prepare_output_stream(&session,reliable_out,topics[6].dw_id,&ub,topic_size);
        const bool success = geographic_msgs_msg_GeoPoseStamped_serialize_topic(&ub, &geo_pose_topic);
        if (!success) {
            // TODO sometimes serialization fails on bootup. Determine why.
            // AP_HAL::panic("FATAL: DDS_Client failed to serialize\n");
        }
    }
}

void AP_DDS_Client::write_clock_topic()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        ucdrBuffer ub {};
        const uint32_t topic_size = rosgraph_msgs_msg_Clock_size_of_topic(&clock_topic, 0);
        uxr_prepare_output_stream(&session,reliable_out,topics[7].dw_id,&ub,topic_size);
        const bool success = rosgraph_msgs_msg_Clock_serialize_topic(&ub, &clock_topic);
        if (!success) {
            // TODO sometimes serialization fails on bootup. Determine why.
            // AP_HAL::panic("FATAL: DDS_Client failed to serialize\n");
        }
    }
}

void AP_DDS_Client::update()
{
    WITH_SEMAPHORE(csem);
    const auto cur_time_ms = AP_HAL::millis64();

    if (cur_time_ms - last_time_time_ms > DELAY_TIME_TOPIC_MS) {
        update_topic(time_topic);
        last_time_time_ms = cur_time_ms;
        write_time_topic();
    }

    constexpr uint8_t gps_instance = 0;
    if (update_topic(nav_sat_fix_topic, gps_instance)) {
        write_nav_sat_fix_topic();
    }

    if (cur_time_ms - last_battery_state_time_ms > DELAY_BATTERY_STATE_TOPIC_MS) {
        constexpr uint8_t battery_instance = 0;
        update_topic(battery_state_topic, battery_instance);
        last_battery_state_time_ms = cur_time_ms;
        write_battery_state_topic();
    }

    if (cur_time_ms - last_local_pose_time_ms > DELAY_LOCAL_POSE_TOPIC_MS) {
        update_topic(local_pose_topic);
        last_local_pose_time_ms = cur_time_ms;
        write_local_pose_topic();
    }

    if (cur_time_ms - last_local_velocity_time_ms > DELAY_LOCAL_VELOCITY_TOPIC_MS) {
        update_topic(tx_local_velocity_topic);
        last_local_velocity_time_ms = cur_time_ms;
        write_tx_local_velocity_topic();
    }

    if (cur_time_ms - last_geo_pose_time_ms > DELAY_GEO_POSE_TOPIC_MS) {
        update_topic(geo_pose_topic);
        last_geo_pose_time_ms = cur_time_ms;
        write_geo_pose_topic();
    }

    if (cur_time_ms - last_clock_time_ms > DELAY_CLOCK_TOPIC_MS) {
        update_topic(clock_topic);
        last_clock_time_ms = cur_time_ms;
        write_clock_topic();
    }

    connected = uxr_run_session_time(&session, 1);
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
#endif // CONFIG_HAL_BOARD != HAL_BOARD_SITL

#endif // AP_DDS_ENABLED


