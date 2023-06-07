#include <AP_HAL/AP_HAL_Boards.h>

#if AP_DDS_ENABLED

#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_AHRS/AP_AHRS.h>

#include "AP_DDS_Client.h"

static constexpr uint16_t DELAY_TIME_TOPIC_MS = 10;
static constexpr uint16_t DELAY_BATTERY_STATE_TOPIC_MS = 1000;
static constexpr uint16_t DELAY_LOCAL_POSE_TOPIC_MS = 33;
static constexpr uint16_t DELAY_LOCAL_VELOCITY_TOPIC_MS = 33;
static constexpr uint16_t DELAY_GEO_POSE_TOPIC_MS = 33;
static constexpr uint16_t DELAY_CLOCK_TOPIC_MS = 10;
static char WGS_84_FRAME_ID[] = "WGS-84";
// https://www.ros.org/reps/rep-0105.html#base-link
static char BASE_LINK_FRAME_ID[] = "base_link";

const AP_Param::GroupInfo AP_DDS_Client::var_info[] {

    // @Param: _ENABLE
    // @DisplayName: DDS enable
    // @Description: Enable DDS subsystem
    // @Values: 0:Disabled,1:Enabled
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_ENABLE", 1, AP_DDS_Client, enabled, 0, AP_PARAM_FLAG_ENABLE),

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

bool AP_DDS_Client::update_topic(sensor_msgs_msg_NavSatFix& msg, const uint8_t instance)
{
    // Add a lambda that takes in navsatfix msg and populates the cov
    // Make it constexpr if possible
    // https://www.fluentcpp.com/2021/12/13/the-evolutions-of-lambdas-in-c14-c17-and-c20/
    // constexpr auto times2 = [] (sensor_msgs_msg_NavSatFix* msg) { return n * 2; };

    // assert(instance >= GPS_MAX_RECEIVERS);
    if (instance >= GPS_MAX_RECEIVERS) {
        return false;
    }

    auto &gps = AP::gps();
    WITH_SEMAPHORE(gps.get_semaphore());

    if (!gps.is_healthy(instance)) {
        msg.status.status = -1; // STATUS_NO_FIX
        msg.status.service = 0; // No services supported
        msg.position_covariance_type = 0; // COVARIANCE_TYPE_UNKNOWN
        return false;
    }

    // No update is needed
    const auto last_fix_time_ms = gps.last_fix_time_ms(instance);
    if (last_nav_sat_fix_time_ms == last_fix_time_ms) {
        return false;
    } else {
        last_nav_sat_fix_time_ms = last_fix_time_ms;
    }


    update_topic(msg.header.stamp);
    strcpy(msg.header.frame_id, WGS_84_FRAME_ID);
    msg.status.service = 0; // SERVICE_GPS
    msg.status.status = -1; // STATUS_NO_FIX


    //! @todo What about glonass, compass, galileo?
    //! This will be properly designed and implemented to spec in #23277
    msg.status.service = 1; // SERVICE_GPS

    const auto status = gps.status(instance);
    switch (status) {
    case AP_GPS::NO_GPS:
    case AP_GPS::NO_FIX:
        msg.status.status = -1; // STATUS_NO_FIX
        msg.position_covariance_type = 0; // COVARIANCE_TYPE_UNKNOWN
        return true;
    case AP_GPS::GPS_OK_FIX_2D:
    case AP_GPS::GPS_OK_FIX_3D:
        msg.status.status = 0; // STATUS_FIX
        break;
    case AP_GPS::GPS_OK_FIX_3D_DGPS:
        msg.status.status = 1; // STATUS_SBAS_FIX
        break;
    case AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT:
    case AP_GPS::GPS_OK_FIX_3D_RTK_FIXED:
        msg.status.status = 2; // STATUS_SBAS_FIX
        break;
    default:
        //! @todo Can we not just use an enum class and not worry about this condition?
        break;
    }
    const auto loc = gps.location(instance);
    msg.latitude = loc.lat * 1E-7;
    msg.longitude = loc.lng * 1E-7;

    int32_t alt_cm;
    if (!loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_cm)) {
        // With absolute frame, this condition is unlikely
        msg.status.status = -1; // STATUS_NO_FIX
        msg.position_covariance_type = 0; // COVARIANCE_TYPE_UNKNOWN
        return true;
    }
    msg.altitude = alt_cm * 0.01;

    // ROS allows double precision, ArduPilot exposes float precision today
    Matrix3f cov;
    msg.position_covariance_type = (uint8_t)gps.position_covariance(instance, cov);
    msg.position_covariance[0] = cov[0][0];
    msg.position_covariance[1] = cov[0][1];
    msg.position_covariance[2] = cov[0][2];
    msg.position_covariance[3] = cov[1][0];
    msg.position_covariance[4] = cov[1][1];
    msg.position_covariance[5] = cov[1][2];
    msg.position_covariance[6] = cov[2][0];
    msg.position_covariance[7] = cov[2][1];
    msg.position_covariance[8] = cov[2][2];

    return true;
}

void AP_DDS_Client::populate_static_transforms(tf2_msgs_msg_TFMessage& msg)
{
    msg.transforms_size = 0;

    auto &gps = AP::gps();
    for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
        const auto gps_type = gps.get_type(i);
        if (gps_type == AP_GPS::GPS_Type::GPS_TYPE_NONE) {
            continue;
        }
        update_topic(msg.transforms[i].header.stamp);
        char gps_frame_id[16];
        //! @todo should GPS frame ID's be 0 or 1 indexed in ROS?
        hal.util->snprintf(gps_frame_id, sizeof(gps_frame_id), "GPS_%u", i);
        strcpy(msg.transforms[i].header.frame_id, BASE_LINK_FRAME_ID);
        strcpy(msg.transforms[i].child_frame_id, gps_frame_id);
        // The body-frame offsets
        // X - Forward
        // Y - Right
        // Z - Down
        // https://ardupilot.org/copter/docs/common-sensor-offset-compensation.html#sensor-position-offset-compensation

        const auto offset = gps.get_antenna_offset(i);

        // In ROS REP 103, it follows this convention
        // X - Forward
        // Y - Left
        // Z - Up
        // https://www.ros.org/reps/rep-0103.html#axis-orientation

        msg.transforms[i].transform.translation.x = offset[0];
        msg.transforms[i].transform.translation.y = -1 * offset[1];
        msg.transforms[i].transform.translation.z = -1 * offset[2];

        msg.transforms_size++;
    }

}

void AP_DDS_Client::update_topic(sensor_msgs_msg_BatteryState& msg, const uint8_t instance)
{
    if (instance >= AP_BATT_MONITOR_MAX_INSTANCES) {
        return;
    }

    update_topic(msg.header.stamp);
    auto &battery = AP::battery();

    if (!battery.healthy(instance)) {
        msg.power_supply_status = 3; //POWER_SUPPLY_HEALTH_DEAD
        msg.present = false;
        return;
    }
    msg.present = true;

    msg.voltage = battery.voltage(instance);

    float temperature;
    msg.temperature = (battery.get_temperature(temperature, instance)) ? temperature : NAN;

    float current;
    msg.current = (battery.current_amps(current, instance)) ? -1 * current : NAN;

    const float design_capacity = (float)battery.pack_capacity_mah(instance) * 0.001;
    msg.design_capacity = design_capacity;

    uint8_t percentage;
    if (battery.capacity_remaining_pct(percentage, instance)) {
        msg.percentage = percentage * 0.01;
        msg.charge = (percentage * design_capacity) * 0.01;
    } else {
        msg.percentage = NAN;
        msg.charge = NAN;
    }

    msg.capacity = NAN;

    if (battery.current_amps(current, instance)) {
        if (percentage == 100) {
            msg.power_supply_status = 4;   //POWER_SUPPLY_STATUS_FULL
        } else if (current < 0.0) {
            msg.power_supply_status = 1;   //POWER_SUPPLY_STATUS_CHARGING
        } else if (current > 0.0) {
            msg.power_supply_status = 2;   //POWER_SUPPLY_STATUS_DISCHARGING
        } else {
            msg.power_supply_status = 3;   //POWER_SUPPLY_STATUS_NOT_CHARGING
        }
    } else {
        msg.power_supply_status = 0; //POWER_SUPPLY_STATUS_UNKNOWN
    }

    msg.power_supply_health = (battery.overpower_detected(instance)) ? 4 : 1; //POWER_SUPPLY_HEALTH_OVERVOLTAGE or POWER_SUPPLY_HEALTH_GOOD

    msg.power_supply_technology = 0; //POWER_SUPPLY_TECHNOLOGY_UNKNOWN

    if (battery.has_cell_voltages(instance)) {
        const uint16_t* cellVoltages = battery.get_cell_voltages(instance).cells;
        std::copy(cellVoltages, cellVoltages + AP_BATT_MONITOR_CELLS_MAX, msg.cell_voltage);
    }
}

void AP_DDS_Client::update_topic(geometry_msgs_msg_PoseStamped& msg)
{
    update_topic(msg.header.stamp);
    strcpy(msg.header.frame_id, BASE_LINK_FRAME_ID);

    auto &ahrs = AP::ahrs();
    WITH_SEMAPHORE(ahrs.get_semaphore());

    // ROS REP 103 uses the ENU convention:
    // X - East
    // Y - North
    // Z - Up
    // https://www.ros.org/reps/rep-0103.html#axis-orientation
    // AP_AHRS uses the NED convention
    // X - North
    // Y - East
    // Z - Down
    // As a consequence, to follow ROS REP 103, it is necessary to switch X and Y,
    // as well as invert Z

    Vector3f position;
    if (ahrs.get_relative_position_NED_home(position)) {
        msg.pose.position.x = position[1];
        msg.pose.position.y = position[0];
        msg.pose.position.z = -position[2];
    }

    // In ROS REP 103, axis orientation uses the following convention:
    // X - Forward
    // Y - Left
    // Z - Up
    // https://www.ros.org/reps/rep-0103.html#axis-orientation
    // As a consequence, to follow ROS REP 103, it is necessary to switch X and Y,
    // as well as invert Z (NED to ENU convertion) as well as a 90 degree rotation in the Z axis
    // for x to point forward
    Quaternion orientation;
    if (ahrs.get_quaternion(orientation)) {
        Quaternion aux(orientation[0], orientation[2], orientation[1], -orientation[3]); //NED to ENU transformation
        Quaternion transformation (sqrtF(2) * 0.5,0,0,sqrtF(2) * 0.5); // Z axis 90 degree rotation
        orientation = aux * transformation;
        msg.pose.orientation.w = orientation[0];
        msg.pose.orientation.x = orientation[1];
        msg.pose.orientation.y = orientation[2];
        msg.pose.orientation.z = orientation[3];
    }
}

void AP_DDS_Client::update_topic(geometry_msgs_msg_TwistStamped& msg)
{
    update_topic(msg.header.stamp);
    strcpy(msg.header.frame_id, BASE_LINK_FRAME_ID);

    auto &ahrs = AP::ahrs();
    WITH_SEMAPHORE(ahrs.get_semaphore());

    // ROS REP 103 uses the ENU convention:
    // X - East
    // Y - North
    // Z - Up
    // https://www.ros.org/reps/rep-0103.html#axis-orientation
    // AP_AHRS uses the NED convention
    // X - North
    // Y - East
    // Z - Down
    // As a consequence, to follow ROS REP 103, it is necessary to switch X and Y,
    // as well as invert Z
    Vector3f velocity;
    if (ahrs.get_velocity_NED(velocity)) {
        msg.twist.linear.x = velocity[1];
        msg.twist.linear.y = velocity[0];
        msg.twist.linear.z = -velocity[2];
    }

    // In ROS REP 103, axis orientation uses the following convention:
    // X - Forward
    // Y - Left
    // Z - Up
    // https://www.ros.org/reps/rep-0103.html#axis-orientation
    // The gyro data is received from AP_AHRS in body-frame
    // X - Forward
    // Y - Right
    // Z - Down
    // As a consequence, to follow ROS REP 103, it is necessary to invert Y and Z
    Vector3f angular_velocity = ahrs.get_gyro();
    msg.twist.angular.x = angular_velocity[0];
    msg.twist.angular.y = -angular_velocity[1];
    msg.twist.angular.z = -angular_velocity[2];
}

void AP_DDS_Client::update_topic(geographic_msgs_msg_GeoPoseStamped& msg)
{
    update_topic(msg.header.stamp);
    strcpy(msg.header.frame_id, BASE_LINK_FRAME_ID);

    auto &ahrs = AP::ahrs();
    WITH_SEMAPHORE(ahrs.get_semaphore());

    Location loc;
    if (ahrs.get_location(loc)) {
        msg.pose.position.latitude = loc.lat * 1E-7;
        msg.pose.position.longitude = loc.lng * 1E-7;
        msg.pose.position.altitude = loc.alt * 0.01; // Transform from cm to m
    }

    // In ROS REP 103, axis orientation uses the following convention:
    // X - Forward
    // Y - Left
    // Z - Up
    // https://www.ros.org/reps/rep-0103.html#axis-orientation
    // As a consequence, to follow ROS REP 103, it is necessary to switch X and Y,
    // as well as invert Z (NED to ENU convertion) as well as a 90 degree rotation in the Z axis
    // for x to point forward
    Quaternion orientation;
    if (ahrs.get_quaternion(orientation)) {
        Quaternion aux(orientation[0], orientation[2], orientation[1], -orientation[3]); //NED to ENU transformation
        Quaternion transformation(sqrtF(2) * 0.5, 0, 0, sqrtF(2) * 0.5); // Z axis 90 degree rotation
        orientation = aux * transformation;
        msg.pose.orientation.w = orientation[0];
        msg.pose.orientation.x = orientation[1];
        msg.pose.orientation.y = orientation[2];
        msg.pose.orientation.z = orientation[3];
    }
}

void AP_DDS_Client::update_topic(rosgraph_msgs_msg_Clock& msg)
{
    update_topic(msg.clock);
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
void AP_DDS_Client::on_topic (uxrSession* uxr_session, uxrObjectId object_id, uint16_t request_id, uxrStreamId stream_id, struct ucdrBuffer* ub, uint16_t length, void* args)
{
    (void) uxr_session; (void) object_id; (void) request_id; (void) stream_id; (void) length;
    /*
    TEMPLATE for reading to the subscribed topics
    1) Store the read contents into the ucdr buffer
    2) Deserialize the said contents into the topic instance
    */

    switch (object_id.id) {
    case topics[to_underlying(TopicIndex::JOY_SUB)].dr_id.id:
        sensor_msgs_msg_Joy topic;
        const bool success = sensor_msgs_msg_Joy_deserialize_topic(ub, &topic);

        if (success == false) {
            break;
        }

        uint32_t* count_ptr = (uint32_t*) args;
        (*count_ptr)++;
        if (topic.axes_size >= 4) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,"Received sensor_msgs/Joy: %f, %f, %f, %f",
                          topic.axes[0], topic.axes[1], topic.axes[2], topic.axes[3]);
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,"Received sensor_msgs/Joy: Insufficient axes size ");
        }
        break;
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

    populate_static_transforms(static_transforms_topic);
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
    uxr_set_topic_callback(&session, AP_DDS_Client::on_topic, &count);

    while (!uxr_create_session(&session)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"DDS Client: Initialization waiting...");
        hal.scheduler->delay(1000);
    }

    // setup reliable stream buffers
    input_reliable_stream = new uint8_t[DDS_STREAM_HISTORY*DDS_BUFFER_SIZE];
    if (input_reliable_stream == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"DDS Client: allocation failed");
        return false;
    }
    output_reliable_stream = new uint8_t[DDS_STREAM_HISTORY*DDS_BUFFER_SIZE];
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

    for (size_t i = 0 ; i < ARRAY_SIZE(topics); i++) {
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
        constexpr uint16_t requestTimeoutMs = (uint8_t) nRequests * maxTimeMsPerRequestMs;
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
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"XRCE Client: Topic/Pub/Writer session request failure for index 'TODO'");
                for (uint8_t s = 0 ; s < nRequests; s++) {
                    GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"XRCE Client: Status '%d' result '%u'", s, status[s]);
                }
                // TODO add a failure log message sharing the status results
                return false;
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO,"XRCE Client: Topic/Pub/Writer session pass for index 'TOOO'");
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
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"XRCE Client: Topic/Sub/Reader session request failure for index '%d'",(int)i);
                for (uint8_t s = 0 ; s < nRequests; s++) {
                    GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"XRCE Client: Status '%d' result '%u'", s, status[s]);
                }
                // TODO add a failure log message sharing the status results
                return false;
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO,"XRCE Client: Topic/Sub/Reader session pass for index '%d'",(int)i);
                uxr_buffer_request_data(&session, reliable_out, topics[i].dr_id, reliable_in, &delivery_control);
            }
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
        const uint32_t topic_size = tf2_msgs_msg_TFMessage_size_of_topic(&static_transforms_topic, 0);
        uxr_prepare_output_stream(&session,reliable_out,topics[2].dw_id,&ub,topic_size);
        const bool success = tf2_msgs_msg_TFMessage_serialize_topic(&ub, &static_transforms_topic);
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

void AP_DDS_Client::write_local_velocity_topic()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        ucdrBuffer ub {};
        const uint32_t topic_size = geometry_msgs_msg_TwistStamped_size_of_topic(&local_velocity_topic, 0);
        uxr_prepare_output_stream(&session,reliable_out,topics[5].dw_id,&ub,topic_size);
        const bool success = geometry_msgs_msg_TwistStamped_serialize_topic(&ub, &local_velocity_topic);
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
        update_topic(local_velocity_topic);
        last_local_velocity_time_ms = cur_time_ms;
        write_local_velocity_topic();
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
#endif // CONFIG_HAL_BOARD

#endif // AP_DDS_ENABLED


