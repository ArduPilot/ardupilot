#include <AP_HAL/AP_HAL_Boards.h>

#include "AP_DDS_config.h"
#if AP_DDS_ENABLED
#include <uxr/client/util/ping.h>

#if AP_DDS_NEEDS_GPS
#include <AP_GPS/AP_GPS.h>
#endif // AP_DDS_NEEDS_GPS
#include <AP_HAL/AP_HAL.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_AHRS/AP_AHRS.h>
#if AP_DDS_ARM_SERVER_ENABLED
#include <AP_Arming/AP_Arming.h>
# endif // AP_DDS_ARM_SERVER_ENABLED
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Common/AP_FWVersion.h>
#include <AP_ExternalControl/AP_ExternalControl_config.h>

#if AP_DDS_ARM_SERVER_ENABLED
#include "ardupilot_msgs/srv/ArmMotors.h"
#endif // AP_DDS_ARM_SERVER_ENABLED
#if AP_DDS_MODE_SWITCH_SERVER_ENABLED
#include "ardupilot_msgs/srv/ModeSwitch.h"
#endif // AP_DDS_MODE_SWITCH_SERVER_ENABLED
#if AP_DDS_ARM_CHECK_SERVER_ENABLED
#include "std_srvs/srv/Trigger.h"
#endif // AP_DDS_ARM_CHECK_SERVER_ENABLED
#if AP_DDS_VTOL_TAKEOFF_SERVER_ENABLED
#include "ardupilot_msgs/srv/Takeoff.h"
#endif // AP_DDS_VTOL_TAKEOFF_SERVER_ENABLED
#if AP_DDS_RC_PUB_ENABLED
#include "AP_RSSI/AP_RSSI.h"
#endif // AP_DDS_RC_PUB_ENABLED

#if AP_EXTERNAL_CONTROL_ENABLED
#include "AP_DDS_ExternalControl.h"
#endif // AP_EXTERNAL_CONTROL_ENABLED
#include "AP_DDS_Frames.h"

#include "AP_DDS_Client.h"
#include "AP_DDS_Topic_Table.h"
#include "AP_DDS_Service_Table.h"
#include "AP_DDS_External_Odom.h"

#define STRCPY(D,S) strncpy(D, S, ARRAY_SIZE(D))

// Enable DDS at runtime by default
static constexpr uint8_t ENABLED_BY_DEFAULT = 1;
#if AP_DDS_TIME_PUB_ENABLED
static constexpr uint16_t DELAY_TIME_TOPIC_MS = AP_DDS_DELAY_TIME_TOPIC_MS;
#endif // AP_DDS_TIME_PUB_ENABLED
#if AP_DDS_BATTERY_STATE_PUB_ENABLED
static constexpr uint16_t DELAY_BATTERY_STATE_TOPIC_MS = AP_DDS_DELAY_BATTERY_STATE_TOPIC_MS;
#endif // AP_DDS_BATTERY_STATE_PUB_ENABLED
#if AP_DDS_IMU_PUB_ENABLED
static constexpr uint16_t DELAY_IMU_TOPIC_MS = AP_DDS_DELAY_IMU_TOPIC_MS;
#endif // AP_DDS_IMU_PUB_ENABLED
#if AP_DDS_LOCAL_POSE_PUB_ENABLED
static constexpr uint16_t DELAY_LOCAL_POSE_TOPIC_MS = AP_DDS_DELAY_LOCAL_POSE_TOPIC_MS;
#endif // AP_DDS_LOCAL_POSE_PUB_ENABLED
#if AP_DDS_LOCAL_VEL_PUB_ENABLED
static constexpr uint16_t DELAY_LOCAL_VELOCITY_TOPIC_MS = AP_DDS_DELAY_LOCAL_VELOCITY_TOPIC_MS;
#endif // AP_DDS_LOCAL_VEL_PUB_ENABLED
#if AP_DDS_AIRSPEED_PUB_ENABLED
static constexpr uint16_t DELAY_AIRSPEED_TOPIC_MS = AP_DDS_DELAY_AIRSPEED_TOPIC_MS;
#endif // AP_DDS_AIRSPEED_PUB_ENABLED
#if AP_DDS_RC_PUB_ENABLED
static constexpr uint16_t DELAY_RC_TOPIC_MS = AP_DDS_DELAY_RC_TOPIC_MS;
#endif // AP_DDS_RC_PUB_ENABLED
#if AP_DDS_GEOPOSE_PUB_ENABLED
static constexpr uint16_t DELAY_GEO_POSE_TOPIC_MS = AP_DDS_DELAY_GEO_POSE_TOPIC_MS;
#endif // AP_DDS_GEOPOSE_PUB_ENABLED
#if AP_DDS_GOAL_PUB_ENABLED
static constexpr uint16_t DELAY_GOAL_TOPIC_MS = AP_DDS_DELAY_GOAL_TOPIC_MS ;
#endif // AP_DDS_GOAL_PUB_ENABLED
#if AP_DDS_CLOCK_PUB_ENABLED
static constexpr uint16_t DELAY_CLOCK_TOPIC_MS =AP_DDS_DELAY_CLOCK_TOPIC_MS;
#endif // AP_DDS_CLOCK_PUB_ENABLED
#if AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED
static constexpr uint16_t DELAY_GPS_GLOBAL_ORIGIN_TOPIC_MS = AP_DDS_DELAY_GPS_GLOBAL_ORIGIN_TOPIC_MS;
#endif // AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED
static constexpr uint16_t DELAY_PING_MS = 500;
#if AP_DDS_STATUS_PUB_ENABLED
static constexpr uint16_t DELAY_STATUS_TOPIC_MS = AP_DDS_DELAY_STATUS_TOPIC_MS;
#endif // AP_DDS_STATUS_PUB_ENABLED

// Define the subscriber data members, which are static class scope.
// If these are created on the stack in the subscriber,
// the AP_DDS_Client::on_topic frame size is exceeded.
#if AP_DDS_JOY_SUB_ENABLED
sensor_msgs_msg_Joy AP_DDS_Client::rx_joy_topic {};
#endif // AP_DDS_JOY_SUB_ENABLED
#if AP_DDS_DYNAMIC_TF_SUB_ENABLED
tf2_msgs_msg_TFMessage AP_DDS_Client::rx_dynamic_transforms_topic {};
#endif // AP_DDS_DYNAMIC_TF_SUB_ENABLED
#if AP_DDS_VEL_CTRL_ENABLED
geometry_msgs_msg_TwistStamped AP_DDS_Client::rx_velocity_control_topic {};
#endif // AP_DDS_VEL_CTRL_ENABLED
#if AP_DDS_GLOBAL_POS_CTRL_ENABLED
ardupilot_msgs_msg_GlobalPosition AP_DDS_Client::rx_global_position_control_topic {};
#endif // AP_DDS_GLOBAL_POS_CTRL_ENABLED

// Define the parameter server data members, which are static class scope.
// If these are created on the stack, then the AP_DDS_Client::on_request
// frame size is exceeded.
#if AP_DDS_PARAMETER_SERVER_ENABLED
rcl_interfaces_srv_SetParameters_Request AP_DDS_Client::set_parameter_request {};
rcl_interfaces_srv_SetParameters_Response AP_DDS_Client::set_parameter_response {};
rcl_interfaces_srv_GetParameters_Request AP_DDS_Client::get_parameters_request {};
rcl_interfaces_srv_GetParameters_Response AP_DDS_Client::get_parameters_response {};
rcl_interfaces_msg_Parameter AP_DDS_Client::param {};
#endif

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
    AP_GROUPINFO("_UDP_PORT", 2, AP_DDS_Client, udp.port, 2019),

    // @Group: _IP
    // @Path: ../AP_Networking/AP_Networking_address.cpp
    AP_SUBGROUPINFO(udp.ip, "_IP", 3,  AP_DDS_Client, AP_Networking_IPV4),

#endif

    // @Param: _DOMAIN_ID
    // @DisplayName: DDS DOMAIN ID
    // @Description: Set the ROS_DOMAIN_ID
    // @Range: 0 232
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("_DOMAIN_ID", 4, AP_DDS_Client, domain_id, 0),

    // @Param: _TIMEOUT_MS
    // @DisplayName: DDS ping timeout
    // @Description: The time in milliseconds the DDS client will wait for a response from the XRCE agent before reattempting.
    // @Units: ms
    // @Range: 1 10000
    // @RebootRequired: True
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_TIMEOUT_MS", 5, AP_DDS_Client, ping_timeout_ms, 1000),

    // @Param: _MAX_RETRY
    // @DisplayName: DDS ping max attempts
    // @Description: The maximum number of times the DDS client will attempt to ping the XRCE agent before exiting. Set to 0 to allow unlimited retries.
    // @Range: 0 100
    // @RebootRequired: True
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_MAX_RETRY", 6, AP_DDS_Client, ping_max_retry, 10),

    AP_GROUPEND
};

#if AP_DDS_STATIC_TF_PUB_ENABLED | AP_DDS_LOCAL_POSE_PUB_ENABLED | AP_DDS_GEOPOSE_PUB_ENABLED | AP_DDS_IMU_PUB_ENABLED
static void initialize(geometry_msgs_msg_Quaternion& q)
{
    q.x = 0.0;
    q.y = 0.0;
    q.z = 0.0;
    q.w = 1.0;
}
#endif // AP_DDS_STATIC_TF_PUB_ENABLED | AP_DDS_LOCAL_POSE_PUB_ENABLED | AP_DDS_GEOPOSE_PUB_ENABLED | AP_DDS_IMU_PUB_ENABLED

AP_DDS_Client::~AP_DDS_Client()
{
    // close transport
    if (is_using_serial) {
        uxr_close_custom_transport(&serial.transport);
    } else {
#if AP_DDS_UDP_ENABLED
        uxr_close_custom_transport(&udp.transport);
#endif
    }
}

#if AP_DDS_TIME_PUB_ENABLED
void AP_DDS_Client::update_topic(builtin_interfaces_msg_Time& msg)
{
    uint64_t utc_usec;
    if (!AP::rtc().get_utc_usec(utc_usec)) {
        utc_usec = AP_HAL::micros64();
    }
    msg.sec = utc_usec / 1000000ULL;
    msg.nanosec = (utc_usec % 1000000ULL) * 1000UL;

}
#endif // AP_DDS_TIME_PUB_ENABLED

#if AP_DDS_NAVSATFIX_PUB_ENABLED
bool AP_DDS_Client::update_topic(sensor_msgs_msg_NavSatFix& msg, const uint8_t instance)
{
    // Add a lambda that takes in navsatfix msg and populates the cov
    // Make it constexpr if possible
    // https://www.fluentcpp.com/2021/12/13/the-evolutions-of-lambdas-in-c14-c17-and-c20/
    // constexpr auto times2 = [] (sensor_msgs_msg_NavSatFix* msg) { return n * 2; };

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
    if (last_nav_sat_fix_time_ms[instance] == last_fix_time_ms) {
        return false;
    } else {
        last_nav_sat_fix_time_ms[instance] = last_fix_time_ms;
    }

    update_topic(msg.header.stamp);
    static_assert(GPS_MAX_RECEIVERS <= 9, "GPS_MAX_RECEIVERS is greater than 9");
    hal.util->snprintf(msg.header.frame_id, 2, "%u", instance);
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
#endif // AP_DDS_NAVSATFIX_PUB_ENABLED

#if AP_DDS_STATIC_TF_PUB_ENABLED
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
        STRCPY(msg.transforms[i].header.frame_id, BASE_LINK_FRAME_ID);
        STRCPY(msg.transforms[i].child_frame_id, gps_frame_id);
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

        // Ensure rotation is initialized.
        initialize(msg.transforms[i].transform.rotation);

        msg.transforms_size++;
    }

}
#endif // AP_DDS_STATIC_TF_PUB_ENABLED

#if AP_DDS_BATTERY_STATE_PUB_ENABLED
void AP_DDS_Client::update_topic(sensor_msgs_msg_BatteryState& msg, const uint8_t instance)
{
    if (instance >= AP_BATT_MONITOR_MAX_INSTANCES) {
        return;
    }
    static_assert(AP_BATT_MONITOR_MAX_INSTANCES <= 99, "AP_BATT_MONITOR_MAX_INSTANCES is greater than 99");

    update_topic(msg.header.stamp);
    hal.util->snprintf(msg.header.frame_id, 2, "%u", instance);

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
        } else if (is_negative(current)) {
            msg.power_supply_status = 1;   //POWER_SUPPLY_STATUS_CHARGING
        } else if (is_positive(current)) {
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
        const auto &cells = battery.get_cell_voltages(instance);
        const uint8_t ncells_max = MIN(ARRAY_SIZE(msg.cell_voltage), ARRAY_SIZE(cells.cells));
        for (uint8_t i=0; i< ncells_max; i++) {
            msg.cell_voltage[i] = cells.cells[i] * 0.001;
        }
    }
}
#endif // AP_DDS_BATTERY_STATE_PUB_ENABLED

#if AP_DDS_LOCAL_POSE_PUB_ENABLED
void AP_DDS_Client::update_topic(geometry_msgs_msg_PoseStamped& msg)
{
    update_topic(msg.header.stamp);
    STRCPY(msg.header.frame_id, BASE_LINK_FRAME_ID);

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
    // as well as invert Z (NED to ENU conversion) as well as a 90 degree rotation in the Z axis
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
    } else {
        initialize(msg.pose.orientation);
    }
}
#endif // AP_DDS_LOCAL_POSE_PUB_ENABLED

#if AP_DDS_LOCAL_VEL_PUB_ENABLED
void AP_DDS_Client::update_topic(geometry_msgs_msg_TwistStamped& msg)
{
    update_topic(msg.header.stamp);
    STRCPY(msg.header.frame_id, BASE_LINK_FRAME_ID);

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
#endif // AP_DDS_LOCAL_VEL_PUB_ENABLED
#if AP_DDS_AIRSPEED_PUB_ENABLED
bool AP_DDS_Client::update_topic(ardupilot_msgs_msg_Airspeed& msg)
{
    update_topic(msg.header.stamp);
    STRCPY(msg.header.frame_id, BASE_LINK_FRAME_ID);
    auto &ahrs = AP::ahrs();
    WITH_SEMAPHORE(ahrs.get_semaphore());
    // In ROS REP 103, axis orientation uses the following convention:
    // X - Forward
    // Y - Left
    // Z - Up
    // https://www.ros.org/reps/rep-0103.html#axis-orientation
    // The true airspeed data is received from AP_AHRS in body-frame
    // X - Forward
    // Y - Right
    // Z - Down
    // As a consequence, to follow ROS REP 103, it is necessary to invert Y and Z
    Vector3f true_airspeed_vec_bf;
    bool is_airspeed_available {false};
    if (ahrs.airspeed_vector_TAS(true_airspeed_vec_bf)) {
        msg.true_airspeed.x = true_airspeed_vec_bf[0];
        msg.true_airspeed.y = -true_airspeed_vec_bf[1];
        msg.true_airspeed.z = -true_airspeed_vec_bf[2];
        msg.eas_2_tas = ahrs.get_EAS2TAS();
        is_airspeed_available = true;
    }
    return is_airspeed_available;
}
#endif // AP_DDS_AIRSPEED_PUB_ENABLED

#if AP_DDS_RC_PUB_ENABLED
bool AP_DDS_Client::update_topic(ardupilot_msgs_msg_Rc& msg)
{
    update_topic(msg.header.stamp);
    AP_RSSI *ap_rssi = AP_RSSI::get_singleton();
    auto rc = RC_Channels::get_singleton();
    static int16_t counter = 0;

    // Is connected if not in failsafe.
    // This is only valid if the RC has been connected at least once
    msg.is_connected = !rc->in_rc_failsafe();
    // Receiver RSSI is reported between 0.0 and 1.0.
    msg.receiver_rssi = static_cast<uint8_t>(ap_rssi->read_receiver_rssi()*100.f);

    // Limit the max number of available channels to 8
    msg.channels_size = MIN(static_cast<uint32_t>(rc->get_valid_channel_count()), 32U);
    msg.active_overrides_size = msg.channels_size;
    if (msg.channels_size) {
        for (uint8_t i = 0; i < static_cast<uint8_t>(msg.channels_size); i++) {
            msg.channels[i] = rc->rc_channel(i)->get_radio_in();
            msg.active_overrides[i] = rc->rc_channel(i)->has_override();
        }
    } else {
        // If no channels are available, the RC is disconnected.
        msg.is_connected = false;
    }

    // Return true if Radio is connected, or once every 10 steps to reduce useless traffic.
    return msg.is_connected ? true : (counter++ % 10 == 0);
}
#endif // AP_DDS_RC_PUB_ENABLED

#if AP_DDS_GEOPOSE_PUB_ENABLED
void AP_DDS_Client::update_topic(geographic_msgs_msg_GeoPoseStamped& msg)
{
    update_topic(msg.header.stamp);
    STRCPY(msg.header.frame_id, BASE_LINK_FRAME_ID);

    auto &ahrs = AP::ahrs();
    WITH_SEMAPHORE(ahrs.get_semaphore());

    Location loc;
    if (ahrs.get_location(loc)) {
        msg.pose.position.latitude = loc.lat * 1E-7;
        msg.pose.position.longitude = loc.lng * 1E-7;
        // TODO this is assumed to be absolute frame in WGS-84 as per the GeoPose message definition in ROS.
        // Use loc.get_alt_frame() to convert if necessary.
        msg.pose.position.altitude = loc.alt * 0.01; // Transform from cm to m
    }

    // In ROS REP 103, axis orientation uses the following convention:
    // X - Forward
    // Y - Left
    // Z - Up
    // https://www.ros.org/reps/rep-0103.html#axis-orientation
    // As a consequence, to follow ROS REP 103, it is necessary to switch X and Y,
    // as well as invert Z (NED to ENU conversion) as well as a 90 degree rotation in the Z axis
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
    } else {
        initialize(msg.pose.orientation);
    }
}
#endif // AP_DDS_GEOPOSE_PUB_ENABLED

#if AP_DDS_GOAL_PUB_ENABLED
bool AP_DDS_Client::update_topic_goal(geographic_msgs_msg_GeoPointStamped& msg)
{
    const auto &vehicle = AP::vehicle();
    update_topic(msg.header.stamp);
    Location target_loc;
    // Exit if no target is available.
    if (!vehicle->get_target_location(target_loc)) {
        return false;
    }
    target_loc.change_alt_frame(Location::AltFrame::ABSOLUTE);
    msg.position.latitude = target_loc.lat * 1e-7;
    msg.position.longitude = target_loc.lng * 1e-7;
    msg.position.altitude = target_loc.alt * 1e-2;

    // Check whether the goal has changed or if the topic has never been published.
    const double tolerance_lat_lon = 1e-8; // One order of magnitude smaller than the target's resolution.
    const double distance_alt = 1e-3;
    if (abs(msg.position.latitude - prev_goal_msg.position.latitude) >  tolerance_lat_lon ||
        abs(msg.position.longitude - prev_goal_msg.position.longitude) >  tolerance_lat_lon ||
        abs(msg.position.altitude - prev_goal_msg.position.altitude) > distance_alt ||
        prev_goal_msg.header.stamp.sec == 0 ) {
        update_topic(prev_goal_msg.header.stamp);
        prev_goal_msg.position.latitude = msg.position.latitude;
        prev_goal_msg.position.longitude = msg.position.longitude;
        prev_goal_msg.position.altitude = msg.position.altitude;
        return true;
    } else {
        return false;
    }
}
#endif // AP_DDS_GOAL_PUB_ENABLED

#if AP_DDS_IMU_PUB_ENABLED
void AP_DDS_Client::update_topic(sensor_msgs_msg_Imu& msg)
{
    update_topic(msg.header.stamp);
    STRCPY(msg.header.frame_id, BASE_LINK_NED_FRAME_ID);

    auto &imu = AP::ins();
    auto &ahrs = AP::ahrs();

    WITH_SEMAPHORE(ahrs.get_semaphore());

    Quaternion orientation;
    if (ahrs.get_quaternion(orientation)) {
        msg.orientation.x = orientation[0];
        msg.orientation.y = orientation[1];
        msg.orientation.z = orientation[2];
        msg.orientation.w = orientation[3];
    } else {
        initialize(msg.orientation);
    }

    uint8_t accel_index = ahrs.get_primary_accel_index();
    uint8_t gyro_index = ahrs.get_primary_gyro_index();
    const Vector3f accel_data = imu.get_accel(accel_index);
    const Vector3f gyro_data = imu.get_gyro(gyro_index);

    // Populate the message fields
    msg.linear_acceleration.x = accel_data.x;
    msg.linear_acceleration.y = accel_data.y;
    msg.linear_acceleration.z = accel_data.z;

    msg.angular_velocity.x = gyro_data.x;
    msg.angular_velocity.y = gyro_data.y;
    msg.angular_velocity.z = gyro_data.z;
}
#endif // AP_DDS_IMU_PUB_ENABLED

#if AP_DDS_CLOCK_PUB_ENABLED
void AP_DDS_Client::update_topic(rosgraph_msgs_msg_Clock& msg)
{
    update_topic(msg.clock);
}
#endif // AP_DDS_CLOCK_PUB_ENABLED

#if AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED
void AP_DDS_Client::update_topic(geographic_msgs_msg_GeoPointStamped& msg)
{
    update_topic(msg.header.stamp);
    STRCPY(msg.header.frame_id, BASE_LINK_FRAME_ID);

    auto &ahrs = AP::ahrs();
    WITH_SEMAPHORE(ahrs.get_semaphore());

    Location ekf_origin;
    // LLA is WGS-84 geodetic coordinate.
    // Altitude converted from cm to m.
    if (ahrs.get_origin(ekf_origin)) {
        msg.position.latitude = ekf_origin.lat * 1E-7;
        msg.position.longitude = ekf_origin.lng * 1E-7;
        msg.position.altitude = ekf_origin.alt * 0.01;
    }
}
#endif // AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED

#if AP_DDS_STATUS_PUB_ENABLED
bool AP_DDS_Client::update_topic(ardupilot_msgs_msg_Status& msg)
{
    // Fill the new message.
    const auto &vehicle = AP::vehicle();
    const auto &battery = AP::battery();
    msg.vehicle_type = static_cast<uint8_t>(AP::fwversion().vehicle_type);
    msg.armed = hal.util->get_soft_armed();
    msg.mode = vehicle->get_mode();
    msg.flying = vehicle->get_likely_flying();
    msg.external_control = true; // Always true for now. To be filled after PR#28429.
    uint8_t fs_iter = 0;
    msg.failsafe_size = 0;
    if (rc().in_rc_failsafe()) {
        msg.failsafe[fs_iter++] = Status::FS_RADIO;
    }
    if (battery.has_failsafed()) {
        msg.failsafe[fs_iter++] = Status::FS_BATTERY;
    }
    // TODO: replace flag with function.
    if (AP_Notify::flags.failsafe_gcs) {
        msg.failsafe[fs_iter++] = Status::FS_GCS;
    }
    // TODO: replace flag with function.
    if (AP_Notify::flags.failsafe_ekf) {
        msg.failsafe[fs_iter++] = Status::FS_EKF;
    }
    msg.failsafe_size = fs_iter;

    // Compare with the previous one.
    bool is_message_changed {false};
    is_message_changed |= (last_status_msg_.flying != msg.flying);
    is_message_changed |= (last_status_msg_.armed != msg.armed);
    is_message_changed |= (last_status_msg_.mode != msg.mode);
    is_message_changed |= (last_status_msg_.vehicle_type != msg.vehicle_type);
    is_message_changed |= (last_status_msg_.failsafe_size != msg.failsafe_size);
    is_message_changed |= (last_status_msg_.external_control != msg.external_control);

    const auto timestamp = AP_HAL::millis64();
    if ( is_message_changed ) {
        last_status_msg_.flying = msg.flying;
        last_status_msg_.armed  = msg.armed;
        last_status_msg_.mode  = msg.mode;
        last_status_msg_.vehicle_type = msg.vehicle_type;
        last_status_msg_.failsafe_size = msg.failsafe_size;
        last_status_msg_.external_control = msg.external_control;
        last_status_publish_time_ms = timestamp;
        update_topic(msg.header.stamp);
        return true;
    } else if (timestamp - last_status_publish_time_ms > DELAY_STATUS_TOPIC_MS * 5) {
        // Publish the status message at 2Hz even if no change is detected.
        last_status_publish_time_ms = timestamp;
        update_topic(msg.header.stamp);
        return true;
    } else {
        return false;
    }
}
#endif // AP_DDS_STATUS_PUB_ENABLED
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
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s Thread create failed", msg_prefix);
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
#if AP_DDS_JOY_SUB_ENABLED
    case topics[to_underlying(TopicIndex::JOY_SUB)].dr_id.id: {
        const bool success = sensor_msgs_msg_Joy_deserialize_topic(ub, &rx_joy_topic);

        if (success == false) {
            break;
        }

        if (rx_joy_topic.axes_size >= 4) {
            const uint32_t t_now = AP_HAL::millis();

            for (uint8_t i = 0; i < MIN(8U, rx_joy_topic.axes_size); i++) {
                // Ignore channel override if NaN.
                if (std::isnan(rx_joy_topic.axes[i])) {
                    // Setting the RC override to 0U releases the channel back to the RC.
                    RC_Channels::set_override(i, 0U, t_now);
                } else {
                    const uint16_t mapped_data = static_cast<uint16_t>(
                                                     linear_interpolate(rc().channel(i)->get_radio_min(),
                                                             rc().channel(i)->get_radio_max(),
                                                             rx_joy_topic.axes[i],
                                                             -1.0, 1.0));
                    RC_Channels::set_override(i, mapped_data, t_now);
                }
            }

        }
        break;
    }
#endif // AP_DDS_JOY_SUB_ENABLED
#if AP_DDS_DYNAMIC_TF_SUB_ENABLED
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
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s Received tf2_msgs/TFMessage: TF is empty", msg_prefix);
        }
        break;
    }
#endif // AP_DDS_DYNAMIC_TF_SUB_ENABLED
#if AP_DDS_VEL_CTRL_ENABLED
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
#endif // AP_DDS_VEL_CTRL_ENABLED
#if AP_DDS_GLOBAL_POS_CTRL_ENABLED
    case topics[to_underlying(TopicIndex::GLOBAL_POSITION_SUB)].dr_id.id: {
        const bool success = ardupilot_msgs_msg_GlobalPosition_deserialize_topic(ub, &rx_global_position_control_topic);
        if (success == false) {
            break;
        }

#if AP_EXTERNAL_CONTROL_ENABLED
        if (!AP_DDS_External_Control::handle_global_position_control(rx_global_position_control_topic)) {
            // TODO #23430 handle global position control failure through rosout, throttled.
        }
#endif // AP_EXTERNAL_CONTROL_ENABLED
        break;
    }
#endif // AP_DDS_GLOBAL_POS_CTRL_ENABLED
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
#if AP_DDS_ARM_SERVER_ENABLED
    case services[to_underlying(ServiceIndex::ARMING_MOTORS)].rep_id: {
        ardupilot_msgs_srv_ArmMotors_Request arm_motors_request;
        ardupilot_msgs_srv_ArmMotors_Response arm_motors_response;
        const bool deserialize_success = ardupilot_msgs_srv_ArmMotors_Request_deserialize_topic(ub, &arm_motors_request);
        if (deserialize_success == false) {
            break;
        }

        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s Request for %sing received", msg_prefix, arm_motors_request.arm ? "arm" : "disarm");
#if AP_EXTERNAL_CONTROL_ENABLED
        const bool do_checks = true;
        arm_motors_response.result = arm_motors_request.arm ? AP_DDS_External_Control::arm(AP_Arming::Method::DDS, do_checks) : AP_DDS_External_Control::disarm(AP_Arming::Method::DDS, do_checks);
        if (!arm_motors_response.result) {
            // TODO #23430 handle arm failure through rosout, throttled.
        }
#endif // AP_EXTERNAL_CONTROL_ENABLED

        const uxrObjectId replier_id = {
            .id = services[to_underlying(ServiceIndex::ARMING_MOTORS)].rep_id,
            .type = UXR_REPLIER_ID
        };

        uint8_t reply_buffer[8] {};
        ucdrBuffer reply_ub;

        ucdr_init_buffer(&reply_ub, reply_buffer, sizeof(reply_buffer));
        const bool serialize_success = ardupilot_msgs_srv_ArmMotors_Response_serialize_topic(&reply_ub, &arm_motors_response);
        if (serialize_success == false) {
            break;
        }

        uxr_buffer_reply(uxr_session, reliable_out, replier_id, sample_id, reply_buffer, ucdr_buffer_length(&reply_ub));
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s Request for Arming/Disarming : %s", msg_prefix, arm_motors_response.result ? "SUCCESS" : "FAIL");
        break;
    }
#endif // AP_DDS_ARM_SERVER_ENABLED
#if AP_DDS_MODE_SWITCH_SERVER_ENABLED
    case services[to_underlying(ServiceIndex::MODE_SWITCH)].rep_id: {
        ardupilot_msgs_srv_ModeSwitch_Request mode_switch_request;
        ardupilot_msgs_srv_ModeSwitch_Response mode_switch_response;
        const bool deserialize_success = ardupilot_msgs_srv_ModeSwitch_Request_deserialize_topic(ub, &mode_switch_request);
        if (deserialize_success == false) {
            break;
        }
        mode_switch_response.status = AP::vehicle()->set_mode(mode_switch_request.mode, ModeReason::DDS_COMMAND);
        mode_switch_response.curr_mode = AP::vehicle()->get_mode();

        const uxrObjectId replier_id = {
            .id = services[to_underlying(ServiceIndex::MODE_SWITCH)].rep_id,
            .type = UXR_REPLIER_ID
        };

        uint8_t reply_buffer[8] {};
        ucdrBuffer reply_ub;

        ucdr_init_buffer(&reply_ub, reply_buffer, sizeof(reply_buffer));
        const bool serialize_success = ardupilot_msgs_srv_ModeSwitch_Response_serialize_topic(&reply_ub, &mode_switch_response);
        if (serialize_success == false) {
            break;
        }

        uxr_buffer_reply(uxr_session, reliable_out, replier_id, sample_id, reply_buffer, ucdr_buffer_length(&reply_ub));
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s Request for Mode Switch : %s", msg_prefix, mode_switch_response.status ? "SUCCESS" : "FAIL");
        break;
    }
#endif // AP_DDS_MODE_SWITCH_SERVER_ENABLED
#if AP_DDS_VTOL_TAKEOFF_SERVER_ENABLED
    case services[to_underlying(ServiceIndex::TAKEOFF)].rep_id: {
        ardupilot_msgs_srv_Takeoff_Request takeoff_request;
        ardupilot_msgs_srv_Takeoff_Response takeoff_response;
        const bool deserialize_success = ardupilot_msgs_srv_Takeoff_Request_deserialize_topic(ub, &takeoff_request);
        if (deserialize_success == false) {
            break;
        }
        takeoff_response.status = AP::vehicle()->start_takeoff(takeoff_request.alt);

        const uxrObjectId replier_id = {
            .id = services[to_underlying(ServiceIndex::TAKEOFF)].rep_id,
            .type = UXR_REPLIER_ID
        };

        uint8_t reply_buffer[8] {};
        ucdrBuffer reply_ub;

        ucdr_init_buffer(&reply_ub, reply_buffer, sizeof(reply_buffer));
        const bool serialize_success = ardupilot_msgs_srv_Takeoff_Response_serialize_topic(&reply_ub, &takeoff_response);
        if (serialize_success == false) {
            break;
        }

        uxr_buffer_reply(uxr_session, reliable_out, replier_id, sample_id, reply_buffer, ucdr_buffer_length(&reply_ub));
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s Request for Takeoff : %s", msg_prefix, takeoff_response.status ? "SUCCESS" : "FAIL");
        break;
    }
#endif // AP_DDS_VTOL_TAKEOFF_SERVER_ENABLED
#if AP_DDS_ARM_CHECK_SERVER_ENABLED
    case services[to_underlying(ServiceIndex::PREARM_CHECK)].rep_id: {
        std_srvs_srv_Trigger_Request prearm_check_request;
        std_srvs_srv_Trigger_Response prearm_check_response;
        const bool deserialize_success = std_srvs_srv_Trigger_Request_deserialize_topic(ub, &prearm_check_request);
        if (deserialize_success == false) {
            break;
        }
        prearm_check_response.success = AP::arming().pre_arm_checks(false);
        STRCPY(prearm_check_response.message, prearm_check_response.success ? "Vehicle is Armable" : "Vehicle is Not Armable");

        const uxrObjectId replier_id = {
            .id = services[to_underlying(ServiceIndex::PREARM_CHECK)].rep_id,
            .type = UXR_REPLIER_ID
        };

        uint8_t reply_buffer[sizeof(prearm_check_response.message) + 1] {};
        ucdrBuffer reply_ub;

        ucdr_init_buffer(&reply_ub, reply_buffer, sizeof(reply_buffer));
        const bool serialize_success = std_srvs_srv_Trigger_Response_serialize_topic(&reply_ub, &prearm_check_response);
        if (serialize_success == false) {
            break;
        }

        uxr_buffer_reply(uxr_session, reliable_out, replier_id, sample_id, reply_buffer, ucdr_buffer_length(&reply_ub));
        break;
    }
#endif //AP_DDS_ARM_CHECK_SERVER_ENABLED
#if AP_DDS_PARAMETER_SERVER_ENABLED
    case services[to_underlying(ServiceIndex::SET_PARAMETERS)].rep_id: {
        const bool deserialize_success = rcl_interfaces_srv_SetParameters_Request_deserialize_topic(ub, &set_parameter_request);
        if (deserialize_success == false) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s Set Parameters Request : Failed to deserialize request.", msg_prefix);
            break;
        }

        if (set_parameter_request.parameters_size > 8U) {
            break;
        }

        // Set parameters and responses for each one requested
        set_parameter_response.results_size = set_parameter_request.parameters_size;
        for (size_t i = 0; i < set_parameter_request.parameters_size; i++) {
            param = set_parameter_request.parameters[i];

            enum ap_var_type var_type;

            // set parameter
            AP_Param *vp;
            char param_key[AP_MAX_NAME_SIZE + 1];
            strncpy(param_key, (char *)param.name, AP_MAX_NAME_SIZE);
            param_key[AP_MAX_NAME_SIZE] = 0;

            // Currently only integer and double value types can be set.
            // The following parameter value types are not handled:
            // bool, string, byte_array, bool_array, integer_array, double_array and string_array
            bool param_isnan = true;
            bool param_isinf = true;
            float param_value = 0.0f;
            switch (param.value.type) {
            case ParameterType::PARAMETER_INTEGER: {
                param_isnan = isnan(param.value.integer_value);
                param_isinf = isinf(param.value.integer_value);
                param_value = float(param.value.integer_value);
                break;
            }
            case ParameterType::PARAMETER_DOUBLE: {
                param_isnan = isnan(param.value.double_value);
                param_isinf = isinf(param.value.double_value);
                param_value = float(param.value.double_value);
                break;
            }
            default: {
                break;
            }
            }

            // find existing param to get the old value
            uint16_t parameter_flags = 0;
            vp = AP_Param::find(param_key, &var_type, &parameter_flags);
            if (vp == nullptr || param_isnan || param_isinf) {
                set_parameter_response.results[i].successful = false;
                strncpy(set_parameter_response.results[i].reason, "Parameter not found", sizeof(set_parameter_response.results[i].reason));
                continue;
            }

            // Add existing parameter checks used in GCS_Param.cpp
            if (parameter_flags & AP_PARAM_FLAG_INTERNAL_USE_ONLY) {
                // The user can set BRD_OPTIONS to enable set of internal
                // parameters, for developer testing or unusual use cases
                if (AP_BoardConfig::allow_set_internal_parameters()) {
                    parameter_flags &= ~AP_PARAM_FLAG_INTERNAL_USE_ONLY;
                }
            }

            if ((parameter_flags & AP_PARAM_FLAG_INTERNAL_USE_ONLY) || vp->is_read_only()) {
                set_parameter_response.results[i].successful = false;
                strncpy(set_parameter_response.results[i].reason, "Parameter is read only",sizeof(set_parameter_response.results[i].reason));
                continue;
            }

            // Set and save the value if it changed
            bool force_save = vp->set_and_save_by_name_ifchanged(param_key, param_value);

            if (force_save && (parameter_flags & AP_PARAM_FLAG_ENABLE)) {
                AP_Param::invalidate_count();
            }

            set_parameter_response.results[i].successful = true;
            strncpy(set_parameter_response.results[i].reason, "Parameter accepted", sizeof(set_parameter_response.results[i].reason));
        }

        const uxrObjectId replier_id = {
            .id = services[to_underlying(ServiceIndex::SET_PARAMETERS)].rep_id,
            .type = UXR_REPLIER_ID
        };

        const uint32_t reply_size = rcl_interfaces_srv_SetParameters_Response_size_of_topic(&set_parameter_response, 0U);
        uint8_t reply_buffer[reply_size];
        memset(reply_buffer, 0, reply_size * sizeof(uint8_t));
        ucdrBuffer reply_ub;

        ucdr_init_buffer(&reply_ub, reply_buffer, reply_size);
        const bool serialize_success = rcl_interfaces_srv_SetParameters_Response_serialize_topic(&reply_ub, &set_parameter_response);
        if (serialize_success == false) {
            break;
        }

        uxr_buffer_reply(uxr_session, reliable_out, replier_id, sample_id, reply_buffer, ucdr_buffer_length(&reply_ub));
        bool successful_params = true;
        for (size_t i = 0; i < set_parameter_response.results_size; i++) {
            // Check that all the parameters were set successfully
            successful_params &= set_parameter_response.results[i].successful;
        }
        GCS_SEND_TEXT(successful_params ? MAV_SEVERITY_INFO : MAV_SEVERITY_WARNING, "%s Set Parameters Request : %s", msg_prefix, successful_params ? "SUCCESSFUL" : "ONE OR MORE PARAMS FAILED" );
        break;
    }
    case services[to_underlying(ServiceIndex::GET_PARAMETERS)].rep_id: {
        const bool deserialize_success = rcl_interfaces_srv_GetParameters_Request_deserialize_topic(ub, &get_parameters_request);
        if (deserialize_success == false) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s Get Parameters Request : Failed to deserialize request.", msg_prefix);
            break;
        }

        if (get_parameters_request.names_size > 8U) {
            break;
        }

        bool successful_read = true;
        get_parameters_response.values_size = get_parameters_request.names_size;
        for (size_t i = 0; i < get_parameters_request.names_size; i++) {
            enum ap_var_type var_type;

            AP_Param *vp;
            char param_key[AP_MAX_NAME_SIZE + 1];
            strncpy(param_key, (char *)get_parameters_request.names[i], AP_MAX_NAME_SIZE);
            param_key[AP_MAX_NAME_SIZE] = 0;

            vp = AP_Param::find(param_key, &var_type);
            if (vp == nullptr) {
                get_parameters_response.values[i].type = ParameterType::PARAMETER_NOT_SET;
                successful_read &= false;
                continue;
            }

            switch (var_type) {
            case AP_PARAM_INT8: {
                get_parameters_response.values[i].type = ParameterType::PARAMETER_INTEGER;
                get_parameters_response.values[i].integer_value = ((AP_Int8 *)vp)->get();
                successful_read &= true;
                break;
            }
            case AP_PARAM_INT16: {
                get_parameters_response.values[i].type = ParameterType::PARAMETER_INTEGER;
                get_parameters_response.values[i].integer_value = ((AP_Int16 *)vp)->get();
                successful_read &= true;
                break;
            }
            case AP_PARAM_INT32: {
                get_parameters_response.values[i].type = ParameterType::PARAMETER_INTEGER;
                get_parameters_response.values[i].integer_value = ((AP_Int32 *)vp)->get();
                successful_read &= true;
                break;
            }
            case AP_PARAM_FLOAT: {
                get_parameters_response.values[i].type = ParameterType::PARAMETER_DOUBLE;
                get_parameters_response.values[i].double_value = vp->cast_to_float(var_type);
                successful_read &= true;
                break;
            }
            default: {
                get_parameters_response.values[i].type = ParameterType::PARAMETER_NOT_SET;
                successful_read &= false;
                break;
            }
            }
        }

        const uxrObjectId replier_id = {
            .id = services[to_underlying(ServiceIndex::GET_PARAMETERS)].rep_id,
            .type = UXR_REPLIER_ID
        };

        const uint32_t reply_size = rcl_interfaces_srv_GetParameters_Response_size_of_topic(&get_parameters_response, 0U);
        uint8_t reply_buffer[reply_size];
        memset(reply_buffer, 0, reply_size * sizeof(uint8_t));
        ucdrBuffer reply_ub;

        ucdr_init_buffer(&reply_ub, reply_buffer, reply_size);
        const bool serialize_success = rcl_interfaces_srv_GetParameters_Response_serialize_topic(&reply_ub, &get_parameters_response);
        if (serialize_success == false) {
            break;
        }

        uxr_buffer_reply(uxr_session, reliable_out, replier_id, sample_id, reply_buffer, ucdr_buffer_length(&reply_ub));

        GCS_SEND_TEXT(successful_read ? MAV_SEVERITY_INFO : MAV_SEVERITY_WARNING, "%s Get Parameters Request : %s", msg_prefix, successful_read ? "SUCCESSFUL" : "ONE OR MORE PARAM NOT FOUND");
        break;
    }
#endif // AP_DDS_PARAMETER_SERVER_ENABLED
    }
}

/*
  main loop for DDS thread
 */
void AP_DDS_Client::main_loop(void)
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s initializing...", msg_prefix);
    if (!init_transport()) {
        return;
    }

    //! @todo check for request to stop task
    while (true) {
        if (comm == nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s transport invalid, exiting", msg_prefix);
            return;
        }
        // If using UDP, check if the network is active before proceeding
        // not applicable for SITL, which doesn't use AP_Networking
#if AP_DDS_UDP_ENABLED && !AP_NETWORKING_BACKEND_SITL
        if (!is_using_serial) {
            const auto &network = AP::network();
            if (network.get_ip_active() == 0) {
                hal.scheduler->delay(1000);
                continue;
            }

        }
#endif

        // check ping
        if (ping_max_retry == 0) {
            if (!uxr_ping_agent(comm, ping_timeout_ms)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s No ping response, retrying", msg_prefix);
                continue;
            }
        } else {
            if (!uxr_ping_agent_attempts(comm, ping_timeout_ms, ping_max_retry)) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%s No ping response, exiting", msg_prefix);
                continue;
            }
        }

        // create session
        if (!init_session() || !create()) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s Creation Requests failed", msg_prefix);
            return;
        }
        connected = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s Initialization passed", msg_prefix);

#if AP_DDS_STATIC_TF_PUB_ENABLED
        populate_static_transforms(tx_static_transforms_topic);
        write_static_transforms();
#endif // AP_DDS_STATIC_TF_PUB_ENABLED

        uint64_t last_ping_ms{0};
        uint8_t num_pings_missed{0};
        bool had_ping_reply{false};
        while (connected) {
            hal.scheduler->delay(1);

            // publish topics
            update();

            // check ping response
            if (session.on_pong_flag == PONG_IN_SESSION_STATUS) {
                had_ping_reply = true;
            }

            const auto cur_time_ms = AP_HAL::millis64();
            if (cur_time_ms - last_ping_ms > DELAY_PING_MS) {
                last_ping_ms = cur_time_ms;

                if (had_ping_reply) {
                    num_pings_missed = 0;

                } else {
                    ++num_pings_missed;
                }

                const int ping_agent_timeout_ms{0};
                const uint8_t ping_agent_attempts{1};
                uxr_ping_agent_session(&session, ping_agent_timeout_ms, ping_agent_attempts);

                had_ping_reply = false;
            }

            if (num_pings_missed > 2) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
                              "%s No ping response, disconnecting", msg_prefix);
                connected = false;
            }
        }

        // delete session if connected
        if (connected) {
            uxr_delete_session(&session);
        }
    }
}

bool AP_DDS_Client::init_transport()
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
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s Transport initialization failed", msg_prefix);
        return false;
    }

    return true;
}

bool AP_DDS_Client::init_session()
{
    // init session
    uxr_init_session(&session, comm, key);

    // Register topic callbacks
    uxr_set_topic_callback(&session, AP_DDS_Client::on_topic_trampoline, this);

    // ROS-2 Service : Register service request callbacks
    uxr_set_request_callback(&session, AP_DDS_Client::on_request_trampoline, this);

    while (!uxr_create_session(&session)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s Initialization waiting...", msg_prefix);
        hal.scheduler->delay(1000);
    }

    // setup reliable stream buffers
    input_reliable_stream = NEW_NOTHROW uint8_t[DDS_BUFFER_SIZE];
    output_reliable_stream = NEW_NOTHROW uint8_t[DDS_BUFFER_SIZE];
    if (input_reliable_stream == nullptr || output_reliable_stream == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s Allocation failed", msg_prefix);
        return false;
    }

    reliable_in = uxr_create_input_reliable_stream(&session, input_reliable_stream, DDS_BUFFER_SIZE, DDS_STREAM_HISTORY);
    reliable_out = uxr_create_output_reliable_stream(&session, output_reliable_stream, DDS_BUFFER_SIZE, DDS_STREAM_HISTORY);

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s Init complete", msg_prefix);

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
    const char* participant_name = AP_DDS_PARTICIPANT_NAME;
    const auto participant_req_id = uxr_buffer_create_participant_bin(&session, reliable_out, participant_id,
                                    static_cast<uint16_t>(domain_id), participant_name, UXR_REPLACE);

    //Participant requests
    constexpr uint8_t nRequestsParticipant = 1;
    const uint16_t requestsParticipant[nRequestsParticipant] = {participant_req_id};

    constexpr uint16_t maxTimeMsPerRequestMs = 500;
    constexpr uint16_t requestTimeoutParticipantMs = (uint16_t) nRequestsParticipant * maxTimeMsPerRequestMs;
    uint8_t statusParticipant[nRequestsParticipant];
    if (!uxr_run_session_until_all_status(&session, requestTimeoutParticipantMs, requestsParticipant, statusParticipant, nRequestsParticipant)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s Participant session request failure", msg_prefix);
        // TODO add a failure log message sharing the status results
        return false;
    }

    for (uint16_t i = 0 ; i < ARRAY_SIZE(topics); i++) {
        // Topic
        const uxrObjectId topic_id = {
            .id = topics[i].topic_id,
            .type = UXR_TOPIC_ID
        };
        const auto topic_req_id = uxr_buffer_create_topic_bin(&session, reliable_out, topic_id,
                                  participant_id, topics[i].topic_name, topics[i].type_name, UXR_REPLACE);

        // Status requests
        constexpr uint8_t nRequests = 3;
        uint16_t requests[nRequests];
        constexpr uint16_t requestTimeoutMs = nRequests * maxTimeMsPerRequestMs;
        uint8_t status[nRequests];

        if (topics[i].topic_rw == Topic_rw::DataWriter) {
            // Publisher
            const uxrObjectId pub_id = {
                .id = topics[i].pub_id,
                .type = UXR_PUBLISHER_ID
            };
            const auto pub_req_id = uxr_buffer_create_publisher_bin(&session, reliable_out, pub_id,
                                    participant_id, UXR_REPLACE);

            // Data Writer
            const auto dwriter_req_id = uxr_buffer_create_datawriter_bin(&session, reliable_out, topics[i].dw_id,
                                        pub_id, topic_id, topics[i].qos, UXR_REPLACE);

            // save the request statuses
            requests[0] = topic_req_id;
            requests[1] = pub_req_id;
            requests[2] = dwriter_req_id;

            if (!uxr_run_session_until_all_status(&session, requestTimeoutMs, requests, status, nRequests)) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s Topic/Pub/Writer session request failure for index '%u'", msg_prefix, i);
                for (uint8_t s = 0 ; s < nRequests; s++) {
                    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s Status '%d' result '%u'", msg_prefix, s, status[s]);
                }
                // TODO add a failure log message sharing the status results
                return false;
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s Topic/Pub/Writer session pass for index '%u'", msg_prefix, i);
            }
        } else if (topics[i].topic_rw == Topic_rw::DataReader) {
            // Subscriber
            const uxrObjectId sub_id = {
                .id = topics[i].sub_id,
                .type = UXR_SUBSCRIBER_ID
            };
            const auto sub_req_id = uxr_buffer_create_subscriber_bin(&session, reliable_out, sub_id,
                                    participant_id, UXR_REPLACE);

            // Data Reader
            const auto dreader_req_id = uxr_buffer_create_datareader_bin(&session, reliable_out, topics[i].dr_id,
                                        sub_id, topic_id, topics[i].qos, UXR_REPLACE);

            // save the request statuses
            requests[0] = topic_req_id;
            requests[1] = sub_req_id;
            requests[2] = dreader_req_id;

            if (!uxr_run_session_until_all_status(&session, requestTimeoutMs, requests, status, nRequests)) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s Topic/Sub/Reader session request failure for index '%u'", msg_prefix, i);
                for (uint8_t s = 0 ; s < nRequests; s++) {
                    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s Status '%d' result '%u'", msg_prefix, s, status[s]);
                }
                // TODO add a failure log message sharing the status results
                return false;
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s Topic/Sub/Reader session pass for index '%u'", msg_prefix, i);
                uxr_buffer_request_data(&session, reliable_out, topics[i].dr_id, reliable_in, &delivery_control);
            }
        }
    }

    // ROS-2 Service : else case for service requests

    for (uint16_t i = 0; i < ARRAY_SIZE(services); i++) {

        constexpr uint16_t requestTimeoutMs = maxTimeMsPerRequestMs;

        if (services[i].service_rr == Service_rr::Replier) {
            const uxrObjectId rep_id = {
                .id = services[i].rep_id,
                .type = UXR_REPLIER_ID
            };
            const auto replier_req_id = uxr_buffer_create_replier_bin(&session, reliable_out, rep_id,
                                        participant_id, services[i].service_name, services[i].request_type, services[i].reply_type,
                                        services[i].request_topic_name, services[i].reply_topic_name, services[i].qos, UXR_REPLACE);

            uint16_t request = replier_req_id;
            uint8_t status;

            if (!uxr_run_session_until_all_status(&session, requestTimeoutMs, &request, &status, 1)) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s Service/Replier session request failure for index '%u'", msg_prefix, i);
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s Status result '%u'", msg_prefix, status);
                // TODO add a failure log message sharing the status results
                return false;
            } else {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s Service/Replier session pass for index '%u'", msg_prefix, i);
                uxr_buffer_request_data(&session, reliable_out, rep_id, reliable_in, &delivery_control);
            }

        } else if (services[i].service_rr == Service_rr::Requester) {
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
        uxr_prepare_output_stream(&session, reliable_out, topics[to_underlying(TopicIndex::TIME_PUB)].dw_id, &ub, topic_size);
        const bool success = builtin_interfaces_msg_Time_serialize_topic(&ub, &time_topic);
        if (!success) {
            // TODO sometimes serialization fails on bootup. Determine why.
            // AP_HAL::panic("FATAL: XRCE_Client failed to serialize");
        }
    }
}

#if AP_DDS_NAVSATFIX_PUB_ENABLED
void AP_DDS_Client::write_nav_sat_fix_topic()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        ucdrBuffer ub {};
        const uint32_t topic_size = sensor_msgs_msg_NavSatFix_size_of_topic(&nav_sat_fix_topic, 0);
        uxr_prepare_output_stream(&session, reliable_out, topics[to_underlying(TopicIndex::NAV_SAT_FIX_PUB)].dw_id, &ub, topic_size);
        const bool success = sensor_msgs_msg_NavSatFix_serialize_topic(&ub, &nav_sat_fix_topic);
        if (!success) {
            // TODO sometimes serialization fails on bootup. Determine why.
            // AP_HAL::panic("FATAL: DDS_Client failed to serialize");
        }
    }
}
#endif // AP_DDS_NAVSATFIX_PUB_ENABLED

#if AP_DDS_STATIC_TF_PUB_ENABLED
void AP_DDS_Client::write_static_transforms()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        ucdrBuffer ub {};
        const uint32_t topic_size = tf2_msgs_msg_TFMessage_size_of_topic(&tx_static_transforms_topic, 0);
        uxr_prepare_output_stream(&session, reliable_out, topics[to_underlying(TopicIndex::STATIC_TRANSFORMS_PUB)].dw_id, &ub, topic_size);
        const bool success = tf2_msgs_msg_TFMessage_serialize_topic(&ub, &tx_static_transforms_topic);
        if (!success) {
            // TODO sometimes serialization fails on bootup. Determine why.
            // AP_HAL::panic("FATAL: DDS_Client failed to serialize");
        }
    }
}
#endif // AP_DDS_STATIC_TF_PUB_ENABLED

#if AP_DDS_BATTERY_STATE_PUB_ENABLED
void AP_DDS_Client::write_battery_state_topic()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        ucdrBuffer ub {};
        const uint32_t topic_size = sensor_msgs_msg_BatteryState_size_of_topic(&battery_state_topic, 0);
        uxr_prepare_output_stream(&session, reliable_out, topics[to_underlying(TopicIndex::BATTERY_STATE_PUB)].dw_id, &ub, topic_size);
        const bool success = sensor_msgs_msg_BatteryState_serialize_topic(&ub, &battery_state_topic);
        if (!success) {
            // TODO sometimes serialization fails on bootup. Determine why.
            // AP_HAL::panic("FATAL: DDS_Client failed to serialize");
        }
    }
}
#endif // AP_DDS_BATTERY_STATE_PUB_ENABLED

#if AP_DDS_LOCAL_POSE_PUB_ENABLED
void AP_DDS_Client::write_local_pose_topic()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        ucdrBuffer ub {};
        const uint32_t topic_size = geometry_msgs_msg_PoseStamped_size_of_topic(&local_pose_topic, 0);
        uxr_prepare_output_stream(&session, reliable_out, topics[to_underlying(TopicIndex::LOCAL_POSE_PUB)].dw_id, &ub, topic_size);
        const bool success = geometry_msgs_msg_PoseStamped_serialize_topic(&ub, &local_pose_topic);
        if (!success) {
            // TODO sometimes serialization fails on bootup. Determine why.
            // AP_HAL::panic("FATAL: DDS_Client failed to serialize");
        }
    }
}
#endif // AP_DDS_LOCAL_POSE_PUB_ENABLED

#if AP_DDS_LOCAL_VEL_PUB_ENABLED
void AP_DDS_Client::write_tx_local_velocity_topic()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        ucdrBuffer ub {};
        const uint32_t topic_size = geometry_msgs_msg_TwistStamped_size_of_topic(&tx_local_velocity_topic, 0);
        uxr_prepare_output_stream(&session, reliable_out, topics[to_underlying(TopicIndex::LOCAL_VELOCITY_PUB)].dw_id, &ub, topic_size);
        const bool success = geometry_msgs_msg_TwistStamped_serialize_topic(&ub, &tx_local_velocity_topic);
        if (!success) {
            // TODO sometimes serialization fails on bootup. Determine why.
            // AP_HAL::panic("FATAL: DDS_Client failed to serialize");
        }
    }
}
#endif // AP_DDS_LOCAL_VEL_PUB_ENABLED
#if AP_DDS_AIRSPEED_PUB_ENABLED
void AP_DDS_Client::write_tx_local_airspeed_topic()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        ucdrBuffer ub {};
        const uint32_t topic_size = ardupilot_msgs_msg_Airspeed_size_of_topic(&tx_local_airspeed_topic, 0);
        uxr_prepare_output_stream(&session, reliable_out, topics[to_underlying(TopicIndex::LOCAL_AIRSPEED_PUB)].dw_id, &ub, topic_size);
        const bool success = ardupilot_msgs_msg_Airspeed_serialize_topic(&ub, &tx_local_airspeed_topic);
        if (!success) {
            // TODO sometimes serialization fails on bootup. Determine why.
            // AP_HAL::panic("FATAL: DDS_Client failed to serialize");
        }
    }
}
#endif // AP_DDS_AIRSPEED_PUB_ENABLED
#if AP_DDS_RC_PUB_ENABLED
void AP_DDS_Client::write_tx_local_rc_topic()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        ucdrBuffer ub {};
        const uint32_t topic_size = ardupilot_msgs_msg_Rc_size_of_topic(&tx_local_rc_topic, 0);
        uxr_prepare_output_stream(&session, reliable_out, topics[to_underlying(TopicIndex::LOCAL_RC_PUB)].dw_id, &ub, topic_size);
        const bool success = ardupilot_msgs_msg_Rc_serialize_topic(&ub, &tx_local_rc_topic);
        if (!success) {
            // TODO sometimes serialization fails on bootup. Determine why.
            // AP_HAL::panic("FATAL: DDS_Client failed to serialize\n");
        }
    }
}
#endif // AP_DDS_RC_PUB_ENABLED
#if AP_DDS_IMU_PUB_ENABLED
void AP_DDS_Client::write_imu_topic()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        ucdrBuffer ub {};
        const uint32_t topic_size = sensor_msgs_msg_Imu_size_of_topic(&imu_topic, 0);
        uxr_prepare_output_stream(&session, reliable_out, topics[to_underlying(TopicIndex::IMU_PUB)].dw_id, &ub, topic_size);
        const bool success = sensor_msgs_msg_Imu_serialize_topic(&ub, &imu_topic);
        if (!success) {
            // TODO sometimes serialization fails on bootup. Determine why.
            // AP_HAL::panic("FATAL: DDS_Client failed to serialize");
        }
    }
}
#endif // AP_DDS_IMU_PUB_ENABLED

#if AP_DDS_GEOPOSE_PUB_ENABLED
void AP_DDS_Client::write_geo_pose_topic()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        ucdrBuffer ub {};
        const uint32_t topic_size = geographic_msgs_msg_GeoPoseStamped_size_of_topic(&geo_pose_topic, 0);
        uxr_prepare_output_stream(&session, reliable_out, topics[to_underlying(TopicIndex::GEOPOSE_PUB)].dw_id, &ub, topic_size);
        const bool success = geographic_msgs_msg_GeoPoseStamped_serialize_topic(&ub, &geo_pose_topic);
        if (!success) {
            // TODO sometimes serialization fails on bootup. Determine why.
            // AP_HAL::panic("FATAL: DDS_Client failed to serialize");
        }
    }
}
#endif // AP_DDS_GEOPOSE_PUB_ENABLED

#if AP_DDS_CLOCK_PUB_ENABLED
void AP_DDS_Client::write_clock_topic()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        ucdrBuffer ub {};
        const uint32_t topic_size = rosgraph_msgs_msg_Clock_size_of_topic(&clock_topic, 0);
        uxr_prepare_output_stream(&session, reliable_out, topics[to_underlying(TopicIndex::CLOCK_PUB)].dw_id, &ub, topic_size);
        const bool success = rosgraph_msgs_msg_Clock_serialize_topic(&ub, &clock_topic);
        if (!success) {
            // TODO sometimes serialization fails on bootup. Determine why.
            // AP_HAL::panic("FATAL: DDS_Client failed to serialize");
        }
    }
}
#endif // AP_DDS_CLOCK_PUB_ENABLED

#if AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED
void AP_DDS_Client::write_gps_global_origin_topic()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        ucdrBuffer ub {};
        const uint32_t topic_size = geographic_msgs_msg_GeoPointStamped_size_of_topic(&gps_global_origin_topic, 0);
        uxr_prepare_output_stream(&session, reliable_out, topics[to_underlying(TopicIndex::GPS_GLOBAL_ORIGIN_PUB)].dw_id, &ub, topic_size);
        const bool success = geographic_msgs_msg_GeoPointStamped_serialize_topic(&ub, &gps_global_origin_topic);
        if (!success) {
            // AP_HAL::panic("FATAL: DDS_Client failed to serialize");
        }
    }
}
#endif // AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED

#if AP_DDS_GOAL_PUB_ENABLED
void AP_DDS_Client::write_goal_topic()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        ucdrBuffer ub {};
        const uint32_t topic_size = geographic_msgs_msg_GeoPointStamped_size_of_topic(&goal_topic, 0);
        uxr_prepare_output_stream(&session, reliable_out, topics[to_underlying(TopicIndex::GOAL_PUB)].dw_id, &ub, topic_size);
        const bool success = geographic_msgs_msg_GeoPointStamped_serialize_topic(&ub, &goal_topic);
        if (!success) {
            // AP_HAL::panic("FATAL: DDS_Client failed to serialize");
        }
    }
}
#endif // AP_DDS_GOAL_PUB_ENABLED

#if AP_DDS_STATUS_PUB_ENABLED
void AP_DDS_Client::write_status_topic()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        ucdrBuffer ub {};
        const uint32_t topic_size = ardupilot_msgs_msg_Status_size_of_topic(&status_topic, 0);
        uxr_prepare_output_stream(&session, reliable_out, topics[to_underlying(TopicIndex::STATUS_PUB)].dw_id, &ub, topic_size);
        const bool success = ardupilot_msgs_msg_Status_serialize_topic(&ub, &status_topic);
        if (!success) {
            // TODO sometimes serialization fails on bootup. Determine why.
            // AP_HAL::panic("FATAL: DDS_Client failed to serialize");
        }
    }
}
#endif // AP_DDS_STATUS_PUB_ENABLED

void AP_DDS_Client::update()
{
    WITH_SEMAPHORE(csem);
    const auto cur_time_ms = AP_HAL::millis64();

#if AP_DDS_TIME_PUB_ENABLED
    if (cur_time_ms - last_time_time_ms > DELAY_TIME_TOPIC_MS) {
        update_topic(time_topic);
        last_time_time_ms = cur_time_ms;
        write_time_topic();
    }
#endif // AP_DDS_TIME_PUB_ENABLED
#if AP_DDS_NAVSATFIX_PUB_ENABLED
    for (uint8_t gps_instance = 0; gps_instance < GPS_MAX_INSTANCES; gps_instance++) {
        if (update_topic(nav_sat_fix_topic, gps_instance)) {
            write_nav_sat_fix_topic();
        }
    }
#endif // AP_DDS_NAVSATFIX_PUB_ENABLED
#if AP_DDS_BATTERY_STATE_PUB_ENABLED
    if (cur_time_ms - last_battery_state_time_ms > DELAY_BATTERY_STATE_TOPIC_MS) {
        for (uint8_t battery_instance = 0; battery_instance < AP_BATT_MONITOR_MAX_INSTANCES; battery_instance++) {
            update_topic(battery_state_topic, battery_instance);
            if (battery_state_topic.present) {
                write_battery_state_topic();
            }
        }
        last_battery_state_time_ms = cur_time_ms;
    }
#endif // AP_DDS_BATTERY_STATE_PUB_ENABLED
#if AP_DDS_LOCAL_POSE_PUB_ENABLED
    if (cur_time_ms - last_local_pose_time_ms > DELAY_LOCAL_POSE_TOPIC_MS) {
        update_topic(local_pose_topic);
        last_local_pose_time_ms = cur_time_ms;
        write_local_pose_topic();
    }
#endif // AP_DDS_LOCAL_POSE_PUB_ENABLED
#if AP_DDS_LOCAL_VEL_PUB_ENABLED
    if (cur_time_ms - last_local_velocity_time_ms > DELAY_LOCAL_VELOCITY_TOPIC_MS) {
        update_topic(tx_local_velocity_topic);
        last_local_velocity_time_ms = cur_time_ms;
        write_tx_local_velocity_topic();
    }
#endif // AP_DDS_LOCAL_VEL_PUB_ENABLED
#if AP_DDS_AIRSPEED_PUB_ENABLED
    if (cur_time_ms - last_airspeed_time_ms > DELAY_AIRSPEED_TOPIC_MS) {
        last_airspeed_time_ms = cur_time_ms;
        if (update_topic(tx_local_airspeed_topic)) {
            write_tx_local_airspeed_topic();
        }
    }
#endif // AP_DDS_AIRSPEED_PUB_ENABLED
#if AP_DDS_RC_PUB_ENABLED
    if (cur_time_ms - last_rc_time_ms > DELAY_RC_TOPIC_MS) {
        last_rc_time_ms = cur_time_ms;
        if (update_topic(tx_local_rc_topic)) {
            write_tx_local_rc_topic();
        }
    }
#endif // AP_DDS_RC_PUB_ENABLED
#if AP_DDS_IMU_PUB_ENABLED
    if (cur_time_ms - last_imu_time_ms > DELAY_IMU_TOPIC_MS) {
        update_topic(imu_topic);
        last_imu_time_ms = cur_time_ms;
        write_imu_topic();
    }
#endif // AP_DDS_IMU_PUB_ENABLED
#if AP_DDS_GEOPOSE_PUB_ENABLED
    if (cur_time_ms - last_geo_pose_time_ms > DELAY_GEO_POSE_TOPIC_MS) {
        update_topic(geo_pose_topic);
        last_geo_pose_time_ms = cur_time_ms;
        write_geo_pose_topic();
    }
#endif // AP_DDS_GEOPOSE_PUB_ENABLED
#if AP_DDS_CLOCK_PUB_ENABLED
    if (cur_time_ms - last_clock_time_ms > DELAY_CLOCK_TOPIC_MS) {
        update_topic(clock_topic);
        last_clock_time_ms = cur_time_ms;
        write_clock_topic();
    }
#endif // AP_DDS_CLOCK_PUB_ENABLED
#if AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED
    if (cur_time_ms - last_gps_global_origin_time_ms > DELAY_GPS_GLOBAL_ORIGIN_TOPIC_MS) {
        update_topic(gps_global_origin_topic);
        last_gps_global_origin_time_ms = cur_time_ms;
        write_gps_global_origin_topic();
    }
#endif // AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED
#if AP_DDS_GOAL_PUB_ENABLED
    if (cur_time_ms - last_goal_time_ms > DELAY_GOAL_TOPIC_MS) {
        if (update_topic_goal(goal_topic)) {
            write_goal_topic();
        }
        last_goal_time_ms = cur_time_ms;
    }
#endif // AP_DDS_GOAL_PUB_ENABLED
#if AP_DDS_STATUS_PUB_ENABLED
    if (cur_time_ms - last_status_check_time_ms > DELAY_STATUS_TOPIC_MS) {
        if (update_topic(status_topic)) {
            write_status_topic();
        }
        last_status_check_time_ms = cur_time_ms;
    }
#endif // AP_DDS_STATUS_PUB_ENABLED

    status_ok = uxr_run_session_time(&session, 1);
}

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
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
#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#endif // AP_DDS_ENABLED
