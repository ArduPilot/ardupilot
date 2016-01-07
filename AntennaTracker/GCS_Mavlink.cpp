// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Tracker.h"

// default sensors are present and healthy: gyro, accelerometer, barometer, rate_control, attitude_stabilization, yaw_position, altitude control, x/y position control, motor_control
#define MAVLINK_SENSOR_PRESENT_DEFAULT (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE | MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL | MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION | MAV_SYS_STATUS_SENSOR_YAW_POSITION | MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL | MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL | MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS)

/*
 *  !!NOTE!!
 *
 *  the use of NOINLINE separate functions for each message type avoids
 *  a compiler bug in gcc that would cause it to use far more stack
 *  space than is needed. Without the NOINLINE we use the sum of the
 *  stack needed for each message type. Please be careful to follow the
 *  pattern below when adding any new messages
 */

void Tracker::send_heartbeat(mavlink_channel_t chan)
{
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t system_status = MAV_STATE_ACTIVE;
    uint32_t custom_mode = control_mode;

    // work out the base_mode. This value is not very useful
    // for APM, but we calculate it as best we can so a generic
    // MAVLink enabled ground station can work out something about
    // what the MAV is up to. The actual bit values are highly
    // ambiguous for most of the APM flight modes. In practice, you
    // only get useful information from the custom_mode, which maps to
    // the APM flight mode and has a well defined meaning in the
    // ArduPlane documentation
    switch (control_mode) {
    case MANUAL:
        base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        break;

    case STOP:
        break;

    case SCAN:
    case SERVO_TEST:
    case AUTO:
        base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED |
            MAV_MODE_FLAG_STABILIZE_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;

    case INITIALISING:
        system_status = MAV_STATE_CALIBRATING;
        break;
    }

    mavlink_msg_heartbeat_send(
        chan,
        MAV_TYPE_ANTENNA_TRACKER,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode,
        custom_mode,
        system_status);
}

void Tracker::send_attitude(mavlink_channel_t chan)
{
    Vector3f omega = ahrs.get_gyro();
    mavlink_msg_attitude_send(
        chan,
        AP_HAL::millis(),
        ahrs.roll,
        ahrs.pitch,
        ahrs.yaw,
        omega.x,
        omega.y,
        omega.z);
}


void Tracker::send_location(mavlink_channel_t chan)
{
    uint32_t fix_time;
    if (gps.status() >= AP_GPS::GPS_OK_FIX_2D) {
        fix_time = gps.last_fix_time_ms();
    } else {
        fix_time = AP_HAL::millis();
    }
    const Vector3f &vel = gps.velocity();
    mavlink_msg_global_position_int_send(
        chan,
        fix_time,
        current_loc.lat,                // in 1E7 degrees
        current_loc.lng,                // in 1E7 degrees
        current_loc.alt * 10,        // millimeters above sea level
        0,
        vel.x * 100,  // X speed cm/s (+ve North)
        vel.y * 100,  // Y speed cm/s (+ve East)
        vel.z * -100, // Z speed cm/s (+ve up)
        ahrs.yaw_sensor);
}

void Tracker::send_radio_out(mavlink_channel_t chan)
{
    mavlink_msg_servo_output_raw_send(
        chan,
        AP_HAL::micros(),
        0,     // port
        hal.rcout->read(0),
        hal.rcout->read(1),
        hal.rcout->read(2),
        hal.rcout->read(3),
        hal.rcout->read(4),
        hal.rcout->read(5),
        hal.rcout->read(6),
        hal.rcout->read(7));
}

void Tracker::send_hwstatus(mavlink_channel_t chan)
{
    mavlink_msg_hwstatus_send(
        chan,
        0,
        hal.i2c->lockup_count());
}

void Tracker::send_nav_controller_output(mavlink_channel_t chan)
{
    mavlink_msg_nav_controller_output_send(
        chan,
        0,
        nav_status.pitch,
        nav_status.bearing,
        nav_status.bearing,
        nav_status.distance,
        nav_status.altitude_difference,
        0,
        0);
}


// report simulator state
void Tracker::send_simstate(mavlink_channel_t chan)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    sitl.simstate_send(chan);
#endif
}

/*
  We eavesdrop on MAVLINK_MSG_ID_GLOBAL_POSITION_INT and
  MAVLINK_MSG_ID_SCALED_PRESSUREs
*/
void Tracker::mavlink_snoop(const mavlink_message_t* msg)
{
    // return immediately if sysid doesn't match our target sysid
    if ((g.sysid_target != 0) && (g.sysid_target != msg->sysid)) {
        return;
    }

    switch (msg->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
    {
        mavlink_check_target(msg);
        break;
    }

    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
    {
        // decode
        mavlink_global_position_int_t packet;
        mavlink_msg_global_position_int_decode(msg, &packet);
        tracking_update_position(packet);
        break;
    }
    
    case MAVLINK_MSG_ID_SCALED_PRESSURE:
    {
        // decode
        mavlink_scaled_pressure_t packet;
        mavlink_msg_scaled_pressure_decode(msg, &packet);
        tracking_update_pressure(packet);
        break;
    }
    }
}

// locks onto a particular target sysid and sets it's position data stream to at least 1hz
void Tracker::mavlink_check_target(const mavlink_message_t* msg)
{
    // exit immediately if the target has already been set
    if (target_set) {
        return;
    }

    // decode
    mavlink_heartbeat_t packet;
    mavlink_msg_heartbeat_decode(msg, &packet);

    // exit immediately if this is not a vehicle we would track
    if ((packet.type == MAV_TYPE_ANTENNA_TRACKER) ||
        (packet.type == MAV_TYPE_GCS) ||
        (packet.type == MAV_TYPE_ONBOARD_CONTROLLER) ||
        (packet.type == MAV_TYPE_GIMBAL)) {
        return;
    }

    // set our sysid to the target, this ensures we lock onto a single vehicle
    if (g.sysid_target == 0) {
        g.sysid_target = msg->sysid;
    }

    // send data stream request to target on all channels
    //  Note: this doesn't check success for all sends meaning it's not guaranteed the vehicle's positions will be sent at 1hz
    gcs_frontend.request_data_streams(msg->sysid, msg->compid);

    // flag target has been set
    target_set = true;
}

void Tracker::mavlink_delay_cb()
{
    static uint32_t last_50hz;

    if (in_mavlink_delay) {
        return;
    }

    in_mavlink_delay = true;

    // process MAVLink packets:
    gcs_frontend.delay_cb();

    // update Notify events:
    uint32_t tnow = AP_HAL::millis();
    if (tnow - last_50hz > 20) {
        last_50hz = tnow;
        notify.update();
    }

    // see if USB has been connected or disconnected:
    check_usb_mux();

    in_mavlink_delay = false;
}

/*
 *  send data streams in the given rate range on both links
 */
void Tracker::gcs_data_stream_send(void)
{
    gcs_frontend.data_stream_send();
}

/*
 *  look for incoming commands on the GCS links
 */
void Tracker::gcs_update(void)
{
    gcs_frontend.update();
}

/**
   retry any deferred messages
 */
void Tracker::gcs_retry_deferred(void)
{
    gcs_frontend.send_message(MSG_RETRY_DEFERRED);
}
