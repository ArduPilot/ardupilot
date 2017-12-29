#include "Copter.h"

/*
 * mode_chase.cpp - chase another mavlink-enabled vehicle by system id
 *
 * TODO: set ROI yaw mode / point camera at target
 * TODO: stick control to move around on sphere
 * TODO: stick control to change sphere diameter
 * TODO: "channel 7 option" to lock onto "pointed at" target
 * TODO: do better in terms of loitering around the moving point; may need a PID?  Maybe use loiter controller somehow?
 */

#if 1
#define Debug(fmt, args ...)  do {::fprintf(stderr, "%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); hal.scheduler->delay(1); } while(0)
#else
#define Debug(fmt, args ...)
#endif

// initialise avoid_adsb controller
bool Copter::ModeChase::init(const bool ignore_checks)
{
    // re-use guided mode
    return Copter::ModeGuided::init(ignore_checks);
}

bool Copter::ModeChase::set_velocity(const Vector3f& velocity_neu)
{
    // check flight mode
    if (_copter.flightmode != &_copter.mode_chase) {
        return false;
    }

    return true;
}

void Copter::ModeChase::run_lonely_mode()
{
    if (lonely_mode == nullptr) {
        if (copter.mode_loiter.init(false)) {
            lonely_mode = &copter.mode_loiter;
        } else if(copter.mode_rtl.init(false)) {
            lonely_mode = &copter.mode_rtl;
        } else {
            copter.mode_land.init(false);
            lonely_mode = &copter.mode_land;
        }

        gcs().send_text(MAV_SEVERITY_INFO, "Chase: Lonely; %s", lonely_mode->name());
    }

    lonely_mode->run();
}


void Copter::ModeChase::run()
{
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock()) {
        zero_throttle_and_relax_ac();
        return;
    }

    // re-use guided mode's velocity controller
    // Note: this is safe from interference from GCSs and companion computer's whose guided mode
    //       position and velocity requests will be ignored while the vehicle is not in guided mode

    const uint32_t now = AP_HAL::millis();
    if (now - target_last_update_ms > target_update_timeout_ms) {
        return run_lonely_mode();
    }

    Vector3f to_vehicle = location_3d_diff_NED(_copter.current_loc, target_loc);
    Debug("to_vehicle: %f %f %f", to_vehicle.x, to_vehicle.y, to_vehicle.z);
    const float distance_to_vehicle = to_vehicle.length();

    if (distance_to_vehicle > sphere_radius_max) {
        return run_lonely_mode();
    }
    lonely_mode = nullptr;

    const float distance_to_stop = pos_control->get_stopping_distance_xyz() * 0.01f;

    const float distance_to_move = distance_to_vehicle - sphere_radius_min;
    Debug("distance_to_vehicle=%f move=%f stop=%f",
          distance_to_vehicle,
          distance_to_move,
          distance_to_stop);
    to_vehicle.normalize();
    if (fabsf(distance_to_move) > distance_to_stop) {
        to_vehicle *= closure_speed * 100; // m/s to cm/s (which set_velocity takes)
        to_vehicle.z = -to_vehicle.z; // translate to NEU
        if (distance_to_move < 0) {
            to_vehicle = -to_vehicle; // too close!  back up!
        }
    } else {
        to_vehicle.x = 0;
        to_vehicle.y = 0;
        to_vehicle.z = 0;
    }
    to_vehicle += target_vel;

    // re-use guided mode's velocity controller (takes NEU)
    Copter::ModeGuided::set_velocity(to_vehicle);

    Copter::ModeGuided::run();
}



void Copter::ModeChase::mavlink_packet_received(const mavlink_message_t &msg)
{
    if (copter.flightmode != &copter.mode_chase) {
        return;
    }
    if (msg.sysid != target_srcid) {
        return;
    }
    if (msg.msgid != MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        // handle position only for now
        return;
    }

    mavlink_global_position_int_t packet;
    mavlink_msg_global_position_int_decode(&msg, &packet);
    target_loc.lat = packet.lat;
    target_loc.lng = packet.lon;
    target_loc.alt = packet.relative_alt / 10; // mm -> cm
    target_vel.x = packet.vx/100.0f; // cm/s to m/s
    target_vel.y = packet.vy/100.0f; // cm/s to m/s
    target_vel.z = packet.vz/100.0f; // cm/s to m/s

    target_last_update_ms = AP_HAL::millis();
}
