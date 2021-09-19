#include "mode.h"
#include "Plane.h"

bool ModeGuided::_enter()
{
    plane.guided_throttle_passthru = false;
    /*
      when entering guided mode we set the target as the current
      location. This matches the behaviour of the copter code
    */
    plane.guided_WP_loc = plane.current_loc;

    if (plane.quadplane.guided_mode_enabled()) {
        /*
          if using Q_GUIDED_MODE then project forward by the stopping distance
        */
        plane.guided_WP_loc.offset_bearing(degrees(plane.ahrs.groundspeed_vector().angle()),
                                           plane.quadplane.stopping_distance());
    }
    plane.set_guided_WP();

    const int32_t targetAngle = plane.g.hm_target_angle;
    const int32_t relative_altitude_cm = plane.adjusted_relative_altitude_cm() * 0.01;

    currentBearing = wrap_360_cd(plane.ahrs.yaw_sensor) + targetAngle * 100;
    minAlt = constrain_float(relative_altitude_cm - plane.g.hm_alt_diff, plane.g.hm_min_alt, relative_altitude_cm);

    hal.console->printf("Hd: %ld ; Tgt_Hd %ld\n", wrap_360_cd(plane.ahrs.yaw_sensor), currentBearing);
    hal.console->printf("min altitude: %f \n", minAlt);

    stopRoll = false;
    stopPitch = false;
    return true;
}

void ModeGuided::update()
{
    if (plane.auto_state.vtol_loiter && plane.quadplane.available()) {
        plane.quadplane.guided_update();
    } else {
        uint32_t now = AP_HAL::millis();

        int32_t diff = currentBearing - wrap_360_cd(plane.ahrs.yaw_sensor);
        bool shouldRoll = abs(diff) > plane.g.hm_deg_eps;
        bool shouldPitch = plane.adjusted_relative_altitude_cm() * 0.01 > minAlt;

        if (shouldRoll && shouldPitch && !stopRoll) {
            gcs().send_text(MAV_SEVERITY_INFO, "Tgt_Hd: %f \n", diff * 0.01);
            plane.guided_state.forced_rpy_cd.x = diff;
            plane.guided_state.last_forced_rpy_ms.x = now;

            plane.guided_state.forced_rpy_cd.y = 1;
            plane.guided_state.last_forced_rpy_ms.y = now;
        }
        else if (shouldPitch && !stopPitch) {
            gcs().send_text(MAV_SEVERITY_INFO, "Tgt_Alt: %f \n", plane.adjusted_relative_altitude_cm() * 0.01 - minAlt);
            stopRoll = true;
            plane.guided_state.forced_rpy_cd.x = 1;
            plane.guided_state.last_forced_rpy_ms.x = now;

            plane.guided_state.forced_rpy_cd.y = plane.g.hm_attack_angle * 100;
            plane.guided_state.last_forced_rpy_ms.y = now;

            plane.guided_state.forced_throttle = plane.g.hm_attack_thr;
            plane.guided_state.last_forced_throttle_ms = now;
        }
        else {
            stopPitch = true;

            plane.guided_state.last_forced_rpy_ms.zero();
            plane.guided_state.last_forced_throttle_ms = 0;

            plane.set_mode(plane.mode_rtl, ModeReason::MISSION_END);
        }

        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }
}

void ModeGuided::navigate()
{
    // Zero indicates to use WP_LOITER_RAD
    plane.update_loiter(0);
}
