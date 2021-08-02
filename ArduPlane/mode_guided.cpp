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

    currentBearing = plane.ahrs.yaw_sensor + targetAngle * 100;
    minAlt = plane.relative_altitude - plane.g.hm_alt_diff;
    minAlt = constrain_float(minAlt, plane.g.hm_min_alt, plane.relative_altitude);

    hal.console->printf("Hd: %d ; Tgt_Hd %d\n", plane.ahrs.yaw_sensor, currentBearing);
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

        int32_t diff = currentBearing - plane.ahrs.yaw_sensor;
        int32_t diffAbs = abs(diff);

        if (diffAbs > plane.g.hm_deg_eps && !stopRoll) {
            hal.console->printf("Tgt_Hd: %d \n", diff);
            plane.guided_state.forced_rpy_cd.y = 1;
            plane.guided_state.last_forced_rpy_ms.y = now;

            plane.guided_state.forced_rpy_cd.x = diff;
            plane.guided_state.last_forced_rpy_ms.x = now;

            plane.calc_nav_roll();
            plane.calc_nav_pitch();
        }
        else if(plane.relative_altitude > minAlt) {
            stopRoll = true;
            plane.guided_state.forced_rpy_cd.y = plane.g.hm_attack_angle * 100;
            plane.guided_state.last_forced_rpy_ms.y = now;

            plane.guided_state.forced_rpy_cd.x = 1;
            plane.guided_state.last_forced_rpy_ms.x = now;

            plane.calc_nav_roll();
            plane.calc_nav_pitch();
        }
        else {
            plane.set_mode(plane.mode_cruise, ModeReason::MISSION_END);
        }

        plane.calc_throttle();
    }
}

void ModeGuided::navigate()
{
    // Zero indicates to use WP_LOITER_RAD
    plane.update_loiter(0);
}
