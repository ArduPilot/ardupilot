/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  main logic for servo control
 */

#include "Plane.h"
#include <utility>

/*****************************************
* Throttle slew limit
*****************************************/
void Plane::throttle_slew_limit(void)
{
    uint8_t slewrate = aparm.throttle_slewrate;
    if (control_mode == &mode_auto) {
        if (auto_state.takeoff_complete == false && g.takeoff_throttle_slewrate != 0) {
            slewrate = g.takeoff_throttle_slewrate;
        } else if (landing.get_throttle_slewrate() != 0 && flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND) {
            slewrate = landing.get_throttle_slewrate();
        }
    }
    // if slew limit rate is set to zero then do not slew limit
    if (slewrate) {                   
        SRV_Channels::limit_slew_rate(SRV_Channel::k_throttle, slewrate, G_Dt);
        SRV_Channels::limit_slew_rate(SRV_Channel::k_throttleLeft, slewrate, G_Dt);
        SRV_Channels::limit_slew_rate(SRV_Channel::k_throttleRight, slewrate, G_Dt);
    }
}

/* We want to suppress the throttle if we think we are on the ground and in an autopilot controlled throttle mode.

   Disable throttle if following conditions are met:
   *       1 - We are in Circle mode (which we use for short term failsafe), or in FBW-B or higher
   *       AND
   *       2 - Our reported altitude is within 10 meters of the home altitude.
   *       3 - Our reported speed is under 5 meters per second.
   *       4 - We are not performing a takeoff in Auto mode or takeoff speed/accel not yet reached
   *       OR
   *       5 - Home location is not set
   *       OR
   *       6- Landing does not want to allow throttle
*/
bool Plane::suppress_throttle(void)
{
#if PARACHUTE == ENABLED
    if (auto_throttle_mode && parachute.release_initiated()) {
        // throttle always suppressed in auto-throttle modes after parachute release initiated
        throttle_suppressed = true;
        return true;
    }
#endif

    if (landing.is_throttle_suppressed()) {
        return true;
    }

    if (!throttle_suppressed) {
        // we've previously met a condition for unsupressing the throttle
        return false;
    }
    if (!auto_throttle_mode) {
        // the user controls the throttle
        throttle_suppressed = false;
        return false;
    }

    if (control_mode == &mode_auto && g.auto_fbw_steer == 42) {
        // user has throttle control
        return false;
    }

    bool gps_movement = (gps.status() >= AP_GPS::GPS_OK_FIX_2D && gps.ground_speed() >= 5);
    
    if (control_mode == &mode_auto &&
        auto_state.takeoff_complete == false) {

        uint32_t launch_duration_ms = ((int32_t)g.takeoff_throttle_delay)*100 + 2000;
        if (is_flying() &&
            millis() - started_flying_ms > MAX(launch_duration_ms, 5000U) && // been flying >5s in any mode
            adjusted_relative_altitude_cm() > 500 && // are >5m above AGL/home
            labs(ahrs.pitch_sensor) < 3000 && // not high pitch, which happens when held before launch
            gps_movement) { // definite gps movement
            // we're already flying, do not suppress the throttle. We can get
            // stuck in this condition if we reset a mission and cmd 1 is takeoff
            // but we're currently flying around below the takeoff altitude
            throttle_suppressed = false;
            return false;
        }
        if (auto_takeoff_check()) {
            // we're in auto takeoff 
            throttle_suppressed = false;
            auto_state.baro_takeoff_alt = barometer.get_altitude();
            return false;
        }
        // keep throttle suppressed
        return true;
    }
    
    if (fabsf(relative_altitude) >= 10.0f) {
        // we're more than 10m from the home altitude
        throttle_suppressed = false;
        return false;
    }

    if (gps_movement) {
        // if we have an airspeed sensor, then check it too, and
        // require 5m/s. This prevents throttle up due to spiky GPS
        // groundspeed with bad GPS reception
        if ((!ahrs.airspeed_sensor_enabled()) || airspeed.get_airspeed() >= 5) {
            // we're moving at more than 5 m/s
            throttle_suppressed = false;
            return false;        
        }
    }

    if (quadplane.is_flying()) {
        throttle_suppressed = false;
        return false;
    }

    // throttle remains suppressed
    return true;
}


/*
  mixer for elevon and vtail channels setup using designated servo
  function values. This mixer operates purely on scaled values,
  allowing the user to trim and limit individual servos using the
  SERVOn_* parameters
 */
void Plane::channel_function_mixer(SRV_Channel::Aux_servo_function_t func1_in, SRV_Channel::Aux_servo_function_t func2_in,
                                   SRV_Channel::Aux_servo_function_t func1_out, SRV_Channel::Aux_servo_function_t func2_out)
{
    // the order is setup so that non-reversed servos go "up", and
    // func1 is the "left" channel. Users can adjust with channel
    // reversal as needed
    float in1 = SRV_Channels::get_output_scaled(func1_in);
    float in2 = SRV_Channels::get_output_scaled(func2_in);

    // apply MIXING_OFFSET to input channels
    if (g.mixing_offset < 0) {
        in2 *= (100 - g.mixing_offset) * 0.01;
    } else if (g.mixing_offset > 0) {
        in1 *= (100 + g.mixing_offset) * 0.01;
    }
    
    float out1 = constrain_float((in2 - in1) * g.mixing_gain, -4500, 4500);
    float out2 = constrain_float((in2 + in1) * g.mixing_gain, -4500, 4500);
    SRV_Channels::set_output_scaled(func1_out, out1);
    SRV_Channels::set_output_scaled(func2_out, out2);
}


/*
  setup flaperon output channels
 */
void Plane::flaperon_update(int8_t flap_percent)
{
    if (!SRV_Channels::function_assigned(SRV_Channel::k_flaperon_left) &&
        !SRV_Channels::function_assigned(SRV_Channel::k_flaperon_right)) {
        return;
    }
    /*
      flaperons are implemented as a mixer between aileron and a
      percentage of flaps. Flap input can come from a manual channel
      or from auto flaps.
     */
    float aileron = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
    float flaperon_left  = constrain_float(aileron + flap_percent * 45, -4500, 4500);
    float flaperon_right = constrain_float(aileron - flap_percent * 45, -4500, 4500);
    SRV_Channels::set_output_scaled(SRV_Channel::k_flaperon_left, flaperon_left);
    SRV_Channels::set_output_scaled(SRV_Channel::k_flaperon_right, flaperon_right);
}


/*
  setup differential spoiler output channels

  Differential spoilers are a type of elevon that is split on each
  wing to give yaw control, mixed from rudder
 */
void Plane::dspoiler_update(void)
{
    // just check we have a left dspoiler, and if so calculate all outputs
    if (!SRV_Channels::function_assigned(SRV_Channel::k_dspoilerLeft1)) {
        return;
    }
    float elevon_left = SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_left);
    float elevon_right = SRV_Channels::get_output_scaled(SRV_Channel::k_elevon_right);
    float rudder_rate = g.dspoiler_rud_rate * 0.01f;
    float rudder = SRV_Channels::get_output_scaled(SRV_Channel::k_rudder) * rudder_rate;
    float dspoiler1_left = elevon_left;
    float dspoiler2_left = elevon_left;
    float dspoiler1_right = elevon_right;
    float dspoiler2_right = elevon_right;

    if (rudder > 0) {
        // apply rudder to right wing
        dspoiler1_right = constrain_float(elevon_right + rudder, -4500, 4500);
        dspoiler2_right = constrain_float(elevon_right - rudder, -4500, 4500);
    } else {
        // apply rudder to left wing
        dspoiler1_left = constrain_float(elevon_left - rudder, -4500, 4500);
        dspoiler2_left = constrain_float(elevon_left + rudder, -4500, 4500);
    }

    const int16_t weight1 = g2.crow_flap_weight1.get();
    const int16_t weight2 = g2.crow_flap_weight2.get();
    if (weight1 > 0 || weight2 > 0) {
        /*
          apply crow flaps by apply the same split of the differential
          spoilers to both wings. Get flap percentage from k_flap_auto, which is set
          in set_servos_flaps() as the maximum of manual and auto flap control
         */
        int16_t flap_percent = SRV_Channels::get_output_scaled(SRV_Channel::k_flap_auto);

        // scale flaps so when weights are 100 they give full up or down
        const float flap_scaled = ((float)flap_percent) * 0.45;

        if (flap_percent > 0) {
            //apply crow brakes to both wings using flap percentage
            dspoiler1_left  = constrain_float(dspoiler1_left  + flap_scaled * weight1, -4500, 4500);
            dspoiler2_left  = constrain_float(dspoiler2_left  - flap_scaled * weight2, -4500, 4500);
            dspoiler1_right = constrain_float(dspoiler1_right + flap_scaled * weight1, -4500, 4500);
            dspoiler2_right = constrain_float(dspoiler2_right - flap_scaled * weight2, -4500, 4500);
        }
    }

    SRV_Channels::set_output_scaled(SRV_Channel::k_dspoilerLeft1, dspoiler1_left);
    SRV_Channels::set_output_scaled(SRV_Channel::k_dspoilerLeft2, dspoiler2_left);
    SRV_Channels::set_output_scaled(SRV_Channel::k_dspoilerRight1, dspoiler1_right);
    SRV_Channels::set_output_scaled(SRV_Channel::k_dspoilerRight2, dspoiler2_right);
}

/*
  setup servos for idle mode
  Idle mode is used during balloon launch to keep servos still, apart
  from occasional wiggle to prevent freezing up
 */
void Plane::set_servos_idle(void)
{
    int16_t servo_value = 0;
    // move over full range for 2 seconds
    auto_state.idle_wiggle_stage += 2;
    if (auto_state.idle_wiggle_stage < 50) {
        servo_value = auto_state.idle_wiggle_stage * (4500 / 50);
    } else if (auto_state.idle_wiggle_stage < 100) {
        servo_value = (100 - auto_state.idle_wiggle_stage) * (4500 / 50);        
    } else if (auto_state.idle_wiggle_stage < 150) {
        servo_value = (100 - auto_state.idle_wiggle_stage) * (4500 / 50);        
    } else if (auto_state.idle_wiggle_stage < 200) {
        servo_value = (auto_state.idle_wiggle_stage-200) * (4500 / 50);        
    } else {
        auto_state.idle_wiggle_stage = 0;
    }
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, servo_value);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, servo_value);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, servo_value);
    SRV_Channels::set_output_to_trim(SRV_Channel::k_throttle);

    SRV_Channels::output_ch_all();
}

/*
  pass through channels in manual mode
 */
void Plane::set_servos_manual_passthrough(void)
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, channel_roll->get_control_in_zero_dz());
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, channel_pitch->get_control_in_zero_dz());
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, channel_rudder->get_control_in_zero_dz());
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, get_throttle_input(true));
}

/*
  calculate any throttle limits based on the watt limiter
 */
void Plane::throttle_watt_limiter(int8_t &min_throttle, int8_t &max_throttle)
{
    uint32_t now = millis();
    if (battery.overpower_detected()) {
        // overpower detected, cut back on the throttle if we're maxing it out by calculating a limiter value
        // throttle limit will attack by 10% per second
        
        if (SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) > 0 && // demanding too much positive thrust
            throttle_watt_limit_max < max_throttle - 25 &&
            now - throttle_watt_limit_timer_ms >= 1) {
            // always allow for 25% throttle available regardless of battery status
            throttle_watt_limit_timer_ms = now;
            throttle_watt_limit_max++;
            
        } else if (SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) < 0 &&
                   min_throttle < 0 && // reverse thrust is available
                   throttle_watt_limit_min < -(min_throttle) - 25 &&
                   now - throttle_watt_limit_timer_ms >= 1) {
            // always allow for 25% throttle available regardless of battery status
            throttle_watt_limit_timer_ms = now;
            throttle_watt_limit_min++;
        }
        
    } else if (now - throttle_watt_limit_timer_ms >= 1000) {
        // it has been 1 second since last over-current, check if we can resume higher throttle.
        // this throttle release is needed to allow raising the max_throttle as the battery voltage drains down
        // throttle limit will release by 1% per second
        if (SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) > throttle_watt_limit_max && // demanding max forward thrust
            throttle_watt_limit_max > 0) { // and we're currently limiting it
            throttle_watt_limit_timer_ms = now;
            throttle_watt_limit_max--;
            
        } else if (SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) < throttle_watt_limit_min && // demanding max negative thrust
                   throttle_watt_limit_min > 0) { // and we're limiting it
            throttle_watt_limit_timer_ms = now;
            throttle_watt_limit_min--;
        }
    }
    
    max_throttle = constrain_int16(max_throttle, 0, max_throttle - throttle_watt_limit_max);
    if (min_throttle < 0) {
        min_throttle = constrain_int16(min_throttle, min_throttle + throttle_watt_limit_min, 0);
    }
}
    


/*
  setup output channels all non-manual modes
 */
void Plane::set_servos_controlled(void)
{
    if (flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND) {
        // allow landing to override servos if it would like to
        landing.override_servos();
    }

    // convert 0 to 100% (or -100 to +100) into PWM
    int8_t min_throttle = aparm.throttle_min.get();
    int8_t max_throttle = aparm.throttle_max.get();
    
    if (min_throttle < 0 && !allow_reverse_thrust()) {
        // reverse thrust is available but inhibited.
        min_throttle = 0;
    }
    
    if (flight_stage == AP_Vehicle::FixedWing::FLIGHT_TAKEOFF || flight_stage == AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND) {
        if(aparm.takeoff_throttle_max != 0) {
            max_throttle = aparm.takeoff_throttle_max;
        } else {
            max_throttle = aparm.throttle_max;
        }
    } else if (landing.is_flaring()) {
        min_throttle = 0;
    }
    
    // apply watt limiter
    throttle_watt_limiter(min_throttle, max_throttle);
    
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle,
                                    constrain_int16(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle), min_throttle, max_throttle));
    
    if (!hal.util->get_soft_armed()) {
        if (arming.arming_required() == AP_Arming::YES_ZERO_PWM) {
            SRV_Channels::set_output_limit(SRV_Channel::k_throttle, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleLeft, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleRight, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
        } else {
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, 0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, 0);
        }
    } else if (suppress_throttle()) {
        // throttle is suppressed in auto mode
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
        if (g.throttle_suppress_manual) {
            // manual pass through of throttle while throttle is suppressed
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, get_throttle_input(true));
        }
    } else if (g.throttle_passthru_stabilize && 
               (control_mode == &mode_stabilize ||
                control_mode == &mode_training ||
                control_mode == &mode_acro ||
                control_mode == &mode_fbwa ||
                control_mode == &mode_autotune) &&
               !failsafe.throttle_counter) {
        // manual pass through of throttle while in FBWA or
        // STABILIZE mode with THR_PASS_STAB set
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, get_throttle_input(true));
    } else if ((control_mode == &mode_guided || control_mode == &mode_avoidADSB) &&
               guided_throttle_passthru) {
        // manual pass through of throttle while in GUIDED
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, get_throttle_input(true));
    } else if (quadplane.in_vtol_mode()) {
        // ask quadplane code for forward throttle
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 
            constrain_int16(quadplane.forward_throttle_pct(), min_throttle, max_throttle));
    }

#if SOARING_ENABLED == ENABLED
    // suppress throttle when soaring is active
    if ((control_mode == &mode_fbwb || control_mode == &mode_cruise ||
        control_mode == &mode_auto || control_mode == &mode_loiter) &&
        g2.soaring_controller.is_active() &&
        g2.soaring_controller.get_throttle_suppressed()) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
    }
#endif
}

/*
  setup flap outputs
 */
void Plane::set_servos_flaps(void)
{
    // Auto flap deployment
    int8_t auto_flap_percent = 0;
    int8_t manual_flap_percent = 0;

    // work out any manual flap input
    RC_Channel *flapin = RC_Channels::rc_channel(g.flapin_channel-1);
    if (flapin != nullptr && !failsafe.rc_failsafe && failsafe.throttle_counter == 0) {
        manual_flap_percent = flapin->percent_input();
    }

    if (auto_throttle_mode) {
        int16_t flapSpeedSource = 0;
        if (ahrs.airspeed_sensor_enabled()) {
            flapSpeedSource = target_airspeed_cm * 0.01f;
        } else {
            flapSpeedSource = aparm.throttle_cruise;
        }
        if (g.flap_2_speed != 0 && flapSpeedSource <= g.flap_2_speed) {
            auto_flap_percent = g.flap_2_percent;
        } else if ( g.flap_1_speed != 0 && flapSpeedSource <= g.flap_1_speed) {
            auto_flap_percent = g.flap_1_percent;
        } //else flaps stay at default zero deflection

        if (flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND && landing.get_flap_percent() != 0) {
            auto_flap_percent = landing.get_flap_percent();
        }

        /*
          special flap levels for takeoff and landing. This works
          better than speed based flaps as it leads to less
          possibility of oscillation
         */
        if (control_mode == &mode_auto) {
            switch (flight_stage) {
            case AP_Vehicle::FixedWing::FLIGHT_TAKEOFF:
            case AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND:
                if (g.takeoff_flap_percent != 0) {
                    auto_flap_percent = g.takeoff_flap_percent;
                }
                break;
            case AP_Vehicle::FixedWing::FLIGHT_NORMAL:
                if (g.takeoff_flap_percent != 0 && in_preLaunch_flight_stage()) {
                    // TODO: move this to a new FLIGHT_PRE_TAKEOFF stage
                    auto_flap_percent = g.takeoff_flap_percent;
                }
                break;
            default:
                break;
            }
        }
    }

    // manual flap input overrides auto flap input
    if (abs(manual_flap_percent) > auto_flap_percent) {
        auto_flap_percent = manual_flap_percent;
    }

    SRV_Channels::set_output_scaled(SRV_Channel::k_flap_auto, auto_flap_percent);
    SRV_Channels::set_output_scaled(SRV_Channel::k_flap, manual_flap_percent);

    if (g.flap_slewrate) {
        SRV_Channels::limit_slew_rate(SRV_Channel::k_flap_auto, g.flap_slewrate, G_Dt);
        SRV_Channels::limit_slew_rate(SRV_Channel::k_flap, g.flap_slewrate, G_Dt);
    }    

    // output to flaperons, if any
    flaperon_update(auto_flap_percent);
}

#if LANDING_GEAR_ENABLED == ENABLED
/*
  change and report landing gear
 */
void Plane::change_landing_gear(AP_LandingGear::LandingGearCommand cmd)
{
    if (cmd != (AP_LandingGear::LandingGearCommand)gear.last_cmd) {
        if (SRV_Channels::function_assigned(SRV_Channel::k_landing_gear_control)) {
            g2.landing_gear.set_position(cmd);
            gcs().send_text(MAV_SEVERITY_INFO, "LandingGear: %s", cmd==AP_LandingGear::LandingGear_Retract?"RETRACT":"DEPLOY");
        }
        gear.last_cmd = (int8_t)cmd;
    }
}

/*
  setup landing gear state
 */
void Plane::set_landing_gear(void)
{
    if (control_mode == AUTO && hal.util->get_soft_armed() && is_flying()) {
        AP_LandingGear::LandingGearCommand cmd = (AP_LandingGear::LandingGearCommand)gear.last_auto_cmd;
        switch (flight_stage) {
        case AP_Vehicle::FixedWing::FLIGHT_LAND:
            cmd = AP_LandingGear::LandingGear_Deploy;
            break;
        case AP_Vehicle::FixedWing::FLIGHT_NORMAL:
            cmd = AP_LandingGear::LandingGear_Retract;
            break;
        case AP_Vehicle::FixedWing::FLIGHT_VTOL:
            if (quadplane.is_vtol_land(mission.get_current_nav_cmd().id)) {
                cmd = AP_LandingGear::LandingGear_Deploy;
            }
            break;
        default:
            break;
        }
        if (cmd != gear.last_auto_cmd) {
            gear.last_auto_cmd = (int8_t)cmd;
            change_landing_gear(cmd);
        }
    }
}
#endif // LANDING_GEAR_ENABLED

/*
  apply vtail and elevon mixers
  the rewrites radio_out for the corresponding channels
 */
void Plane::servo_output_mixers(void)
{
    // mix elevons and vtail channels
    channel_function_mixer(SRV_Channel::k_aileron, SRV_Channel::k_elevator, SRV_Channel::k_elevon_left, SRV_Channel::k_elevon_right);
    channel_function_mixer(SRV_Channel::k_rudder,  SRV_Channel::k_elevator, SRV_Channel::k_vtail_right, SRV_Channel::k_vtail_left);

    // implement differential spoilers
    dspoiler_update();
}

/*
  support for twin-engine planes
 */
void Plane::servos_twin_engine_mix(void)
{
    float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    float rud_gain = float(plane.g2.rudd_dt_gain) / 100;
    float rudder = rud_gain * SRV_Channels::get_output_scaled(SRV_Channel::k_rudder) / float(SERVO_MAX);

    if (afs.should_crash_vehicle()) {
        // when in AFS failsafe force rudder input for differential thrust to zero
        rudder = 0;
    }

    float throttle_left, throttle_right;

    if (throttle < 0 && have_reverse_thrust() && allow_reverse_thrust()) {
        // doing reverse thrust
        throttle_left  = constrain_float(throttle + 50 * rudder, -100, 0);
        throttle_right = constrain_float(throttle - 50 * rudder, -100, 0);
    } else if (throttle <= 0) {
        throttle_left  = throttle_right = 0;
    } else {
        // doing forward thrust
        throttle_left  = constrain_float(throttle + 50 * rudder, 0, 100);
        throttle_right = constrain_float(throttle - 50 * rudder, 0, 100);
    }
    if (!hal.util->get_soft_armed()) {
        if (arming.arming_required() == AP_Arming::YES_ZERO_PWM) {
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleLeft, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
            SRV_Channels::set_output_limit(SRV_Channel::k_throttleRight, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
        } else {
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, 0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, 0);
        }
    } else {
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, throttle_left);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, throttle_right);
    }
}


/*
  Set the flight control servos based on the current calculated values

  This function operates by first building up output values for
  channels using set_servo() and set_radio_out(). Using
  set_radio_out() is for when a raw PWM value of output is given which
  does not depend on any output scaling. Using set_servo() is for when
  scaling and mixing will be needed.

  Finally servos_output() is called to push the final PWM values
  for output channels
*/
void Plane::set_servos(void)
{
    // start with output corked. the cork is released when we run
    // servos_output(), which is run from all code paths in this
    // function
    SRV_Channels::cork();
    
    // this is to allow the failsafe module to deliberately crash 
    // the plane. Only used in extreme circumstances to meet the
    // OBC rules
    if (afs.should_crash_vehicle()) {
        afs.terminate_vehicle();
        return;
    }

    // do any transition updates for quadplane
    quadplane.update();    

    if (control_mode == &mode_auto && auto_state.idle_mode) {
        // special handling for balloon launch
        set_servos_idle();
        servos_output();
        return;
    }

    /*
      see if we are doing ground steering.
     */
    if (!steering_control.ground_steering) {
        // we are not at an altitude for ground steering. Set the nose
        // wheel to the rudder just in case the barometer has drifted
        // a lot
        steering_control.steering = steering_control.rudder;
    } else if (!SRV_Channels::function_assigned(SRV_Channel::k_steering)) {
        // we are within the ground steering altitude but don't have a
        // dedicated steering channel. Set the rudder to the ground
        // steering output
        steering_control.rudder = steering_control.steering;
    }

    // clear ground_steering to ensure manual control if the yaw stabilizer doesn't run
    steering_control.ground_steering = false;

    if (control_mode == &mode_training) {
        steering_control.rudder = channel_rudder->get_control_in();
    }
    
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, steering_control.rudder);
    SRV_Channels::set_output_scaled(SRV_Channel::k_steering, steering_control.steering);

    if (control_mode == &mode_manual) {
        set_servos_manual_passthrough();
    } else {
        set_servos_controlled();
    }

    // setup flap outputs
    set_servos_flaps();

#if LANDING_GEAR_ENABLED == ENABLED
    // setup landing gear output
    set_landing_gear();
#endif
    
    if (auto_throttle_mode ||
        quadplane.in_assisted_flight() ||
        quadplane.in_vtol_mode()) {
        /* only do throttle slew limiting in modes where throttle
         *  control is automatic */
        throttle_slew_limit();
    }

    if (!arming.is_armed()) {
        //Some ESCs get noisy (beep error msgs) if PWM == 0.
        //This little segment aims to avoid this.
        switch (arming.arming_required()) { 
        case AP_Arming::NO:
            //keep existing behavior: do nothing to radio_out
            //(don't disarm throttle channel even if AP_Arming class is)
            break;

        case AP_Arming::YES_ZERO_PWM:
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttle, 0);
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, 0);
            SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, 0);
            break;

        case AP_Arming::YES_MIN_PWM:
        default:
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, 0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, 0);
            break;
        }
    }

#if HIL_SUPPORT
    if (g.hil_mode == 1) {
        // get the servos to the GCS immediately for HIL
        if (HAVE_PAYLOAD_SPACE(MAVLINK_COMM_0, RC_CHANNELS_SCALED)) {
            send_servo_out(MAVLINK_COMM_0);
        }
        if (!g.hil_servos) {
            // we don't run the output mixer
            return;
        }
    }
#endif

    if (landing.get_then_servos_neutral() > 0 &&
            control_mode == &mode_auto &&
            landing.get_disarm_delay() > 0 &&
            landing.is_complete() &&
            !arming.is_armed()) {
        // after an auto land and auto disarm, set the servos to be neutral just
        // in case we're upside down or some crazy angle and straining the servos.
        if (landing.get_then_servos_neutral() == 1) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, 0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, 0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, 0);
        } else if (landing.get_then_servos_neutral() == 2) {
            SRV_Channels::set_output_limit(SRV_Channel::k_aileron, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
            SRV_Channels::set_output_limit(SRV_Channel::k_elevator, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
            SRV_Channels::set_output_limit(SRV_Channel::k_rudder, SRV_Channel::SRV_CHANNEL_LIMIT_ZERO_PWM);
        }
    }

    uint8_t override_pct;
    if (g2.ice_control.throttle_override(override_pct)) {
        // the ICE controller wants to override the throttle for starting
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, override_pct);
    }

    // run output mixer and send values to the hal for output
    servos_output();
}


/*
  run configured output mixer. This takes calculated servo_out values
  for each channel and calculates PWM values, then pushes them to
  hal.rcout
 */
void Plane::servos_output(void)
{
    SRV_Channels::cork();

    // support twin-engine aircraft
    servos_twin_engine_mix();

    // cope with tailsitters
    quadplane.tailsitter_output();
    
    // the mixers need pwm to be calculated now
    SRV_Channels::calc_pwm();
    
    // run vtail and elevon mixers
    servo_output_mixers();

    // support MANUAL_RCMASK
    if (g2.manual_rc_mask.get() != 0 && control_mode == &mode_manual) {
        SRV_Channels::copy_radio_in_out_mask(uint16_t(g2.manual_rc_mask.get()));
    }
    
    SRV_Channels::calc_pwm();

    SRV_Channels::output_ch_all();
    
    SRV_Channels::push();

    if (g2.servo_channels.auto_trim_enabled()) {
        servos_auto_trim();
    }
}

/*
  implement automatic persistent trim of control surfaces with
  AUTO_TRIM=2, only available when SERVO_RNG_ENABLE=1 as otherwise it
  would impact R/C transmitter calibration
 */
void Plane::servos_auto_trim(void)
{
    // only in auto modes and FBWA
    if (!auto_throttle_mode && control_mode != &mode_fbwa) {
        return;
    }
    if (!hal.util->get_soft_armed()) {
        return;
    }
    if (!is_flying()) {
        return;
    }
    if (quadplane.in_assisted_flight() || quadplane.in_vtol_mode()) {
        // can't auto-trim with quadplane motors running
        return;
    }
    if (abs(nav_roll_cd) > 700 || abs(nav_pitch_cd) > 700) {
        // only when close to level
        return;
    }
    uint32_t now = AP_HAL::millis();
    if (now - auto_trim.last_trim_check < 500) {
        // check twice a second. We want slow trim update
        return;
    }
    if (ahrs.groundspeed() < 8 || smoothed_airspeed < 8) {
        // only when definitely moving
        return;
    }

    // adjust trim on channels by a small amount according to I value
    float roll_I = rollController.get_pid_info().I;
    float pitch_I = pitchController.get_pid_info().I;
    
    g2.servo_channels.adjust_trim(SRV_Channel::k_aileron, roll_I);
    g2.servo_channels.adjust_trim(SRV_Channel::k_elevator, pitch_I);

    g2.servo_channels.adjust_trim(SRV_Channel::k_elevon_left,  pitch_I - roll_I);
    g2.servo_channels.adjust_trim(SRV_Channel::k_elevon_right, pitch_I + roll_I);

    g2.servo_channels.adjust_trim(SRV_Channel::k_vtail_left,  pitch_I);
    g2.servo_channels.adjust_trim(SRV_Channel::k_vtail_right, pitch_I);

    g2.servo_channels.adjust_trim(SRV_Channel::k_flaperon_left,  roll_I);
    g2.servo_channels.adjust_trim(SRV_Channel::k_flaperon_right, -roll_I);

    g2.servo_channels.adjust_trim(SRV_Channel::k_dspoilerLeft1,  pitch_I - roll_I);
    g2.servo_channels.adjust_trim(SRV_Channel::k_dspoilerLeft2,  pitch_I - roll_I);
    g2.servo_channels.adjust_trim(SRV_Channel::k_dspoilerRight1, pitch_I + roll_I);
    g2.servo_channels.adjust_trim(SRV_Channel::k_dspoilerRight2, pitch_I + roll_I);
    
    auto_trim.last_trim_check = now;

    if (now - auto_trim.last_trim_save > 10000) {
        auto_trim.last_trim_save = now;
        g2.servo_channels.save_trim();
    }
    
}
