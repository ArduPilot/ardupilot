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
    if (control_mode==AUTO) {
        if (auto_state.takeoff_complete == false && g.takeoff_throttle_slewrate != 0) {
            slewrate = g.takeoff_throttle_slewrate;
        } else if (landing.get_throttle_slewrate() != 0 && flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND) {
            slewrate = landing.get_throttle_slewrate();
        }
    }
    // if slew limit rate is set to zero then do not slew limit
    if (slewrate) {                   
        SRV_Channels::limit_slew_rate(SRV_Channel::k_throttle, slewrate, G_Dt);
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

    if (control_mode==AUTO && g.auto_fbw_steer == 42) {
        // user has throttle control
        return false;
    }

    bool gps_movement = (gps.status() >= AP_GPS::GPS_OK_FIX_2D && gps.ground_speed() >= 5);
    
    if (control_mode==AUTO && 
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
  implement a software VTail or elevon mixer. There are 4 different mixing modes
 */
void Plane::channel_output_mixer_pwm(uint8_t mixing_type, uint16_t & chan1_out, uint16_t & chan2_out) const
{
    int16_t c1, c2;
    int16_t v1, v2;

    // first get desired elevator and rudder as -500..500 values
    c1 = chan1_out - 1500;
    c2 = chan2_out - 1500;

    // apply MIXING_OFFSET to input channels using long-integer version
    //  of formula:  x = x * (g.mixing_offset/100.0 + 1.0)
    //  -100 => 2x on 'c1', 100 => 2x on 'c2'
    if (g.mixing_offset < 0) {
        c1 = (int16_t)(((int32_t)c1) * (-g.mixing_offset+100) / 100);
    } else if (g.mixing_offset > 0) {
        c2 = (int16_t)(((int32_t)c2) * (g.mixing_offset+100) / 100);
    }

    v1 = (c1 - c2) * g.mixing_gain;
    v2 = (c1 + c2) * g.mixing_gain;

    // now map to mixed output
    switch (mixing_type) {
    case MIXING_DISABLED:
        return;

    case MIXING_UPUP:
        break;

    case MIXING_UPDN:
        v2 = -v2;
        break;

    case MIXING_DNUP:
        v1 = -v1;
        break;

    case MIXING_DNDN:
        v1 = -v1;
        v2 = -v2;
        break;

    case MIXING_UPUP_SWP:
        std::swap(v1, v2);
        break;

    case MIXING_UPDN_SWP:
        v2 = -v2;
        std::swap(v1, v2);        
        break;

    case MIXING_DNUP_SWP:
        v1 = -v1;
        std::swap(v1, v2);        
        break;

    case MIXING_DNDN_SWP:
        v1 = -v1;
        v2 = -v2;
        std::swap(v1, v2);        
        break;
    }

    // scale for a 1500 center and 900..2100 range, symmetric
    v1 = constrain_int16(v1, -600, 600);
    v2 = constrain_int16(v2, -600, 600);

    chan1_out = 1500 + v1;
    chan2_out = 1500 + v2;
}

/*
  output mixer based on two channel output types
 */
void Plane::channel_output_mixer(uint8_t mixing_type, SRV_Channel::Aux_servo_function_t func1, SRV_Channel::Aux_servo_function_t func2)
{
    SRV_Channel *chan1, *chan2;
    if (!(chan1 = SRV_Channels::get_channel_for(func1)) ||
        !(chan2 = SRV_Channels::get_channel_for(func2))) {
        return;
    }

    uint16_t chan1_out, chan2_out;
    chan1_out = chan1->get_output_pwm();
    chan2_out = chan2->get_output_pwm();
    
    channel_output_mixer_pwm(mixing_type, chan1_out, chan2_out);

    chan1->set_output_pwm(chan1_out);
    chan2->set_output_pwm(chan2_out);
}


/*
  setup flaperon output channels
 */
void Plane::flaperon_update(int8_t flap_percent)
{
    if (!SRV_Channels::function_assigned(SRV_Channel::k_flaperon1) ||
        !SRV_Channels::function_assigned(SRV_Channel::k_flaperon2)) {
        return;
    }
    uint16_t ch1, ch2;
    /*
      flaperons are implemented as a mixer between aileron and a
      percentage of flaps. Flap input can come from a manual channel
      or from auto flaps.

      Use k_flaperon1 and k_flaperon2 channel trims to center servos.
      Then adjust aileron trim for level flight (note that aileron trim is affected
      by mixing gain). flapin_channel's trim is not used.
     */
     
    if (!SRV_Channels::get_output_pwm(SRV_Channel::k_aileron, ch1)) {
        return;
    }
    // The *5 is to take a percentage to a value from -500 to 500 for the mixer
    ch2 = 1500 - flap_percent * 5;
    channel_output_mixer_pwm(g.flaperon_output, ch1, ch2);
    SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_flaperon1, ch1);
    SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_flaperon2, ch2);
}

/*
  setup servos for idle mode
  Idle mode is used during balloon launch to keep servos still, apart
  from occasional wiggle to prevent freezing up
 */
void Plane::set_servos_idle(void)
{
    if (auto_state.idle_wiggle_stage == 0) {
        SRV_Channels::output_trim_all();
        return;
    }
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
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, channel_throttle->get_control_in_zero_dz());
    
    // this variant assumes you have the corresponding
    // input channel setup in your transmitter for manual control
    // of the 2nd aileron
    SRV_Channels::copy_radio_in_out(SRV_Channel::k_aileron_with_input);
    SRV_Channels::copy_radio_in_out(SRV_Channel::k_elevator_with_input);
}

/*
  old (deprecated) elevon support
 */
void Plane::set_servos_old_elevons(void)
{
    /*Elevon mode*/
    float ch1;
    float ch2;
    int16_t roll  = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
    int16_t pitch = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
    ch1 = pitch - (BOOL_TO_SIGN(g.reverse_elevons) * roll);
    ch2 = pitch + (BOOL_TO_SIGN(g.reverse_elevons) * roll);
    
    /* Differential Spoilers
       If differential spoilers are setup, then we translate
       rudder control into splitting of the two ailerons on
       the side of the aircraft where we want to induce
       additional drag.
    */
    if (SRV_Channels::function_assigned(SRV_Channel::k_dspoiler1) && SRV_Channels::function_assigned(SRV_Channel::k_dspoiler2)) {
        float ch3 = ch1;
        float ch4 = ch2;
        int16_t rudder = SRV_Channels::get_output_scaled(SRV_Channel::k_rudder);
        if (BOOL_TO_SIGN(g.reverse_elevons) * rudder < 0) {
            ch1 += abs(rudder);
            ch3 -= abs(rudder);
        } else {
            ch2 += abs(rudder);
            ch4 -= abs(rudder);
        }
        SRV_Channels::set_output_scaled(SRV_Channel::k_dspoiler1, ch3);
        SRV_Channels::set_output_scaled(SRV_Channel::k_dspoiler2, ch4);
    }
    
    // directly set the radio_out values for elevon mode
    SRV_Channels::set_output_pwm_first(SRV_Channel::k_aileron, elevon.trim1 + (BOOL_TO_SIGN(g.reverse_ch1_elevon) * (ch1 * 500.0f/ SERVO_MAX)));
    SRV_Channels::set_output_pwm_first(SRV_Channel::k_elevator, elevon.trim2 + (BOOL_TO_SIGN(g.reverse_ch2_elevon) * (ch2 * 500.0f/ SERVO_MAX)));
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
    if (g.mix_mode != 0) {
        set_servos_old_elevons();
    } else {
        // both types of secondary aileron are slaved to the roll servo out
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron_with_input,
                                        SRV_Channels::get_output_scaled(SRV_Channel::k_aileron));

        // both types of secondary elevator are slaved to the pitch servo out
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator_with_input,
                                            SRV_Channels::get_output_scaled(SRV_Channel::k_elevator));
    }

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
        } else {
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
        }
    } else if (suppress_throttle()) {
        // throttle is suppressed in auto mode
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
        if (g.throttle_suppress_manual) {
            // manual pass through of throttle while throttle is suppressed
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, channel_throttle->get_control_in_zero_dz());
        }
    } else if (g.throttle_passthru_stabilize && 
               (control_mode == STABILIZE || 
                control_mode == TRAINING ||
                control_mode == ACRO ||
                control_mode == FLY_BY_WIRE_A ||
                control_mode == AUTOTUNE) &&
               !failsafe.ch3_counter) {
        // manual pass through of throttle while in FBWA or
        // STABILIZE mode with THR_PASS_STAB set
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, channel_throttle->get_control_in_zero_dz());
    } else if ((control_mode == GUIDED || control_mode == AVOID_ADSB) &&
               guided_throttle_passthru) {
        // manual pass through of throttle while in GUIDED
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, channel_throttle->get_control_in_zero_dz());
    } else if (quadplane.in_vtol_mode()) {
        // ask quadplane code for forward throttle
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, quadplane.forward_throttle_pct());
    }

    // suppress throttle when soaring is active
    if ((control_mode == FLY_BY_WIRE_B || control_mode == CRUISE ||
        control_mode == AUTO || control_mode == LOITER) &&
        g2.soaring_controller.is_active() &&
        g2.soaring_controller.get_throttle_suppressed()) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
    }
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
    if (flapin != nullptr && !failsafe.ch3_failsafe && failsafe.ch3_counter == 0) {
        flapin->input();
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
        if (control_mode == AUTO) {
            switch (flight_stage) {
            case AP_Vehicle::FixedWing::FLIGHT_TAKEOFF:
            case AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND:
                if (g.takeoff_flap_percent != 0) {
                    auto_flap_percent = g.takeoff_flap_percent;
                }
                break;
            case AP_Vehicle::FixedWing::FLIGHT_NORMAL:
                if (auto_flap_percent != 0 && in_preLaunch_flight_stage()) {
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

    if (g.flaperon_output != MIXING_DISABLED && g.elevon_output == MIXING_DISABLED && g.mix_mode == 0) {
        flaperon_update(auto_flap_percent);
    }
}


/*
  apply vtail and elevon mixers
  the rewrites radio_out for the corresponding channels
 */
void Plane::servo_output_mixers(void)
{
    if (g.vtail_output != MIXING_DISABLED) {
        channel_output_mixer(g.vtail_output, SRV_Channel::k_elevator, SRV_Channel::k_rudder);
    } else if (g.elevon_output != MIXING_DISABLED) {
        channel_output_mixer(g.elevon_output, SRV_Channel::k_elevator, SRV_Channel::k_aileron);
        // if (both) differential spoilers setup then apply rudder
        //  control into splitting the two elevons on the side of
        //  the aircraft where we want to induce additional drag:
        uint16_t ch3, ch4;
        
        if (SRV_Channels::function_assigned(SRV_Channel::k_dspoiler1) &&
            SRV_Channels::function_assigned(SRV_Channel::k_dspoiler2) &&
            SRV_Channels::get_output_pwm(SRV_Channel::k_aileron, ch3) &&
            SRV_Channels::get_output_pwm(SRV_Channel::k_elevator, ch4)) {
            // convert rudder-servo output (-4500 to 4500) to PWM offset
            //  value (-500 to 500) and multiply by DSPOILR_RUD_RATE/100
            //  (rudder->servo_out * 500 / SERVO_MAX * dspoiler_rud_rate/100):
            int16_t ruddVal = (int16_t)(int32_t(SRV_Channels::get_output_scaled(SRV_Channel::k_rudder)) *
                                        g.dspoiler_rud_rate / (SERVO_MAX/5));
            if (ruddVal != 0) {   //if nonzero rudder then apply to spoilers
                int16_t ch1 = ch3;          //elevon 1
                int16_t ch2 = ch4;          //elevon 2
                if (ruddVal > 0) {     //apply rudder to right or left side
                    ch1 += ruddVal;
                    ch3 -= ruddVal;
                } else {
                    ch2 += ruddVal;
                    ch4 -= ruddVal;
                }
                // change elevon 1 & 2 positions; constrain min/max:
                SRV_Channels::set_output_pwm_first(SRV_Channel::k_aileron, constrain_int16(ch1, 900, 2100));
                SRV_Channels::set_output_pwm_first(SRV_Channel::k_elevator, constrain_int16(ch2, 900, 2100));
                // constrain min/max for intermediate dspoiler positions:
                ch3 = constrain_int16(ch3, 900, 2100);
                ch4 = constrain_int16(ch4, 900, 2100);
            }
            // set positions of differential spoilers (convert PWM
            //  900-2100 range to servo output (-4500 to 4500)
            //  and use function that supports rev/min/max/trim):
            SRV_Channels::set_output_scaled(SRV_Channel::k_dspoiler1,
                                                (int16_t(ch3)-1500) * (int16_t)(SERVO_MAX/300) / (int16_t)2);
            SRV_Channels::set_output_scaled(SRV_Channel::k_dspoiler2,
                                                (int16_t(ch4)-1500) * (int16_t)(SERVO_MAX/300) / (int16_t)2);
        }
    }
}

/*
  support for twin-engine planes
 */
void Plane::servos_twin_engine_mix(void)
{
    float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    float rud_gain = float(plane.g2.rudd_dt_gain) / 100;
    float rudder = rud_gain * SRV_Channels::get_output_scaled(SRV_Channel::k_rudder) / float(SERVO_MAX);
    float throttle_left, throttle_right;
    
    if (throttle < 0 && aparm.throttle_min < 0) {
        // doing reverse thrust
        throttle_left  = constrain_float(throttle + 50 * rudder, -100, 0);
        throttle_right = constrain_float(throttle - 50 * rudder, -100, 0);
    } else {
        // doing forward thrust
        throttle_left  = constrain_float(throttle + 50 * rudder, 0, 100);
        throttle_right = constrain_float(throttle - 50 * rudder, 0, 100);
    }
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, throttle_left);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, throttle_right);
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
    hal.rcout->cork();
    
    // this is to allow the failsafe module to deliberately crash 
    // the plane. Only used in extreme circumstances to meet the
    // OBC rules
    if (afs.should_crash_vehicle()) {
        afs.terminate_vehicle();
        return;
    }

    // do any transition updates for quadplane
    quadplane.update();    

    if (control_mode == AUTO && auto_state.idle_mode) {
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
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, steering_control.rudder);

    // clear ground_steering to ensure manual control if the yaw stabilizer doesn't run
    steering_control.ground_steering = false;

    if (control_mode == TRAINING) {
        steering_control.rudder = channel_rudder->get_control_in();
    }
    
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, steering_control.rudder);
    SRV_Channels::set_output_scaled(SRV_Channel::k_steering, steering_control.steering);

    if (control_mode == MANUAL) {
        set_servos_manual_passthrough();
    } else {
        set_servos_controlled();
    }

    // setup flap outputs
    set_servos_flaps();
    
    if (control_mode >= FLY_BY_WIRE_B ||
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
            break;

        case AP_Arming::YES_MIN_PWM:
        default:
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
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
            control_mode == AUTO &&
            landing.get_disarm_delay() > 0 &&
            landing.is_complete() &&
            !arming.is_armed()) {
        // after an auto land and auto disarm, set the servos to be neutral just
        // in case we're upside down or some crazy angle and straining the servos.
        if (landing.get_then_servos_neutral() == 1) {
            SRV_Channels::set_output_limit(SRV_Channel::k_aileron, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
            SRV_Channels::set_output_limit(SRV_Channel::k_elevator, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
            SRV_Channels::set_output_limit(SRV_Channel::k_rudder, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
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

    // support twin-engine aircraft
    servos_twin_engine_mix();
    
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
    hal.rcout->cork();

    // cope with tailsitters
    quadplane.tailsitter_output();
    
    // the mixers need pwm to be calculated now
    SRV_Channels::calc_pwm();
    
    // run vtail and elevon mixers
    servo_output_mixers();

    SRV_Channels::calc_pwm();
    
    SRV_Channels::output_ch_all();
    
    hal.rcout->push();

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
    if (!auto_throttle_mode && control_mode != FLY_BY_WIRE_A) {
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
    g2.servo_channels.adjust_trim(SRV_Channel::k_aileron, rollController.get_pid_info().I);
    g2.servo_channels.adjust_trim(SRV_Channel::k_elevator, pitchController.get_pid_info().I);

    auto_trim.last_trim_check = now;

    if (now - auto_trim.last_trim_save > 10000) {
        auto_trim.last_trim_save = now;
        g2.servo_channels.save_trim();
    }
    
}
