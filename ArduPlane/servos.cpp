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
void Plane::throttle_slew_limit(int16_t last_throttle)
{
    uint8_t slewrate = aparm.throttle_slewrate;
    if (control_mode==AUTO) {
        if (auto_state.takeoff_complete == false && g.takeoff_throttle_slewrate != 0) {
            slewrate = g.takeoff_throttle_slewrate;
        } else if (landing.get_throttle_slewrate() != 0 && landing.in_progress) {
            slewrate = landing.get_throttle_slewrate();
        }
    }
    // if slew limit rate is set to zero then do not slew limit
    if (slewrate) {                   
        // limit throttle change by the given percentage per second
        float temp = slewrate * G_Dt * 0.01f * fabsf(channel_throttle->get_radio_max() - channel_throttle->get_radio_min());
        // allow a minimum change of 1 PWM per cycle
        if (temp < 1) {
            temp = 1;
        }
        channel_throttle->set_radio_out(constrain_int16(channel_throttle->get_radio_out(), last_throttle - temp, last_throttle + temp));
    }
}

/*****************************************
Flap slew limit
*****************************************/
void Plane::flap_slew_limit(int8_t &last_value, int8_t &new_value)
{
    uint8_t slewrate = g.flap_slewrate;
    // if slew limit rate is set to zero then do not slew limit
    if (slewrate) {                   
        // limit flap change by the given percentage per second
        float temp = slewrate * G_Dt;
        // allow a minimum change of 1% per cycle. This means the
        // slowest flaps we can do is full change over 2 seconds
        if (temp < 1) {
            temp = 1;
        }
        new_value = constrain_int16(new_value, last_value - temp, last_value + temp);
    }
    last_value = new_value;
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
    
    if (relative_altitude_abs_cm() >= 1000) {
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
void Plane::channel_output_mixer(uint8_t mixing_type, int16_t & chan1_out, int16_t & chan2_out)const
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

void Plane::channel_output_mixer(uint8_t mixing_type, RC_Channel* chan1, RC_Channel* chan2)const
{
   int16_t ch1 = chan1->get_radio_out();
   int16_t ch2 = chan2->get_radio_out();

   channel_output_mixer(mixing_type,ch1,ch2);

   chan1->set_radio_out(ch1);
   chan2->set_radio_out(ch2);
}

/*
  setup flaperon output channels
 */
void Plane::flaperon_update(int8_t flap_percent)
{
    if (!RC_Channel_aux::function_assigned(RC_Channel_aux::k_flaperon1) ||
        !RC_Channel_aux::function_assigned(RC_Channel_aux::k_flaperon2)) {
        return;
    }
    int16_t ch1, ch2;
    /*
      flaperons are implemented as a mixer between aileron and a
      percentage of flaps. Flap input can come from a manual channel
      or from auto flaps.

      Use k_flaperon1 and k_flaperon2 channel trims to center servos.
      Then adjust aileron trim for level flight (note that aileron trim is affected
      by mixing gain). flapin_channel's trim is not used.
     */
     
    ch1 = channel_roll->get_radio_out();
    // The *5 is to take a percentage to a value from -500 to 500 for the mixer
    ch2 = 1500 - flap_percent * 5;
    channel_output_mixer(g.flaperon_output, ch1, ch2);
    RC_Channel_aux::set_radio_trimmed(RC_Channel_aux::k_flaperon1, ch1);
    RC_Channel_aux::set_radio_trimmed(RC_Channel_aux::k_flaperon2, ch2);
}

/*
  setup servos for idle mode
  Idle mode is used during balloon launch to keep servos still, apart
  from occasional wiggle to prevent freezing up
 */
void Plane::set_servos_idle(void)
{
    RC_Channel_aux::output_ch_all();
    if (auto_state.idle_wiggle_stage == 0) {
        RC_Channel::output_trim_all();
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
    channel_roll->set_servo_out(servo_value);
    channel_pitch->set_servo_out(servo_value);
    channel_rudder->set_servo_out(servo_value);
    channel_roll->calc_pwm();
    channel_pitch->calc_pwm();
    channel_rudder->calc_pwm();
    channel_throttle->output_trim();
}

/*
  return minimum throttle PWM value, taking account of throttle reversal. For reverse thrust you get the throttle off position
 */
uint16_t Plane::throttle_min(void) const
{
    if (aparm.throttle_min < 0) {
        return channel_throttle->get_radio_trim();
    }
    return channel_throttle->get_reverse() ? channel_throttle->get_radio_max() : channel_throttle->get_radio_min();
};


/*
  pass through channels in manual mode
 */
void Plane::set_servos_manual_passthrough(void)
{
    // do a direct pass through of radio values
    if (g.mix_mode == 0 || g.elevon_output != MIXING_DISABLED) {
        channel_roll->set_radio_out(channel_roll->get_radio_in());
        channel_pitch->set_radio_out(channel_pitch->get_radio_in());
    } else {
        channel_roll->set_radio_out(channel_roll->read());
        channel_pitch->set_radio_out(channel_pitch->read());
    }
    channel_throttle->set_radio_out(channel_throttle->get_radio_in());
    channel_rudder->set_radio_out(channel_rudder->get_radio_in());
    
    // setup extra channels. We want this to come from the
    // main input channel, but using the 2nd channels dead
    // zone, reverse and min/max settings. We need to use
    // pwm_to_angle_dz() to ensure we don't trim the value for the
    // deadzone of the main aileron channel, otherwise the 2nd
    // aileron won't quite follow the first one
    RC_Channel_aux::set_servo_out_for(RC_Channel_aux::k_aileron, channel_roll->pwm_to_angle_dz(0));
    RC_Channel_aux::set_servo_out_for(RC_Channel_aux::k_elevator, channel_pitch->pwm_to_angle_dz(0));
    
    // this variant assumes you have the corresponding
    // input channel setup in your transmitter for manual control
    // of the 2nd aileron
    RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_aileron_with_input);
    RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_elevator_with_input);
}

/*
  old (deprecated) elevon support
 */
void Plane::set_servos_old_elevons(void)
{
    /*Elevon mode*/
    float ch1;
    float ch2;
    ch1 = channel_pitch->get_servo_out() - (BOOL_TO_SIGN(g.reverse_elevons) * channel_roll->get_servo_out());
    ch2 = channel_pitch->get_servo_out() + (BOOL_TO_SIGN(g.reverse_elevons) * channel_roll->get_servo_out());
    
    /* Differential Spoilers
       If differential spoilers are setup, then we translate
       rudder control into splitting of the two ailerons on
       the side of the aircraft where we want to induce
       additional drag.
    */
    if (RC_Channel_aux::function_assigned(RC_Channel_aux::k_dspoiler1) && RC_Channel_aux::function_assigned(RC_Channel_aux::k_dspoiler2)) {
        float ch3 = ch1;
        float ch4 = ch2;
        if ( BOOL_TO_SIGN(g.reverse_elevons) * channel_rudder->get_servo_out() < 0) {
            ch1 += abs(channel_rudder->get_servo_out());
            ch3 -= abs(channel_rudder->get_servo_out());
        } else {
            ch2 += abs(channel_rudder->get_servo_out());
            ch4 -= abs(channel_rudder->get_servo_out());
        }
        RC_Channel_aux::set_servo_out_for(RC_Channel_aux::k_dspoiler1, ch3);
        RC_Channel_aux::set_servo_out_for(RC_Channel_aux::k_dspoiler2, ch4);
    }
    
    // directly set the radio_out values for elevon mode
    channel_roll->set_radio_out(elevon.trim1 + (BOOL_TO_SIGN(g.reverse_ch1_elevon) * (ch1 * 500.0f/ SERVO_MAX)));
    channel_pitch->set_radio_out(elevon.trim2 + (BOOL_TO_SIGN(g.reverse_ch2_elevon) * (ch2 * 500.0f/ SERVO_MAX)));
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
        
        if (channel_throttle->get_servo_out() > 0 && // demanding too much positive thrust
            throttle_watt_limit_max < max_throttle - 25 &&
            now - throttle_watt_limit_timer_ms >= 1) {
            // always allow for 25% throttle available regardless of battery status
            throttle_watt_limit_timer_ms = now;
            throttle_watt_limit_max++;
            
        } else if (channel_throttle->get_servo_out() < 0 &&
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
        if (channel_throttle->get_servo_out() > throttle_watt_limit_max && // demanding max forward thrust
            throttle_watt_limit_max > 0) { // and we're currently limiting it
            throttle_watt_limit_timer_ms = now;
            throttle_watt_limit_max--;
            
        } else if (channel_throttle->get_servo_out() < throttle_watt_limit_min && // demanding max negative thrust
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
        RC_Channel_aux::set_servo_out_for(RC_Channel_aux::k_aileron, channel_roll->get_servo_out());
        RC_Channel_aux::set_servo_out_for(RC_Channel_aux::k_aileron_with_input, channel_roll->get_servo_out());
        
        // both types of secondary elevator are slaved to the pitch servo out
        RC_Channel_aux::set_servo_out_for(RC_Channel_aux::k_elevator, channel_pitch->get_servo_out());
        RC_Channel_aux::set_servo_out_for(RC_Channel_aux::k_elevator_with_input, channel_pitch->get_servo_out());

        // push out the PWM values
        channel_roll->calc_pwm();
        channel_pitch->calc_pwm();
    }

    channel_rudder->calc_pwm();
    
    // convert 0 to 100% (or -100 to +100) into PWM
    int8_t min_throttle = aparm.throttle_min.get();
    int8_t max_throttle = aparm.throttle_max.get();
    
    if (min_throttle < 0 && !allow_reverse_thrust()) {
        // reverse thrust is available but inhibited.
        min_throttle = 0;
    }
    
    if (control_mode == AUTO) {
        if (landing.get_stage() == AP_Landing::STAGE_FINAL) {
            min_throttle = 0;
        }
        
        if (flight_stage == AP_Vehicle::FixedWing::FLIGHT_TAKEOFF || flight_stage == AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND) {
            if(aparm.takeoff_throttle_max != 0) {
                max_throttle = aparm.takeoff_throttle_max;
            } else {
                max_throttle = aparm.throttle_max;
            }
        }
    }

    // apply watt limiter
    throttle_watt_limiter(min_throttle, max_throttle);

    channel_throttle->set_servo_out(constrain_int16(channel_throttle->get_servo_out(), 
                                                    min_throttle,
                                                    max_throttle));
    
    if (!hal.util->get_soft_armed()) {
        channel_throttle->set_servo_out(0);
        channel_throttle->calc_pwm();
    } else if (suppress_throttle()) {
        // throttle is suppressed in auto mode
        channel_throttle->set_servo_out(0);
        if (g.throttle_suppress_manual) {
            // manual pass through of throttle while throttle is suppressed
            channel_throttle->set_radio_out(channel_throttle->get_radio_in());
        } else {
            channel_throttle->calc_pwm();
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
        channel_throttle->set_radio_out(channel_throttle->get_radio_in());
    } else if ((control_mode == GUIDED || control_mode == AVOID_ADSB) &&
               guided_throttle_passthru) {
        // manual pass through of throttle while in GUIDED
        channel_throttle->set_radio_out(channel_throttle->get_radio_in());
    } else if (quadplane.in_vtol_mode()) {
        // ask quadplane code for forward throttle
        channel_throttle->set_servo_out(quadplane.forward_throttle_pct());
        channel_throttle->calc_pwm();
    } else {
        // normal throttle calculation based on servo_out
        channel_throttle->calc_pwm();
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
    static int8_t last_auto_flap;
    static int8_t last_manual_flap;

    // work out any manual flap input
    RC_Channel *flapin = RC_Channel::rc_channel(g.flapin_channel-1);
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
            case AP_Vehicle::FixedWing::FLIGHT_LAND:
                if (landing.get_flap_percent() != 0) {
                    auto_flap_percent = landing.get_flap_percent();
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

    flap_slew_limit(last_auto_flap, auto_flap_percent);
    flap_slew_limit(last_manual_flap, manual_flap_percent);

    RC_Channel_aux::set_servo_out_for(RC_Channel_aux::k_flap_auto, auto_flap_percent);
    RC_Channel_aux::set_servo_out_for(RC_Channel_aux::k_flap, manual_flap_percent);

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
        channel_output_mixer(g.vtail_output, channel_pitch, channel_rudder);
    } else if (g.elevon_output != MIXING_DISABLED) {
        channel_output_mixer(g.elevon_output, channel_pitch, channel_roll);
        // if (both) differential spoilers setup then apply rudder
        //  control into splitting the two elevons on the side of
        //  the aircraft where we want to induce additional drag:
        if (RC_Channel_aux::function_assigned(RC_Channel_aux::k_dspoiler1) &&
            RC_Channel_aux::function_assigned(RC_Channel_aux::k_dspoiler2)) {
            int16_t ch3 = channel_roll->get_radio_out();    //diff spoiler 1
            int16_t ch4 = channel_pitch->get_radio_out();   //diff spoiler 2
            // convert rudder-servo output (-4500 to 4500) to PWM offset
            //  value (-500 to 500) and multiply by DSPOILR_RUD_RATE/100
            //  (rudder->servo_out * 500 / SERVO_MAX * dspoiler_rud_rate/100):
            int16_t ruddVal = (int16_t)((int32_t)(channel_rudder->get_servo_out()) *
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
                channel_roll->set_radio_out(constrain_int16(ch1, 900, 2100));
                channel_pitch->set_radio_out(constrain_int16(ch2, 900, 2100));
                // constrain min/max for intermediate dspoiler positions:
                ch3 = constrain_int16(ch3, 900, 2100);
                ch4 = constrain_int16(ch4, 900, 2100);
            }
            // set positions of differential spoilers (convert PWM
            //  900-2100 range to servo output (-4500 to 4500)
            //  and use function that supports rev/min/max/trim):
            RC_Channel_aux::set_servo_out_for(RC_Channel_aux::k_dspoiler1,
                                              (ch3-(int16_t)1500) * (int16_t)(SERVO_MAX/300) / (int16_t)2);
            RC_Channel_aux::set_servo_out_for(RC_Channel_aux::k_dspoiler2,
                                              (ch4-(int16_t)1500) * (int16_t)(SERVO_MAX/300) / (int16_t)2);
        }
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
    hal.rcout->cork();
    
    // this is to allow the failsafe module to deliberately crash 
    // the plane. Only used in extreme circumstances to meet the
    // OBC rules
    if (afs.should_crash_vehicle()) {
        afs.terminate_vehicle();
        return;
    }

    int16_t last_throttle = channel_throttle->get_radio_out();

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
    } else if (!RC_Channel_aux::function_assigned(RC_Channel_aux::k_steering)) {
        // we are within the ground steering altitude but don't have a
        // dedicated steering channel. Set the rudder to the ground
        // steering output
        steering_control.rudder = steering_control.steering;
    }
    channel_rudder->set_servo_out(steering_control.rudder);

    // clear ground_steering to ensure manual control if the yaw stabilizer doesn't run
    steering_control.ground_steering = false;

    RC_Channel_aux::set_servo_out_for(RC_Channel_aux::k_rudder, steering_control.rudder);
    RC_Channel_aux::set_servo_out_for(RC_Channel_aux::k_steering, steering_control.steering);

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
        throttle_slew_limit(last_throttle);
    }

    if (control_mode == TRAINING) {
        // copy rudder in training mode
        channel_rudder->set_radio_out(channel_rudder->get_radio_in());
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
            channel_throttle->set_servo_out(0);
            channel_throttle->set_radio_out(0);
            break;

        case AP_Arming::YES_MIN_PWM:
        default:
            channel_throttle->set_servo_out(0);
            channel_throttle->set_radio_out(throttle_min());
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
            channel_roll->set_radio_out(channel_roll->get_radio_trim());
            channel_pitch->set_radio_out(channel_pitch->get_radio_trim());
            channel_rudder->set_radio_out(channel_rudder->get_radio_trim());
        } else if (landing.get_then_servos_neutral() == 2) {
            channel_roll->disable_out();
            channel_pitch->disable_out();
            channel_rudder->disable_out();
        }
    }

    uint8_t override_pct;
    if (g2.ice_control.throttle_override(override_pct)) {
        // the ICE controller wants to override the throttle for starting
        channel_throttle->set_servo_out(override_pct);
        channel_throttle->calc_pwm();
    }

    // allow for secondary throttle
    RC_Channel_aux::set_servo_out_for(RC_Channel_aux::k_throttle, channel_throttle->get_servo_out());
    
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

    // to enable the throttle slew rate to work we need to remember
    // and restore the throttle radio_out
    uint16_t thr_radio_out_saved = channel_throttle->get_radio_out();
    
    // remap servo output to SERVO* ranges if enabled
    g2.servo_channels.remap_servo_output();

    // run vtail and elevon mixers
    servo_output_mixers();

    channel_roll->output();
    channel_pitch->output();
    channel_throttle->output();
    channel_rudder->output();

    if (!afs.should_crash_vehicle()) {
        RC_Channel_aux::output_ch_all();
    }
    
    hal.rcout->push();

    // restore throttle radio out
    channel_throttle->set_radio_out(thr_radio_out_saved);
    
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
    if (!g2.servo_channels.enabled()) {
        // only possible with SERVO_RNG_ENABLE=1
        return;
    }
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
    g2.servo_channels.adjust_trim(channel_roll->get_ch_out(), rollController.get_pid_info().I);
    g2.servo_channels.adjust_trim(channel_pitch->get_ch_out(), pitchController.get_pid_info().I);

    auto_trim.last_trim_check = now;

    if (now - auto_trim.last_trim_save > 10000) {
        auto_trim.last_trim_save = now;
        g2.servo_channels.save_trim();
    }
    
}
