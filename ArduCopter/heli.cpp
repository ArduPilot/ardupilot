/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

// Traditional helicopter variables and functions

#if FRAME_CONFIG == HELI_FRAME

#ifndef HELI_DYNAMIC_FLIGHT_SPEED_MIN
 #define HELI_DYNAMIC_FLIGHT_SPEED_MIN      500     // we are in "dynamic flight" when the speed is over 1m/s for 2 seconds
#endif

// counter to control dynamic flight profile
static int8_t heli_dynamic_flight_counter;

// heli_init - perform any special initialisation required for the tradheli
void Copter::heli_init()
{
    // helicopters are always using motor interlock
    set_using_interlock(true);

    /*
      automatically set H_RSC_MIN and H_RSC_MAX from RC8_MIN and
      RC8_MAX so that when users upgrade from tradheli version 3.3 to
      3.4 they get the same throttle range as in previous versions of
      the code
     */
    if (!g.heli_servo_rsc.radio_min.load()) {
        g.heli_servo_rsc.radio_min.set_and_save(g.rc_8.radio_min.get());
    }
    if (!g.heli_servo_rsc.radio_max.load()) {
        g.heli_servo_rsc.radio_max.set_and_save(g.rc_8.radio_max.get());
    }
}

// get_pilot_desired_collective - converts pilot input (from 0 ~ 1000) to a value that can be fed into the channel_throttle->servo_out function
int16_t Copter::get_pilot_desired_collective(int16_t control_in)
{
    // return immediately if reduce collective range for manual flight has not been configured
    if (g.heli_stab_col_min == 0 && g.heli_stab_col_max == 1000) {
        return control_in;
    }

    // scale pilot input to reduced collective range
    float scalar = ((float)(g.heli_stab_col_max - g.heli_stab_col_min))/1000.0f;
    int16_t collective_out = g.heli_stab_col_min + control_in * scalar;
    collective_out = constrain_int16(collective_out, 0, 1000);
    return collective_out;
}

// heli_check_dynamic_flight - updates the dynamic_flight flag based on our horizontal velocity
// should be called at 50hz
void Copter::check_dynamic_flight(void)
{
    if (!motors.armed() || !motors.rotor_runup_complete() ||
        control_mode == LAND || (control_mode==RTL && rtl_state == RTL_Land) || (control_mode == AUTO && auto_mode == Auto_Land)) {
        heli_dynamic_flight_counter = 0;
        heli_flags.dynamic_flight = false;
        return;
    }

    bool moving = false;

    // with GPS lock use inertial nav to determine if we are moving
    if (position_ok()) {
        // get horizontal velocity
        float velocity = inertial_nav.get_velocity_xy();
        moving = (velocity >= HELI_DYNAMIC_FLIGHT_SPEED_MIN);
    }else{
        // with no GPS lock base it on throttle and forward lean angle
        moving = (motors.get_throttle() > 800.0f || ahrs.pitch_sensor < -1500);
    }

    if (!moving && sonar_enabled && sonar.status() == RangeFinder::RangeFinder_Good) {
        // when we are more than 2m from the ground with good
        // rangefinder lock consider it to be dynamic flight
        moving = (sonar.distance_cm() > 200);
    }
    
    if (moving) {
        // if moving for 2 seconds, set the dynamic flight flag
        if (!heli_flags.dynamic_flight) {
            heli_dynamic_flight_counter++;
            if (heli_dynamic_flight_counter >= 100) {
                heli_flags.dynamic_flight = true;
                heli_dynamic_flight_counter = 100;
            }
        }
    }else{
        // if not moving for 2 seconds, clear the dynamic flight flag
        if (heli_flags.dynamic_flight) {
            if (heli_dynamic_flight_counter > 0) {
                heli_dynamic_flight_counter--;
            }else{
                heli_flags.dynamic_flight = false;
            }
        }
    }
}

// update_heli_control_dynamics - pushes several important factors up into AP_MotorsHeli.
// should be run between the rate controller and the servo updates.
void Copter::update_heli_control_dynamics(void)
{
    // Use Leaky_I if we are not moving fast
    attitude_control.use_leaky_i(!heli_flags.dynamic_flight);
    
    // To-Do: Update dynamic phase angle of swashplate
}

// heli_update_landing_swash - sets swash plate flag so higher minimum is used when landed or landing
// should be called soon after update_land_detector in main code
void Copter::heli_update_landing_swash()
{
    switch(control_mode) {
        case ACRO:
        case STABILIZE:
        case DRIFT:
        case SPORT:
            // manual modes always uses full swash range
            motors.set_collective_for_landing(false);
            break;

        case LAND:
            // landing always uses limit swash range
            motors.set_collective_for_landing(true);
            break;

        case RTL:
            if (rtl_state == RTL_Land) {
                motors.set_collective_for_landing(true);
            }else{
                motors.set_collective_for_landing(!heli_flags.dynamic_flight || ap.land_complete || !ap.auto_armed);
            }
            break;

        case AUTO:
            if (auto_mode == Auto_Land) {
                motors.set_collective_for_landing(true);
            }else{
                motors.set_collective_for_landing(!heli_flags.dynamic_flight || ap.land_complete || !ap.auto_armed);
            }
            break;

        default:
            // auto and hold use limited swash when landed
            motors.set_collective_for_landing(!heli_flags.dynamic_flight || ap.land_complete || !ap.auto_armed);
            break;
    }
}

// heli_update_rotor_speed_targets - reads pilot input and passes new rotor speed targets to heli motors object
void Copter::heli_update_rotor_speed_targets()
{

    static bool rotor_runup_complete_last = false;

    // get rotor control method
    uint8_t rsc_control_mode = motors.get_rsc_mode();

    rsc_control_deglitched = rotor_speed_deglitch_filter.apply(g.rc_8.control_in);


    switch (rsc_control_mode) {
        case ROTOR_CONTROL_MODE_SPEED_PASSTHROUGH:
            // pass through pilot desired rotor speed if control input is higher than 10, creating a deadband at the bottom
            if (rsc_control_deglitched > 10) {
                motors.set_interlock(true);
                motors.set_desired_rotor_speed(rsc_control_deglitched);
            } else {
                motors.set_interlock(false);
                motors.set_desired_rotor_speed(0);
            }
            break;
        case ROTOR_CONTROL_MODE_SPEED_SETPOINT:
        case ROTOR_CONTROL_MODE_OPEN_LOOP_POWER_OUTPUT:
        case ROTOR_CONTROL_MODE_CLOSED_LOOP_POWER_OUTPUT:
            // pass setpoint through as desired rotor speed, this is almost pointless as the Setpoint serves no function in this mode
            // other than being used to create a crude estimate of rotor speed
            if (rsc_control_deglitched > 0) {
                motors.set_interlock(true);
                motors.set_desired_rotor_speed(motors.get_rsc_setpoint());
            }else{
                motors.set_interlock(false);
                motors.set_desired_rotor_speed(0);
            }
            break;
    }

    // when rotor_runup_complete changes to true, log event
    if (!rotor_runup_complete_last && motors.rotor_runup_complete()){
        Log_Write_Event(DATA_ROTOR_RUNUP_COMPLETE);
    } else if (rotor_runup_complete_last && !motors.rotor_runup_complete()){
        Log_Write_Event(DATA_ROTOR_SPEED_BELOW_CRITICAL);
    }
    rotor_runup_complete_last = motors.rotor_runup_complete();
}

// heli_radio_passthrough send RC inputs direct into motors library for use during manual passthrough for helicopter setup
void Copter::heli_radio_passthrough()
{
    motors.set_radio_passthrough(channel_roll->control_in, channel_pitch->control_in, channel_throttle->control_in, channel_yaw->control_in);
}

#endif  // FRAME_CONFIG == HELI_FRAME
