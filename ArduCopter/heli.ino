/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Traditional helicopter variables and functions

#include "heli.h"

#if FRAME_CONFIG == HELI_FRAME

#ifndef HELI_DYNAMIC_FLIGHT_SPEED_MIN
 #define HELI_DYNAMIC_FLIGHT_SPEED_MIN      500     // we are in "dynamic flight" when the speed is over 1m/s for 2 seconds
#endif

// counter to control dynamic flight profile
static int8_t heli_dynamic_flight_counter;

// heli_init - perform any special initialisation required for the tradheli
static void heli_init()
{
    attitude_control.update_feedforward_filter_rates(MAIN_LOOP_SECONDS);
    motors.set_dt(MAIN_LOOP_SECONDS);
    // force recalculation of RSC ramp rates after setting _dt
    motors.recalc_scalers();
}

// get_pilot_desired_collective - converts pilot input (from 0 ~ 1000) to a value that can be fed into the g.rc_3.servo_out function
static int16_t get_pilot_desired_collective(int16_t control_in)
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
static void check_dynamic_flight(void)
{
    if (!motors.armed() || !motors.motor_runup_complete() ||
        control_mode == LAND || (control_mode==RTL && rtl_state == Land) || (control_mode == AUTO && auto_mode == Auto_Land)) {
        heli_dynamic_flight_counter = 0;
        heli_flags.dynamic_flight = false;
        return;
    }

    bool moving = false;

    // with GPS lock use inertial nav to determine if we are moving
    if (GPS_ok()) {
        // get horizontal velocity
        float velocity = inertial_nav.get_velocity_xy();
        moving = (velocity >= HELI_DYNAMIC_FLIGHT_SPEED_MIN);
    }else{
        // with no GPS lock base it on throttle and forward lean angle
        moving = (g.rc_3.servo_out > 800 || ahrs.pitch_sensor < -1500);
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
static void update_heli_control_dynamics(void)
{
    // Use Leaky_I if we are not moving fast
    attitude_control.use_leaky_i(!heli_flags.dynamic_flight);
    
    // To-Do: Update dynamic phase angle of swashplate
}

// heli_update_landing_swash - sets swash plate flag so higher minimum is used when landed or landing
// should be called soon after update_land_detector in main code
static void heli_update_landing_swash()
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
            if (rtl_state == Land) {
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
static void heli_update_rotor_speed_targets()
{
    // get rotor control method
    uint8_t rsc_control_mode = motors.get_rsc_mode();
    int16_t rsc_control_deglitched = rotor_speed_deglitch_filter.apply(g.rc_8.control_in);

    switch (rsc_control_mode) {
        case AP_MOTORS_HELI_RSC_MODE_NONE:
            // even though pilot passes rotors speed directly to rotor ESC via receiver, motor lib needs to know if
            // rotor is spinning in case we are using direct drive tailrotor which must be spun up at same time
        case AP_MOTORS_HELI_RSC_MODE_CH8_PASSTHROUGH:
            // pass through pilot desired rotor speed
            motors.set_desired_rotor_speed(rsc_control_deglitched);
            break;
        case AP_MOTORS_HELI_RSC_MODE_SETPOINT:
            // pass setpoint through as desired rotor speed
            if (rsc_control_deglitched > 0) {
                motors.set_desired_rotor_speed(motors.get_rsc_setpoint());
            }else{
                motors.set_desired_rotor_speed(0);
            }
            break;
    }
}

#endif  // FRAME_CONFIG == HELI_FRAME
