// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *  Copyright (c) BirdsEyeView Aerobotics, LLC, 2016.
 *
 *  This program is free software: you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 3 as published
 *  by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License version 3 for more details.
 *
 *  You should have received a copy of the GNU General Public License version
 *  3 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------

static void default_dead_zones()
{
    g.rc_1.set_default_dead_zone(30);
    g.rc_2.set_default_dead_zone(30);
#if FRAME_CONFIG == HELI_FRAME
    g.rc_3.set_default_dead_zone(10);
    g.rc_4.set_default_dead_zone(15);
    g.rc_8.set_default_dead_zone(10);
#else
    g.rc_3.set_default_dead_zone(30);
    g.rc_4.set_default_dead_zone(40);
#endif
    g.rc_6.set_default_dead_zone(0);
}

static void init_rc_in()
{
    // set rc channel ranges
    g.rc_1.set_angle(ROLL_PITCH_INPUT_MAX);
    g.rc_2.set_angle(ROLL_PITCH_INPUT_MAX);
    g.rc_3.set_range(g.throttle_min, g.throttle_max);
    g.rc_4.set_angle(4500);

    g.rc_1.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
    g.rc_2.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
    g.rc_4.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

    //set auxiliary servo ranges
    g.rc_5.set_range(0,1000);
    g.rc_6.set_range(0,1000);
    g.rc_7.set_range(0,1000);
    g.rc_8.set_range(0,1000);

    // set default dead zones
    default_dead_zones();
}

 // init_rc_out -- initialise motors and check if pilot wants to perform ESC calibration
static void init_rc_out()
{
    motors.Init();                                              // motor initialisation
    motors.set_min_throttle(g.throttle_min);

    for(uint8_t i = 0; i < 5; i++) {
        delay(20);
        read_radio();
    }

    // we want the input to be scaled correctly
    g.rc_3.set_range_out(0,1000);

    // full throttle means to enter ESC calibration
    if(g.rc_3.control_in >= (g.throttle_max - 50) || (g.esc_calibrate == 2)) {
        if(g.esc_calibrate == 0) {
            // we will enter esc_calibrate mode on next reboot
            g.esc_calibrate.set_and_save(1);
            // display message on console
            cliSerial->printf_P(PSTR("Entering ESC Cal: restart APM.\n"));
            // turn on esc calibration notification
            AP_Notify::flags.esc_calibration = true;
            // block until we restart
            while(1) { delay(5); }
        }else{
            cliSerial->printf_P(PSTR("ESC Cal: passing throttle through to ESCs.\n"));
            // clear esc flag
            g.esc_calibrate.set_and_save(0);
            // pass through user throttle to escs
            init_esc();
        }
    }else{
        // did we abort the calibration?
        if(g.esc_calibrate == 1)
            g.esc_calibrate.set_and_save(0);
    }

    // enable output to motors
    pre_arm_rc_checks();
    if (ap.pre_arm_rc_check) {
        output_min();
    }
}

// output_min - enable and output lowest possible value to motors
void output_min()
{
    // enable motors
    motors.enable();
    motors.output_min();
}

//BEV used to determine if joystick is overriding certain axis when RC failsafe happens
bool throttle_input_valid()
{
    return !failsafe.radio || hal.rcin->is_overridden(2);
}
bool roll_pitch_input_valid()
{
    return !failsafe.radio || (hal.rcin->is_overridden(0) && hal.rcin->is_overridden(1));
}
bool yaw_input_valid()
{
    return !failsafe.radio || hal.rcin->is_overridden(3);
}
bool mode_input_valid()
{
    return !failsafe.radio || hal.rcin->is_overridden(4);
}
bool transition_gear_input_valid()
{
    return !failsafe.radio || (hal.rcin->is_overridden(5) && hal.rcin->is_overridden(6));
}

static void read_radio()
{
    static uint32_t last_update = 0;
    if (hal.rcin->new_input()) {
        last_update = millis();
        ap.new_radio_frame = true;
        uint16_t periods[8];
        hal.rcin->read(periods,8);
        g.rc_1.set_pwm(periods[0]);
        g.rc_2.set_pwm(periods[1]);

        set_throttle_and_failsafe(periods[2]);

        g.rc_4.set_pwm(periods[3]);
        g.rc_5.set_pwm(periods[4]);
        g.rc_6.set_pwm(periods[5]);
        g.rc_7.set_pwm(periods[6]);
        g.rc_8.set_pwm(periods[7]);

        // read channels 9 ~ 14
        for (uint8_t i=8; i<RC_MAX_CHANNELS; i++) {
            if (RC_Channel::rc_channel(i) != NULL) {
                RC_Channel::rc_channel(i)->set_pwm(RC_Channel::rc_channel(i)->read());
            }
        }

        // flag we must have an rc receiver attached
        if (!failsafe.rc_override_active) {
            ap.rc_receiver_present = true;
        }

        // update output on any aux channels, for manual passthru
        RC_Channel_aux::output_ch_all();
    }else{
        //BEV override channels as needed for joystick from GCS
        if(failsafe.rc_override_active) {
            uint16_t periods[8];
            hal.rcin->read(periods,8);
            if(hal.rcin->is_overridden(0)) {
                g.rc_1.set_pwm(periods[0]);
            }
            if(hal.rcin->is_overridden(1)) {
                g.rc_2.set_pwm(periods[1]);
            }
            if(hal.rcin->is_overridden(2)) {
                g.rc_3.set_pwm(periods[2]);
            }
            if(hal.rcin->is_overridden(3)) {
                g.rc_4.set_pwm(periods[3]);
            }
            if(hal.rcin->is_overridden(4)) {
                g.rc_5.set_pwm(periods[4]);
            }
            if(hal.rcin->is_overridden(5)) {
                g.rc_6.set_pwm(periods[5]);
            }
            if(hal.rcin->is_overridden(6)) {
                g.rc_7.set_pwm(periods[6]);
            }
            if(hal.rcin->is_overridden(7)) {
                g.rc_8.set_pwm(periods[7]);
            }
        }

        uint32_t elapsed = millis() - last_update;
        // turn on throttle failsafe if no update from the RC Radio for 500ms or 2000ms if we are using RC_OVERRIDE
        if (((!failsafe.rc_override_active && (elapsed >= FS_RADIO_TIMEOUT_MS)) || (failsafe.rc_override_active && (elapsed >= FS_RADIO_RC_OVERRIDE_TIMEOUT_MS))) &&
            (g.failsafe_throttle && motors.armed() && !failsafe.radio)) {
            Log_Write_Error(ERROR_SUBSYSTEM_RADIO, ERROR_CODE_RADIO_LATE_FRAME);
            set_failsafe_radio(true);
        }
    }
}

#define FS_COUNTER 3        // radio failsafe kicks in after 3 consecutive throttle values below failsafe_throttle_value
static void set_throttle_and_failsafe(uint16_t throttle_pwm)
{
    // if failsafe not enabled pass through throttle and exit
    if(g.failsafe_throttle == FS_THR_DISABLED) {
        g.rc_3.set_pwm(throttle_pwm);
        return;
    }

    //check for low throttle value
    if (throttle_pwm < (uint16_t)g.failsafe_throttle_value) {

        // if we are already in failsafe or motors not armed pass through throttle and exit
        if (failsafe.radio || !motors.armed()) {
            g.rc_3.set_pwm(throttle_pwm);
            return;
        }

        // check for 3 low throttle values
        // Note: we do not pass through the low throttle until 3 low throttle values are recieved
        failsafe.radio_counter++;
        if( failsafe.radio_counter >= FS_COUNTER ) {
            failsafe.radio_counter = FS_COUNTER;  // check to ensure we don't overflow the counter
            set_failsafe_radio(true);
            g.rc_3.set_pwm(throttle_pwm);   // pass through failsafe throttle
        }
    }else{
        // we have a good throttle so reduce failsafe counter
        failsafe.radio_counter--;
        if( failsafe.radio_counter <= 0 ) {
            failsafe.radio_counter = 0;   // check to ensure we don't underflow the counter

            // disengage failsafe after three (nearly) consecutive valid throttle values
            if (failsafe.radio) {
                set_failsafe_radio(false);
            }
        }
        // pass through throttle
        g.rc_3.set_pwm(throttle_pwm);
    }
}

static void trim_radio()
{
    for (uint8_t i = 0; i < 30; i++) {
        read_radio();
    }

    g.rc_1.trim();      // roll
    g.rc_2.trim();      // pitch
    g.rc_4.trim();      // yaw

    g.rc_1.save_eeprom();
    g.rc_2.save_eeprom();
    g.rc_4.save_eeprom();
}

