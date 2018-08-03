/*
  OreoLED PX4 driver
*/
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

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include <AP_BoardConfig/AP_BoardConfig.h>
#include "OreoLED_PX4.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <drivers/drv_oreoled.h>
#include <stdio.h>
#include <errno.h>
#include "AP_Notify.h"

#define OREOLED_BACKLEFT                0       // back left led instance number
#define OREOLED_BACKRIGHT               1       // back right led instance number
#define OREOLED_FRONTRIGHT              2       // front right led instance number
#define OREOLED_FRONTLEFT               3       // front left led instance number
#define PERIOD_SLOW                     800     // slow flash rate
#define PERIOD_FAST                     500     // fast flash rate
#define PERIOD_SUPER                    150     // super fast rate
#define PO_ALTERNATE                    180     // 180 degree phase offset

extern const AP_HAL::HAL& hal;

// constructor
OreoLED_PX4::OreoLED_PX4(uint8_t theme): NotifyDevice(),
    _oreoled_fd(-1),
    _oreo_theme(theme)
{
    // initialise desired and sent state
    memset(_state_desired,0,sizeof(_state_desired));
    memset(_state_sent,0,sizeof(_state_sent));
}

extern "C" int oreoled_main(int, char **);


//
// Initialize the LEDs
//
bool OreoLED_PX4::init()
{

#if defined(CONFIG_ARCH_BOARD_PX4FMU_V2)
    if (!AP_BoardConfig::px4_start_driver(oreoled_main, "oreoled", "start autoupdate")) {
        hal.console->printf("Unable to start oreoled driver\n");
    } else {
        // give it time to initialise
        hal.scheduler->delay(500);
    }
#endif

    // open the device
    _oreoled_fd = open(OREOLED0_DEVICE_PATH, O_RDWR);
    if (_oreoled_fd == -1) {
        hal.console->printf("Unable to open " OREOLED0_DEVICE_PATH);
        _overall_health = false;
    } else {
        // set overall health
        _overall_health = true;
        // register timer
        hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&OreoLED_PX4::update_timer, void));
    }

    // return health
    return _overall_health;
}


// UPDATE device according to timed_updated. Called at 50Hz
void OreoLED_PX4::update()
{

    if (!_overall_health) { return; }       // don't go any further if LED driver reports unhealthy

    if (slow_counter()) { return; }         // slow rate from 50hz to 10hz

    sync_counter();                         // syncronizes LEDs every 10 seconds

    if (mode_firmware_update()) { return; } // don't go any further if the Pixhawk is in firmware update

    if (mode_init()) { return; }            // don't go any further if the Pixhawk is initializing

    if (mode_failsafe_radio()) { return; }  // don't go any further if the Pixhawk is is in radio failsafe

    set_standard_colors();                  // set the rear LED standard colors as described above

    if (mode_failsafe_batt()) { return; }   // stop here if the battery is low.

    if (_pattern_override) { return; }      // stop here if in mavlink LED control override.

    if (mode_auto_flight()) { return; }     // stop here if in an autopilot mode.

    mode_pilot_flight();                    // stop here if in an pilot controlled mode.
}

// Slow the update rate from 50hz to 10hz
// Returns true if counting up
// Returns false and resets one counter hits 5
bool OreoLED_PX4::slow_counter()
{
    static uint8_t update_counter;

    update_counter++;
    if (update_counter < 5) {
        return true;
    } else {
        update_counter = 0;
        return false;
    }
}


// Calls resyncing the LEDs every 10 seconds
// Always returns false, no action needed.
void OreoLED_PX4::sync_counter()
{
    static uint8_t counter = 80;

    counter++;
    if (counter > 100) {
        counter = 0;
        send_sync();
    }
}


// Procedure for when Pixhawk is in FW update / bootloader
// Makes all LEDs go into color cycle mode
// Returns true if firmware update in progress. False if not
bool OreoLED_PX4::mode_firmware_update()
{
    if (AP_Notify::flags.firmware_update) {
        set_macro(OREOLED_INSTANCE_ALL, OREOLED_PARAM_MACRO_COLOUR_CYCLE);
        return true;
    } else {
        return false;
    }
}


// Makes all LEDs rapidly strobe blue while gyros initialize.
bool OreoLED_PX4::mode_init()
{
    if (AP_Notify::flags.initialising) {
        set_rgb(OREOLED_INSTANCE_ALL, OREOLED_PATTERN_STROBE, 0, 0, 255,0,0,0,PERIOD_SUPER,0);
        return true;
    } else { 
        return false;
    }
}


// Procedure for when Pixhawk is in radio failsafe
// LEDs perform alternating Red X pattern
bool OreoLED_PX4::mode_failsafe_radio()
{
    if (AP_Notify::flags.failsafe_radio) {
        set_rgb(OREOLED_FRONTLEFT, OREOLED_PATTERN_STROBE, 255, 0, 0,0,0,0,PERIOD_SLOW,0);
        set_rgb(OREOLED_FRONTRIGHT, OREOLED_PATTERN_STROBE, 255, 0, 0,0,0,0,PERIOD_SLOW,PO_ALTERNATE);
        set_rgb(OREOLED_BACKLEFT, OREOLED_PATTERN_STROBE, 255, 0, 0,0,0,0,PERIOD_SLOW,PO_ALTERNATE);
        set_rgb(OREOLED_BACKRIGHT, OREOLED_PATTERN_STROBE, 255, 0, 0,0,0,0,PERIOD_SLOW,0);
    }
    return AP_Notify::flags.failsafe_radio;
}


// Procedure to set standard rear LED colors in aviation theme
// Back LEDS White for normal, yellow for GPS not usable, purple for EKF bad]
// Returns true GPS or EKF problem, returns false if all ok
bool OreoLED_PX4::set_standard_colors()
{
    if (!(AP_Notify::flags.gps_fusion)) {
        _rear_color_r = 255;
        _rear_color_g = 50;
        _rear_color_b = 0;
        return true;

    } else if (AP_Notify::flags.ekf_bad){
        _rear_color_r = 255;
        _rear_color_g = 0;
        _rear_color_b = 255;
        return true;

    } else {
        _rear_color_r = 255;
        _rear_color_g = 255;
        _rear_color_b = 255;
        return false;
    }
}


// Procedure to set low battery LED output
// Colors standard
// Fast strobe alternating front/back
bool OreoLED_PX4::mode_failsafe_batt()
{
    if (AP_Notify::flags.failsafe_battery){

        switch (_oreo_theme) {
            case OreoLED_Aircraft:
                set_rgb(OREOLED_FRONTLEFT, OREOLED_PATTERN_STROBE, 255, 0, 0,0,0,0,PERIOD_FAST,0);
                set_rgb(OREOLED_FRONTRIGHT, OREOLED_PATTERN_STROBE, 0, 255, 0,0,0,0,PERIOD_FAST,0);
                set_rgb(OREOLED_BACKLEFT, OREOLED_PATTERN_STROBE, _rear_color_r, _rear_color_g, _rear_color_b,0,0,0,PERIOD_FAST,PO_ALTERNATE);
                set_rgb(OREOLED_BACKRIGHT, OREOLED_PATTERN_STROBE, _rear_color_r, _rear_color_g, _rear_color_b,0,0,0,PERIOD_FAST,PO_ALTERNATE);
                break;

            case OreoLED_Automobile:
                set_rgb(OREOLED_FRONTLEFT, OREOLED_PATTERN_STROBE, 255, 255, 255,0,0,0,PERIOD_FAST,0);
                set_rgb(OREOLED_FRONTRIGHT, OREOLED_PATTERN_STROBE, 255, 255, 255,0,0,0,PERIOD_FAST,0);
                set_rgb(OREOLED_BACKLEFT, OREOLED_PATTERN_STROBE, 255, 0, 0,0,0,0,PERIOD_FAST,PO_ALTERNATE);
                set_rgb(OREOLED_BACKRIGHT, OREOLED_PATTERN_STROBE, 255, 0, 0,0,0,0,PERIOD_FAST,PO_ALTERNATE);
                break;

            default:
                set_rgb(OREOLED_FRONTLEFT, OREOLED_PATTERN_STROBE, 255, 255, 255,0,0,0,PERIOD_FAST,0);
                set_rgb(OREOLED_FRONTRIGHT, OREOLED_PATTERN_STROBE, 255, 255, 255,0,0,0,PERIOD_FAST,0);
                set_rgb(OREOLED_BACKLEFT, OREOLED_PATTERN_STROBE, 255, 0, 0,0,0,0,PERIOD_FAST,PO_ALTERNATE);
                set_rgb(OREOLED_BACKRIGHT, OREOLED_PATTERN_STROBE, 255, 0, 0,0,0,0,PERIOD_FAST,PO_ALTERNATE);
                break;
        }
    }
    return AP_Notify::flags.failsafe_battery;
}


// Procedure for when Pixhawk is in an autopilot mode
// Makes all LEDs strobe super fast using standard colors
bool OreoLED_PX4::mode_auto_flight()
{
    switch (_oreo_theme) {

        case OreoLED_Aircraft:
            set_rgb(OREOLED_FRONTLEFT, OREOLED_PATTERN_STROBE, 255, 0, 0,0,0,0,PERIOD_SUPER,0);
            set_rgb(OREOLED_FRONTRIGHT, OREOLED_PATTERN_STROBE, 0, 255, 0,0,0,0,PERIOD_SUPER,0);
            if ((AP_Notify::flags.pre_arm_check && AP_Notify::flags.pre_arm_gps_check) || AP_Notify::flags.armed) {
                set_rgb(OREOLED_BACKLEFT, OREOLED_PATTERN_STROBE, _rear_color_r, _rear_color_g, _rear_color_b,0,0,0,PERIOD_SUPER,PO_ALTERNATE);
                set_rgb(OREOLED_BACKRIGHT, OREOLED_PATTERN_STROBE, _rear_color_r, _rear_color_g, _rear_color_b,0,0,0,PERIOD_SUPER,PO_ALTERNATE);
            } else {
                set_rgb(OREOLED_BACKLEFT, OREOLED_PATTERN_SOLID, _rear_color_r, _rear_color_g, _rear_color_b);
                set_rgb(OREOLED_BACKRIGHT, OREOLED_PATTERN_SOLID, _rear_color_r, _rear_color_g, _rear_color_b);
            }
            break;

        case OreoLED_Automobile:
            set_rgb(OREOLED_FRONTLEFT, OREOLED_PATTERN_STROBE, 255, 255, 255,0,0,0,PERIOD_SUPER,0);
            set_rgb(OREOLED_FRONTRIGHT, OREOLED_PATTERN_STROBE, 255, 255, 255,0,0,0,PERIOD_SUPER,0);
            set_rgb(OREOLED_BACKLEFT, OREOLED_PATTERN_STROBE, 255, 0, 0,0,0,0,PERIOD_SUPER,PO_ALTERNATE);
            set_rgb(OREOLED_BACKRIGHT, OREOLED_PATTERN_STROBE, 255, 0, 0,0,0,0,PERIOD_SUPER,PO_ALTERNATE);
            break;

        default:
            set_rgb(OREOLED_FRONTLEFT, OREOLED_PATTERN_STROBE, 255, 255, 255,0,0,0,PERIOD_SUPER,0);
            set_rgb(OREOLED_FRONTRIGHT, OREOLED_PATTERN_STROBE, 255, 255, 255,0,0,0,PERIOD_SUPER,0);
            set_rgb(OREOLED_BACKLEFT, OREOLED_PATTERN_STROBE, 255, 0, 0,0,0,0,PERIOD_SUPER,PO_ALTERNATE);
            set_rgb(OREOLED_BACKRIGHT, OREOLED_PATTERN_STROBE, 255, 0, 0,0,0,0,PERIOD_SUPER,PO_ALTERNATE);
            break;
    }    

    return AP_Notify::flags.autopilot_mode;
}


// Procedure for when Pixhawk is in a pilot controlled mode
// All LEDs use standard pattern and colors
bool OreoLED_PX4::mode_pilot_flight()
{
    switch (_oreo_theme) {

        case OreoLED_Aircraft:
            set_rgb(OREOLED_FRONTLEFT, OREOLED_PATTERN_SOLID, 255, 0, 0);
            set_rgb(OREOLED_FRONTRIGHT, OREOLED_PATTERN_SOLID, 0, 255, 0);
            if ((AP_Notify::flags.pre_arm_check && AP_Notify::flags.pre_arm_gps_check) || AP_Notify::flags.armed) {
                set_rgb(OREOLED_BACKLEFT, OREOLED_PATTERN_STROBE, _rear_color_r, _rear_color_g, _rear_color_b,0,0,0,PERIOD_FAST,0);
                set_rgb(OREOLED_BACKRIGHT, OREOLED_PATTERN_STROBE, _rear_color_r, _rear_color_g, _rear_color_b,0,0,0,PERIOD_FAST,PO_ALTERNATE);
            } else {
                set_rgb(OREOLED_BACKLEFT, OREOLED_PATTERN_SOLID, _rear_color_r, _rear_color_g, _rear_color_b);
                set_rgb(OREOLED_BACKRIGHT, OREOLED_PATTERN_SOLID, _rear_color_r, _rear_color_g, _rear_color_b);
            }
            break;

        case OreoLED_Automobile:
            set_rgb(OREOLED_FRONTLEFT, OREOLED_PATTERN_SOLID, 255, 255, 255);
            set_rgb(OREOLED_FRONTRIGHT, OREOLED_PATTERN_SOLID, 255, 255, 255);
            set_rgb(OREOLED_BACKLEFT, OREOLED_PATTERN_SOLID, 255, 0, 0);
            set_rgb(OREOLED_BACKRIGHT, OREOLED_PATTERN_SOLID, 255, 0, 0);
            break;

        default:
            set_rgb(OREOLED_FRONTLEFT, OREOLED_PATTERN_SOLID, 255, 255, 255);
            set_rgb(OREOLED_FRONTRIGHT, OREOLED_PATTERN_SOLID, 255, 255, 255);
            set_rgb(OREOLED_BACKLEFT, OREOLED_PATTERN_SOLID, 255, 0, 0);
            set_rgb(OREOLED_BACKRIGHT, OREOLED_PATTERN_SOLID, 255, 0, 0);
            break;
    }

    return true;
}


// set_rgb - Solid color settings only
void OreoLED_PX4::set_rgb(uint8_t instance, uint8_t red, uint8_t green, uint8_t blue)
{
	set_rgb(instance, OREOLED_PATTERN_SOLID, red, green, blue);
}


// set_rgb - Set a color and selected pattern.
void OreoLED_PX4::set_rgb(uint8_t instance, oreoled_pattern pattern, uint8_t red, uint8_t green, uint8_t blue)
{
    // get semaphore
    _state_desired_semaphore = true;

    // check for all instances
    if (instance == OREOLED_INSTANCE_ALL) {
        // store desired rgb for all LEDs
        for (uint8_t i=0; i<OREOLED_NUM_LEDS; i++) {
            _state_desired[i].set_rgb(pattern, red, green, blue);
            if (!(_state_desired[i] == _state_sent[i])) {
                _send_required = true;
            }
        }
    } else if (instance < OREOLED_NUM_LEDS) {
        // store desired rgb for one LED
        _state_desired[instance].set_rgb(pattern, red, green, blue);
        if (!(_state_desired[instance] == _state_sent[instance])) {
            _send_required = true;
        }
    }

    // release semaphore
    _state_desired_semaphore = false;
}


// set_rgb - Sets a color, pattern, and uses extended options for amplitude, period, and phase offset
void OreoLED_PX4::set_rgb(uint8_t instance, oreoled_pattern pattern, uint8_t red, uint8_t green, uint8_t blue,
        uint8_t amplitude_red, uint8_t amplitude_green, uint8_t amplitude_blue,
        uint16_t period, uint16_t phase_offset)
{
    // get semaphore
    _state_desired_semaphore = true;

    // check for all instances
    if (instance == OREOLED_INSTANCE_ALL) {
        // store desired rgb for all LEDs
        for (uint8_t i=0; i<OREOLED_NUM_LEDS; i++) {
            _state_desired[i].set_rgb(pattern, red, green, blue, amplitude_red, amplitude_green, amplitude_blue, period, phase_offset);
            if (!(_state_desired[i] == _state_sent[i])) {
                _send_required = true;
            }
        }
    } else if (instance < OREOLED_NUM_LEDS) {
        // store desired rgb for one LED
        _state_desired[instance].set_rgb(pattern, red, green, blue, amplitude_red, amplitude_green, amplitude_blue, period, phase_offset);
        if (!(_state_desired[instance] == _state_sent[instance])) {
            _send_required = true;
        }
    }

    // release semaphore
    _state_desired_semaphore = false;
}


// set_macro - set macro for one or all LEDs
void OreoLED_PX4::set_macro(uint8_t instance, oreoled_macro macro)
{
    // return immediately if no healthy leds
    if (!_overall_health) {
        return;
    }

    // set semaphore
    _state_desired_semaphore = true;

    // check for all instances
    if (instance == OREOLED_INSTANCE_ALL) {
        // store desired macro for all LEDs
        for (uint8_t i=0; i<OREOLED_NUM_LEDS; i++) {
            _state_desired[i].set_macro(macro);
            if (!(_state_desired[i] == _state_sent[i])) {
                _send_required = true;
            }
        }
    } else if (instance < OREOLED_NUM_LEDS) {
        // store desired macro for one LED
        _state_desired[instance].set_macro(macro);
        if (!(_state_desired[instance] == _state_sent[instance])) {
            _send_required = true;
        }
    }

    // release semaphore
    _state_desired_semaphore = false;
}

// send_sync - force a syncronisation of the all LED's
void OreoLED_PX4::send_sync()
{
    // return immediately if no healthy leds
    if (!_overall_health) {
        return;
    }

    // set semaphore
    _state_desired_semaphore = true;

    // store desired macro for all LEDs
    for (uint8_t i=0; i<OREOLED_NUM_LEDS; i++) {
        _state_desired[i].send_sync();
        if (!(_state_desired[i] == _state_sent[i])) {
            _send_required = true;
        }
    }

    // release semaphore
    _state_desired_semaphore = false;
}


// Clear the desired state
void OreoLED_PX4::clear_state(void)
{
    // set semaphore
     _state_desired_semaphore = true;

    for (uint8_t i=0; i<OREOLED_NUM_LEDS; i++) {
        _state_desired[i].clear_state();
    }

    _send_required = false;

    // release semaphore
    _state_desired_semaphore = false;
}


// update_timer - called by scheduler and updates PX4 driver with commands
void OreoLED_PX4::update_timer(void)
{
    // exit immediately if unhealthy
    if (!_overall_health) {
        return;
    }

    // exit immediately if send not required, or state is being updated
    if (!_send_required || _state_desired_semaphore) {
        return;
    }

    // for each LED
    for (uint8_t i=0; i<OREOLED_NUM_LEDS; i++) {

        // check for state change
        if (!(_state_desired[i] == _state_sent[i])) {
            switch (_state_desired[i].mode) {
                case OREOLED_MODE_MACRO:
                    {
                    oreoled_macrorun_t macro_run = {i, _state_desired[i].macro};
                    ioctl(_oreoled_fd, OREOLED_RUN_MACRO, (unsigned long)&macro_run);
                    }
                    break;
                case OREOLED_MODE_RGB:
                    {
                    oreoled_rgbset_t rgb_set = {i, _state_desired[i].pattern, _state_desired[i].red, _state_desired[i].green, _state_desired[i].blue};
                    ioctl(_oreoled_fd, OREOLED_SET_RGB, (unsigned long)&rgb_set);
                    }
                    break;
                case OREOLED_MODE_RGB_EXTENDED:
                    {
                    oreoled_cmd_t cmd;
                    memset(&cmd, 0, sizeof(oreoled_cmd_t));
                    cmd.led_num = i;
                    cmd.buff[0] = _state_desired[i].pattern;
                    cmd.buff[1] = OREOLED_PARAM_BIAS_RED;
                    cmd.buff[2] = _state_desired[i].red;
                    cmd.buff[3] = OREOLED_PARAM_BIAS_GREEN;
                    cmd.buff[4] = _state_desired[i].green;
                    cmd.buff[5] = OREOLED_PARAM_BIAS_BLUE;
                    cmd.buff[6] = _state_desired[i].blue;
                    cmd.buff[7] = OREOLED_PARAM_AMPLITUDE_RED;
                    cmd.buff[8] = _state_desired[i].amplitude_red;
                    cmd.buff[9] = OREOLED_PARAM_AMPLITUDE_GREEN;
                    cmd.buff[10] = _state_desired[i].amplitude_green;
                    cmd.buff[11] = OREOLED_PARAM_AMPLITUDE_BLUE;
                    cmd.buff[12] = _state_desired[i].amplitude_blue;
                    // Note: The Oreo LED controller expects to receive uint16 values
                    // in little endian order
                    cmd.buff[13] = OREOLED_PARAM_PERIOD;
                    cmd.buff[14] = (_state_desired[i].period & 0xFF00) >> 8;
                    cmd.buff[15] = (_state_desired[i].period & 0x00FF);
                    cmd.buff[16] = OREOLED_PARAM_PHASEOFFSET;
                    cmd.buff[17] = (_state_desired[i].phase_offset & 0xFF00) >> 8;
                    cmd.buff[18] = (_state_desired[i].phase_offset & 0x00FF);
                    cmd.num_bytes = 19;
                    ioctl(_oreoled_fd, OREOLED_SEND_BYTES, (unsigned long)&cmd);
                    }
                    break;
                case OREOLED_MODE_SYNC:
                    {
                    ioctl(_oreoled_fd, OREOLED_FORCE_SYNC, 0);
                    }
                    break;
                default:
                    break;
            };
            // save state change
            _state_sent[i] = _state_desired[i];
        }
    }

    // flag updates sent
    _send_required = false;
}


// Handle an LED_CONTROL mavlink message
void OreoLED_PX4::handle_led_control(mavlink_message_t *msg)
{
    // exit immediately if unhealthy
    if (!_overall_health) {
        return;
    }

    // decode mavlink message
    mavlink_led_control_t packet;
    mavlink_msg_led_control_decode(msg, &packet);

    // exit immediately if instance is invalid
    if (packet.instance >= OREOLED_NUM_LEDS && packet.instance != OREOLED_INSTANCE_ALL) {
        return;
    }

    // if pattern is OFF, we clear pattern override so normal lighting should resume
    if (packet.pattern == LED_CONTROL_PATTERN_OFF) {
        _pattern_override = 0;
        clear_state();
        return;
    }

    if (packet.pattern == LED_CONTROL_PATTERN_CUSTOM) {
        // Here we handle two different "sub commands",
        // depending on the bytes in the first CUSTOM_HEADER_LENGTH
        // of the custom pattern byte buffer

        // Return if we don't have at least CUSTOM_HEADER_LENGTH bytes
        if (packet.custom_len < CUSTOM_HEADER_LENGTH) {
            return;
        }

        // check for the RGB0 sub-command
        if (memcmp(packet.custom_bytes, "RGB0", CUSTOM_HEADER_LENGTH) == 0) {
            // check to make sure the total length matches the length of the RGB0 command + data values
            if (packet.custom_len != CUSTOM_HEADER_LENGTH + 4) {
                return;
            }

            // check for valid pattern id
            if (packet.custom_bytes[CUSTOM_HEADER_LENGTH] >= OREOLED_PATTERN_ENUM_COUNT) {
            return;
            }

            // convert the first byte after the command to a oreoled_pattern
            oreoled_pattern pattern = (oreoled_pattern)packet.custom_bytes[CUSTOM_HEADER_LENGTH];

            // call the set_rgb function, using the rest of the bytes as the RGB values
            set_rgb(packet.instance, pattern, packet.custom_bytes[CUSTOM_HEADER_LENGTH + 1], packet.custom_bytes[CUSTOM_HEADER_LENGTH + 2], packet.custom_bytes[CUSTOM_HEADER_LENGTH + 3]);

        } else if (memcmp(packet.custom_bytes, "RGB1", CUSTOM_HEADER_LENGTH) == 0) { // check for the RGB1 sub-command

            // check to make sure the total length matches the length of the RGB1 command + data values
            if (packet.custom_len != CUSTOM_HEADER_LENGTH + 11) {
                return;
            }

            // check for valid pattern id
            if (packet.custom_bytes[CUSTOM_HEADER_LENGTH] >= OREOLED_PATTERN_ENUM_COUNT) {
                return;
            }

            // convert the first byte after the command to a oreoled_pattern
            oreoled_pattern pattern = (oreoled_pattern)packet.custom_bytes[CUSTOM_HEADER_LENGTH];

            // uint16_t values are stored in custom_bytes in little endian order
            // assume the flight controller is little endian when decoding values
            uint16_t period =
                    ((0x00FF & (uint16_t)packet.custom_bytes[CUSTOM_HEADER_LENGTH + 7]) << 8) |
                    (0x00FF & (uint16_t)packet.custom_bytes[CUSTOM_HEADER_LENGTH + 8]);
            uint16_t phase_offset =
                    ((0x00FF & (uint16_t)packet.custom_bytes[CUSTOM_HEADER_LENGTH + 9]) << 8) |
                    (0x00FF & (uint16_t)packet.custom_bytes[CUSTOM_HEADER_LENGTH + 10]);

            // call the set_rgb function, using the rest of the bytes as the RGB values
            set_rgb(packet.instance, pattern, packet.custom_bytes[CUSTOM_HEADER_LENGTH + 1], packet.custom_bytes[CUSTOM_HEADER_LENGTH + 2],
                    packet.custom_bytes[CUSTOM_HEADER_LENGTH + 3], packet.custom_bytes[CUSTOM_HEADER_LENGTH + 4], packet.custom_bytes[CUSTOM_HEADER_LENGTH + 5],
                    packet.custom_bytes[CUSTOM_HEADER_LENGTH + 6], period, phase_offset);
        } else if (memcmp(packet.custom_bytes, "SYNC", CUSTOM_HEADER_LENGTH) == 0) { // check for the SYNC sub-command
            // check to make sure the total length matches the length of the SYN0 command + data values
            if (packet.custom_len != CUSTOM_HEADER_LENGTH + 0) {
                return;
            }
            send_sync();
        } else { // unrecognized command
            return;
        }
    } else {
    	// other patterns sent as macro
    	set_macro(packet.instance, (oreoled_macro)packet.pattern);
    }
    _pattern_override = packet.pattern;
}

OreoLED_PX4::oreo_state::oreo_state() {
    clear_state();
}

void OreoLED_PX4::oreo_state::clear_state() {
    mode = OREOLED_MODE_NONE;
    pattern = OREOLED_PATTERN_OFF;
    macro = OREOLED_PARAM_MACRO_RESET;
    red = 0;
    green = 0;
    blue = 0;
    amplitude_red = 0;
    amplitude_green = 0;
    amplitude_blue = 0;
    period = 0;
    repeat = 0;
    phase_offset = 0;
}

void OreoLED_PX4::oreo_state::send_sync() {
    clear_state();
    mode = OREOLED_MODE_SYNC;
}

void OreoLED_PX4::oreo_state::set_macro(oreoled_macro new_macro) {
    clear_state();
    mode = OREOLED_MODE_MACRO;
    macro = new_macro;
}

void OreoLED_PX4::oreo_state::set_rgb(enum oreoled_pattern new_pattern, uint8_t new_red, uint8_t new_green, uint8_t new_blue) {
    clear_state();
    mode = OREOLED_MODE_RGB;
    pattern = new_pattern;
    red = new_red;
    green = new_green;
    blue = new_blue;
}

void OreoLED_PX4::oreo_state::set_rgb(enum oreoled_pattern new_pattern, uint8_t new_red, uint8_t new_green,
        uint8_t new_blue, uint8_t new_amplitude_red, uint8_t new_amplitude_green, uint8_t new_amplitude_blue,
        uint16_t new_period, uint16_t new_phase_offset) {
    clear_state();
    mode = OREOLED_MODE_RGB_EXTENDED;
    pattern = new_pattern;
    red = new_red;
    green = new_green;
    blue = new_blue;
    amplitude_red = new_amplitude_red;
    amplitude_green = new_amplitude_green;
    amplitude_blue = new_amplitude_blue;
    period = new_period;
    phase_offset = new_phase_offset;
}

bool OreoLED_PX4::oreo_state::operator==(const OreoLED_PX4::oreo_state &os) {
   return ((os.mode==mode) && (os.pattern==pattern) && (os.macro==macro) && (os.red==red) && (os.green==green) && (os.blue==blue)
           && (os.amplitude_red==amplitude_red) && (os.amplitude_green==amplitude_green) && (os.amplitude_blue==amplitude_blue)
           && (os.period==period) && (os.repeat==repeat) && (os.phase_offset==phase_offset));
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4

