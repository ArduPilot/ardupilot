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
#include <DataFlash/DataFlash.h>
#define OREOLED_BACKLEFT                0       // back left led instance number
#define OREOLED_BACKRIGHT               1       // back right led instance number
#define OREOLED_FRONTRIGHT              2       // front right led instance number
#define OREOLED_FRONTLEFT               3       // front left led instance number
#define PERIOD_SLOW                     8195    // Slow flash rate
#define PERIOD_FAST                     62465   // Fast flash rate
#define PERIOD_SUPER                    38400   // Super fast rate
#define PO_ALTERNATE                    46080   // 180 degree Phase Offset

extern const AP_HAL::HAL& hal;

// constructor
OreoLED_PX4::OreoLED_PX4() : NotifyDevice(),
    _overall_health(false),
    _oreoled_fd(-1),
    _send_required(false),
    _state_desired_semaphore(false),
    _pattern_override(0)
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


//
// UPDATE device according to timed_updated. Called at 50Hz
//
void OreoLED_PX4::update()
{

    if (!_overall_health) { return;}        // Don't go any further if LED driver reports unhealthy

    if (Slow_Counter()) {return; }          // Slowing rate from 50hz to 10hz.
    
    Sync_Counter();                         // Syncronizes LEDs every 10 seconds. No need to do this at 10hz
    
    if (Mode_FW()) { return; }              // Don't go any further if the Pixhawk is in firmware update.

    if (Mode_Init()) { return; }            // Don't go any further if the Pixhawk is initializing
    
    if (Mode_FS_Radio()) { return; }        // Don't go any further if the Pixhawk is is in radio failsafe
    
    // At this point, we've cleared all the situations that would require special handling.
    // Everything beyond this point uses the visual feedback LED colors and patterns.
    // Front LEDs use aviation standard red/green
    // Rear LED colors use white for normal, yellow for GPS unusable, and purple for EKF bad.
    // Rear LEDs remain solid until pre-arm checks pass, then they strobe.
    
    Set_Std_Colors();                       // Set the rear LED standard colors as described above 
    
    if (Mode_FS_Batt()) { return; }         // Stop here if the battery is low.
    
    if (_pattern_override) { return; }      // Stop here if in mavlink LED control override.
    
    if (Mode_Auto_Flight()) {return; }      // Stop here if in an autopilot mode.
    
    if (Mode_Pilot_Flight()) {return; }     // Stop here if in an pilot controlled mode.
    
    return;                                 // The end
    
}

// Slow the update rate from 50hz to 10hz
// Returns true if counting up
// Returns false and resets one counter hits 5
bool OreoLED_PX4::Slow_Counter()
{
    static uint8_t UpdateCounter{0};
    
    UpdateCounter++;
    if (UpdateCounter < 5) { 
        return 1;
    } else {
        UpdateCounter = 0;
        return 0;
    }
}


// Calls resyncing the LEDs every 10 seconds
// Returns true if counting up
// Always returns false, no action needed.
bool OreoLED_PX4::Sync_Counter()
{
    static uint8_t SyncCounter{800};
    
    SyncCounter++;
    if (SyncCounter > 100) { 
        SyncCounter = 0;
        send_sync();
    }
    return 0;
}


// Procedure for when Pixhawk is in FW update / bootloader
// Makes all LEDs go into color cycle mode
// Returns true if firmware update in progress. False if not
bool OreoLED_PX4::Mode_FW()
{
    if (AP_Notify::flags.firmware_update) {
        set_macro(OREOLED_INSTANCE_ALL, OREOLED_PARAM_MACRO_COLOUR_CYCLE);
        return true;
    } else {
        return false;
    }
}


// Procedure for when Pixhawk is booting up and initializing
// Makes all LEDs rapidly strobe blue. Returns LED_INIT_STAGE
// 1 = Default, initialization has not yet begun
// 2 = Initialization flag found, initialization in progress
// 3 = Initialization flag no longer found, initialization complete
bool OreoLED_PX4::Mode_Init()
{
    static uint16_t stage{1};
    
    // Pixhawk has not begun initializing yet. Strobe all blue
    if ((!AP_Notify::flags.initialising) && ((stage == 1))) {
        set_rgb(OREOLED_INSTANCE_ALL, OREOLED_PATTERN_STROBE, 0, 0, 255,0,0,0,PERIOD_SUPER,0);

    // Pixhawk has begun initializing
    } else if ((AP_Notify::flags.initialising) && ((stage == 1))) { 
        stage = 2;
        set_rgb(OREOLED_INSTANCE_ALL, OREOLED_PATTERN_STROBE, 0, 0, 255,0,0,0,PERIOD_SUPER,0);
        
    // Pixhawk still initializing
    } else if ((AP_Notify::flags.initialising) && ((stage == 2))) { 
        set_rgb(OREOLED_INSTANCE_ALL, OREOLED_PATTERN_STROBE, 0, 0, 255,0,0,0,PERIOD_SUPER,0);
    
    // Pixhawk has completed initialization
    } else if((!AP_Notify::flags.initialising) && ((stage == 2))) {
        stage = 0;
        set_rgb(OREOLED_INSTANCE_ALL, OREOLED_PATTERN_SOLID, 0, 0, 255);

    } else { stage = 0; }
    
    return stage;
}


// Procedure for when Pixhawk is in radio failsafe
// LEDs perfoirm alternating Red X pattern
bool OreoLED_PX4::Mode_FS_Radio()
{
    if (AP_Notify::flags.failsafe_radio) {
        set_rgb(OREOLED_FRONTLEFT, OREOLED_PATTERN_STROBE, 255, 0, 0,0,0,0,PERIOD_SLOW,0);
        set_rgb(OREOLED_FRONTRIGHT, OREOLED_PATTERN_STROBE, 255, 0, 0,0,0,0,PERIOD_SLOW,PO_ALTERNATE);
        set_rgb(OREOLED_BACKLEFT, OREOLED_PATTERN_STROBE, 255, 0, 0,0,0,0,PERIOD_SLOW,PO_ALTERNATE);
        set_rgb(OREOLED_BACKRIGHT, OREOLED_PATTERN_STROBE, 255, 0, 0,0,0,0,PERIOD_SLOW,0);
    }
    return AP_Notify::flags.failsafe_radio;
}


// Procedure to set standard LED colors
// Front left pre defined as red
// Front right pre defined as green
// Back LEDS White for normal, yellow for GPS not usable, purple for EKF bad]
// Returns true GPS or EKF problem, returns false if all ok
bool OreoLED_PX4::Set_Std_Colors()
{
    if (!(AP_Notify::flags.gps_fusion)) {
        rear_color_r = 255;
        rear_color_g = 50;
        rear_color_b = 0;
        return 1;
        
    } else if (AP_Notify::flags.ekf_bad){
        rear_color_r = 255;
        rear_color_g = 0;
        rear_color_b = 255;
        return 1;
        
    } else {
        rear_color_r = 255;
        rear_color_g = 255;
        rear_color_b = 255;
        return 0;
    }
}


// Procedure to set low battery LED output
// Colors standard
// Fast strobe alternating front/back
bool OreoLED_PX4::Mode_FS_Batt()
{
    if (AP_Notify::flags.failsafe_battery){
        set_rgb(OREOLED_FRONTLEFT, OREOLED_PATTERN_STROBE, 255, 0, 0,0,0,0,PERIOD_FAST,0);
        set_rgb(OREOLED_FRONTRIGHT, OREOLED_PATTERN_STROBE, 0, 255, 0,0,0,0,PERIOD_FAST,0);
        set_rgb(OREOLED_BACKLEFT, OREOLED_PATTERN_STROBE, rear_color_r, rear_color_g, rear_color_b,0,0,0,PERIOD_FAST,PO_ALTERNATE);
        set_rgb(OREOLED_BACKRIGHT, OREOLED_PATTERN_STROBE, rear_color_r, rear_color_g, rear_color_b,0,0,0,PERIOD_FAST,PO_ALTERNATE);
    }
    return AP_Notify::flags.failsafe_battery;
}


// Procedure for when Pixhawk is in an autopilot mode
// Makes all LEDs strobe super fast using standard colors
bool OreoLED_PX4::Mode_Auto_Flight()
{
    if (AP_Notify::flags.autopilot_mode) {
        if ((AP_Notify::flags.pre_arm_check && AP_Notify::flags.pre_arm_gps_check) || AP_Notify::flags.armed) {
            set_rgb(OREOLED_FRONTLEFT, OREOLED_PATTERN_STROBE, 255, 0, 0,0,0,0,PERIOD_SUPER,0);
            set_rgb(OREOLED_FRONTRIGHT, OREOLED_PATTERN_STROBE, 0, 255, 0,0,0,0,PERIOD_SUPER,0);
            set_rgb(OREOLED_BACKLEFT, OREOLED_PATTERN_STROBE, rear_color_r, rear_color_g, rear_color_b,0,0,0,PERIOD_SUPER,PO_ALTERNATE);
            set_rgb(OREOLED_BACKRIGHT, OREOLED_PATTERN_STROBE, rear_color_r, rear_color_g, rear_color_b,0,0,0,PERIOD_SUPER,PO_ALTERNATE);
        } else {
            set_rgb(OREOLED_FRONTLEFT, OREOLED_PATTERN_STROBE, 255, 0, 0,0,0,0,PERIOD_SUPER,0);
            set_rgb(OREOLED_FRONTRIGHT, OREOLED_PATTERN_STROBE, 0, 255, 0,0,0,0,PERIOD_SUPER,0);
            set_rgb(OREOLED_BACKLEFT, OREOLED_PATTERN_SOLID, rear_color_r, rear_color_g, rear_color_b);
            set_rgb(OREOLED_BACKRIGHT, OREOLED_PATTERN_SOLID, rear_color_r, rear_color_g, rear_color_b);
        }
    }
    return AP_Notify::flags.autopilot_mode;
}


// Procedure for when Pixhawk is in a pilot controlled mode
// All LEDs use standard pattern and colors
bool OreoLED_PX4::Mode_Pilot_Flight()
{
    if ((AP_Notify::flags.pre_arm_check && AP_Notify::flags.pre_arm_gps_check) || AP_Notify::flags.armed) {
        set_rgb(OREOLED_FRONTLEFT, OREOLED_PATTERN_SOLID, 255, 0, 0,0,0,0,PERIOD_SUPER,0);
        set_rgb(OREOLED_FRONTRIGHT, OREOLED_PATTERN_SOLID, 0, 255, 0,0,0,0,PERIOD_SUPER,0);
        set_rgb(OREOLED_BACKLEFT, OREOLED_PATTERN_STROBE, rear_color_r, rear_color_g, rear_color_b,0,0,0,PERIOD_FAST,0);
        set_rgb(OREOLED_BACKRIGHT, OREOLED_PATTERN_STROBE, rear_color_r, rear_color_g, rear_color_b,0,0,0,PERIOD_FAST,PO_ALTERNATE);
    } else {
        set_rgb(OREOLED_FRONTLEFT, OREOLED_PATTERN_SOLID, 255, 0, 0);
        set_rgb(OREOLED_FRONTRIGHT, OREOLED_PATTERN_SOLID, 0, 255, 0);
        set_rgb(OREOLED_BACKLEFT, OREOLED_PATTERN_SOLID, rear_color_r, rear_color_g, rear_color_b);
        set_rgb(OREOLED_BACKRIGHT, OREOLED_PATTERN_SOLID, rear_color_r, rear_color_g, rear_color_b);
    }
    return 1;
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
                    cmd.buff[13] = OREOLED_PARAM_PERIOD;
                    cmd.buff[14] = (_state_desired[i].period & 0x00FF);
                    cmd.buff[15] = (_state_desired[i].period & 0xFF00) >> 8;
                    cmd.buff[16] = OREOLED_PARAM_PHASEOFFSET;
                    cmd.buff[17] = (_state_desired[i].phase_offset & 0x00FF);
                    cmd.buff[18] = (_state_desired[i].phase_offset & 0xFF00) >> 8;
                    cmd.num_bytes = 19;
                    ioctl(_oreoled_fd, OREOLED_SEND_BYTES, (unsigned long)&cmd);
                    }
                    DataFlash_Class::instance()->Log_Write("OrEX", "TimeUS,id,patt,r,g,b,PerV,per1,per2,poV,po1,po2", "Qfffffffffff",
                       AP_HAL::micros64(),
                       (double)i,
                       (double)_state_desired[i].pattern,
                       (double)_state_desired[i].red,
                       (double)_state_desired[i].green,
                       (double)_state_desired[i].blue,
                       (double)_state_desired[i].period,
                       (double)(_state_desired[i].period & 0x00FF),
                       (double)((_state_desired[i].period & 0xFF00) >> 8),
                       (double)_state_desired[i].phase_offset,
                       (double)(_state_desired[i].phase_offset & 0x00FF),
                       (double)((_state_desired[i].phase_offset & 0xFF00) >> 8));
                    
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

            uint16_t period =
                    (0x00FF & (uint16_t)packet.custom_bytes[CUSTOM_HEADER_LENGTH + 7]) |
                    ((0x00FF & (uint16_t)packet.custom_bytes[CUSTOM_HEADER_LENGTH + 8]) << 8);
            uint16_t phase_offset =
                    (0x00FF & (uint16_t)packet.custom_bytes[CUSTOM_HEADER_LENGTH + 9]) |
                    ((0x00FF & (uint16_t)packet.custom_bytes[CUSTOM_HEADER_LENGTH + 10]) << 8);

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
#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4