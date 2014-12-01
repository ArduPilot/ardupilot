/*
   DycoLED Driver Interface
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
/*
DycoLED Driver Usage Guide

Five Parameters to set pattern:
Color Series: color values after completion or at start of each step of pattern
Time Series: time between color transitions.
Brightness series: Brightness of the color at the same index
Resolution: No. of steps in which color transitions will occur.
Steps: Total no. of color transitions

Example:
with color series as {BLUE,RED,GREEN}
Time Series as {1000,1000,1000} --> first time is transition time in ms for BLUE -> RED
                                --> second time is for RED -> GREEN
                                --> Third time is for GREEN -> BLUE
Brightness series as {0.5,0.5,1.0}
Resolution as 10
Steps as 3

The loop of pattern will be:
first the LED will be set to HALF BRIGHT BLUE color. After
every 1000/10 milliseconds BLUE color will reduce at such
a rate that it diminishes at 1000 ms after the start of 
pattern and RED component will increase such that it increases
to HALF_BRIGHT at same time as BLUE goes to zero and similar
process will continue with RED diminishing and GREEN increases
to FULL brightness (1.0) thus completing 3 basic transitions
of loop i.e. from BLUE to RED to GREEN. The loop will until
another pattern or solid color is set.
*/
#include <AP_HAL.h>
#include <AP_GPS.h>
#include "DycoLED_Linux.h"
#include "AP_Notify.h"

extern const AP_HAL::HAL& hal;

static led_pattern preset_pattern[14]={{{BLUE,RED},{100,100},{1.0,1.0},10,2},                               //INITIALISING
                                       {{RED,BLUE,GREEN},{250,250,250},{1.0,1.0,1.0},15,3},                 //SAV_TRIM_ESC_CAL
                                       {{BLACK,YELLOW},{250,250},{1.0,1.0},15,2},                           //FS_RAD_BATT
                                       {{BLUE,YELLOW},{250,250},{1.0,1.0},15,2},                            //FS_GPS
                                       {{PURPLE,YELLOW},{250,250},{1.0,1.0},15,2},                          //BARO_GLITCH
                                       {{RED,YELLOW},{250,250},{1.0,1.0},15,2},                             //EKF_BAD
                                       {{GREEN,GREEN},{1000,1000},{1.0,1.0},1,2},                           //ARMED_GPS
                                       {{BLUE,BLUE},{1000,1000},{1.0,1.0},1,2},                             //ARMED_NOGPS
                                       {{BLACK,YELLOW},{500,500},{1.0,1.0},15,2},                           //PREARM_CHECK
                                       {{BLACK,GREEN},{500,500},{1.0,1.0},15,2},                            //DISARMED_GPS
                                       {{BLACK,BLUE},{500,500},{1.0,1.0},31,2},                             //DISARMED_NOGPS
                                       {{GREEN,BLACK,GREEN,BLACK},{100,50,100,1000},{1.0,1.0,1.0,1.0},1,4}, //SAFE_STROBE
                                       {{RED,BLACK,RED,BLACK},{100,50,100,1000},{1.0,1.0,1.0,1.0},1,4},     //FAIL_STROBE
                                       {{BLUE,BLACK,BLUE,BLACK},{100,50,100,1000},{1.0,1.0,1.0,1.0},1,4}};  //NEUTRAL_STROBE
 

bool DycoLED::init()
{
    hal.util->led_init(MAX_NUM_LEDS);
	return true;
}

void DycoLED::set_preset_pattern(uint16_t led,uint8_t patt)
{
      hal.util->led_set_pattern(led,preset_pattern[patt].color,preset_pattern[patt].brightness,preset_pattern[patt].time,
                                 preset_pattern[patt].res,preset_pattern[patt].len);
}
 
// update - updates led according to timed_updated.  Should be called
// at 50Hz
void DycoLED::update()
{
    if (AP_Notify::flags.initialising) {
        set_preset_pattern(STATUS_LED,NOTIFY_INITIALISING);
        set_preset_pattern(STROBE_LED,NOTIFY_NEUTRAL_STROBE);
        return;                  // exit so no other status modify this pattern
    }
    
    if (AP_Notify::flags.save_trim || AP_Notify::flags.esc_calibration){
        set_preset_pattern(STATUS_LED,NOTIFY_SAV_TRIM_ESC_CAL);
        set_preset_pattern(STROBE_LED,NOTIFY_NEUTRAL_STROBE);
        return;
    }
    if(AP_Notify::flags.failsafe_radio || AP_Notify::flags.failsafe_battery){
        set_preset_pattern(STATUS_LED,NOTIFY_FS_RAD_BATT);
        set_preset_pattern(STROBE_LED,NOTIFY_FAIL_STROBE);
        return;
    }
    if(AP_Notify::flags.failsafe_gps || AP_Notify::flags.gps_glitching){
        set_preset_pattern(STATUS_LED,NOTIFY_FS_GPS);
        set_preset_pattern(STROBE_LED,NOTIFY_FAIL_STROBE);
        return;
    }
    if(AP_Notify::flags.baro_glitching){
        set_preset_pattern(STATUS_LED,NOTIFY_BARO_GLITCH);
        set_preset_pattern(STROBE_LED,NOTIFY_FAIL_STROBE);
        return;
    }
    if(AP_Notify::flags.ekf_bad){
        set_preset_pattern(STATUS_LED,NOTIFY_EKF_BAD);
        set_preset_pattern(STROBE_LED,NOTIFY_FAIL_STROBE);
        return;
    }
    
    if (AP_Notify::flags.armed) {
        if (AP_Notify::flags.gps_status >= AP_GPS::GPS_OK_FIX_3D_DGPS) {
            hal.util->led_set_solid_color(STATUS_LED,GREEN);
            set_preset_pattern(STROBE_LED,NOTIFY_SAFE_STROBE);
        } else{
            hal.util->led_set_solid_color(STATUS_LED,BLUE);
            set_preset_pattern(STROBE_LED,NOTIFY_NEUTRAL_STROBE);
        }
    } else{
        if (!AP_Notify::flags.pre_arm_check) {
            set_preset_pattern(STATUS_LED,NOTIFY_PREARM_CHECK);
            set_preset_pattern(STROBE_LED,NOTIFY_NEUTRAL_STROBE);
        } else{
            if (AP_Notify::flags.gps_status >= AP_GPS::GPS_OK_FIX_3D_DGPS) {
                set_preset_pattern(STATUS_LED,NOTIFY_DISARMED_GPS);
                set_preset_pattern(STROBE_LED,NOTIFY_SAFE_STROBE);
            } else{
                set_preset_pattern(STATUS_LED,NOTIFY_DISARMED_NOGPS);
                set_preset_pattern(STROBE_LED,NOTIFY_SAFE_STROBE);
            }
        }
    }
}
