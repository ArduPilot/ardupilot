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
  SRV_Channel.cpp - object to separate input and output channel
  ranges, trim and reversal
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include "SRV_Channel.h"

extern const AP_HAL::HAL& hal;

SRV_Channel *SRV_Channels::channels;
bool SRV_Channels::disabled_passthrough;
bool SRV_Channels::initialised;
Bitmask SRV_Channels::function_mask{SRV_Channel::k_nr_aux_servo_functions};
SRV_Channels::srv_function SRV_Channels::functions[SRV_Channel::k_nr_aux_servo_functions];

AP_Int32* SRV_Channels::p_can_servo_bm = nullptr;
AP_Int32* SRV_Channels::p_can_esc_bm = nullptr;

const AP_Param::GroupInfo SRV_Channels::var_info[] = {
    // @Group: 1_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[0], "1_",  1, SRV_Channels, SRV_Channel),

    // @Group: 2_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[1], "2_",  2, SRV_Channels, SRV_Channel),

    // @Group: 3_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[2], "3_",  3, SRV_Channels, SRV_Channel),

    // @Group: 4_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[3], "4_",  4, SRV_Channels, SRV_Channel),

    // @Group: 5_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[4], "5_",  5, SRV_Channels, SRV_Channel),

    // @Group: 6_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[5], "6_",  6, SRV_Channels, SRV_Channel),

    // @Group: 7_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[6], "7_",  7, SRV_Channels, SRV_Channel),

    // @Group: 8_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[7], "8_",  8, SRV_Channels, SRV_Channel),

    // @Group: 9_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[8], "9_",  9, SRV_Channels, SRV_Channel),

    // @Group: 10_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[9], "10_",  10, SRV_Channels, SRV_Channel),

    // @Group: 11_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[10], "11_",  11, SRV_Channels, SRV_Channel),

    // @Group: 12_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[11], "12_",  12, SRV_Channels, SRV_Channel),

    // @Group: 13_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[12], "13_",  13, SRV_Channels, SRV_Channel),

    // @Group: 14_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[13], "14_",  14, SRV_Channels, SRV_Channel),

    // @Group: 15_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[14], "15_",  15, SRV_Channels, SRV_Channel),

    // @Group: 16_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[15], "16_",  16, SRV_Channels, SRV_Channel),
    
    // @Param: _AUTO_TRIM
    // @DisplayName: Automatic servo trim
    // @Description: This enables automatic servo trim in flight. Servos will be trimed in stabilized flight modes when the aircraft is close to level. Changes to servo trim will be saved every 10 seconds and will persist between flights.
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO_FRAME("_AUTO_TRIM",  17, SRV_Channels, auto_trim, 0, AP_PARAM_FRAME_PLANE),
    
    // @Param: _CANSRV_BM
    // @DisplayName: RC Out channels to be transmitted as servo over CAN bus
    // @Description: Bitmask with one set for channel to be transmitted as a servo command over CAN bus
    // @Bitmask: 0: Servo 1, 1: Servo 2, 2: Servo 3, 3: Servo 4, 4: Servo 5, 5: Servo 6, 6: Servo 7, 7: Servo 8, 8: Servo 9, 9: Servo 10, 10: Servo 11, 11: Servo 12, 12: Servo 13, 13: Servo 14, 14: Servo 15
    // @User: Standard
    AP_GROUPINFO("_CANSRV_BM", 18, SRV_Channels, can_servo_bm, 0),

    // @Param: _CANESC_BM
    // @DisplayName: RC Out channels to be transmitted as ESC over CAN bus
    // @Description: Bitmask with one set for channel to be transmitted as a ESC command over CAN bus
    // @Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16
    // @User: Standard
    AP_GROUPINFO("_CANESC_BM", 19, SRV_Channels, can_esc_bm, 0),

    AP_GROUPEND
};

/*
  constructor
 */
SRV_Channels::SRV_Channels(void)
{
    channels = obj_channels;
    p_can_servo_bm = &can_servo_bm;
    p_can_esc_bm = &can_esc_bm;
    
    // set defaults from the parameter table
    AP_Param::setup_object_defaults(this, var_info);

    // setup ch_num on channels
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        channels[i].ch_num = i;
    }
}

/*
  save adjusted trims
 */
void SRV_Channels::save_trim(void)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (trimmed_mask & (1U<<i)) {
            channels[i].servo_trim.set_and_save(channels[i].servo_trim.get());
        }
    }
    trimmed_mask = 0;
}

void SRV_Channels::output_trim_all(void)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        channels[i].set_output_pwm(channels[i].servo_trim);
    }
}

void SRV_Channels::setup_failsafe_trim_all(void)
{
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        hal.rcout->set_failsafe_pwm(1U<<channels[i].ch_num, channels[i].servo_trim);
    }
}

/*
  run calc_pwm for all channels
 */
void SRV_Channels::calc_pwm(void)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        channels[i].calc_pwm(functions[channels[i].function].output_scaled);
    }
}
