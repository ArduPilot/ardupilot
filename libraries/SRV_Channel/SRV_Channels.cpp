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

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
  #include <AP_CANManager/AP_CANManager.h>
  #include <AP_UAVCAN/AP_UAVCAN.h>

  // To be replaced with macro saying if KDECAN library is included
  #if APM_BUILD_TYPE(APM_BUILD_ArduCopter) || APM_BUILD_TYPE(APM_BUILD_ArduPlane) || APM_BUILD_TYPE(APM_BUILD_ArduSub)
    #include <AP_KDECAN/AP_KDECAN.h>
  #endif
  #include <AP_ToshibaCAN/AP_ToshibaCAN.h>
  #include <AP_PiccoloCAN/AP_PiccoloCAN.h>
#endif

#if NUM_SERVO_CHANNELS == 0
#pragma GCC diagnostic ignored "-Wtype-limits"
#endif

extern const AP_HAL::HAL& hal;

SRV_Channel *SRV_Channels::channels;
SRV_Channels *SRV_Channels::_singleton;

#ifndef HAL_BUILD_AP_PERIPH
AP_Volz_Protocol *SRV_Channels::volz_ptr;
AP_SBusOut *SRV_Channels::sbus_ptr;
AP_RobotisServo *SRV_Channels::robotis_ptr;
#endif // HAL_BUILD_AP_PERIPH

uint16_t SRV_Channels::override_counter[NUM_SERVO_CHANNELS];

#if HAL_SUPPORT_RCOUT_SERIAL
AP_BLHeli *SRV_Channels::blheli_ptr;
#endif

uint16_t SRV_Channels::disabled_mask;
uint16_t SRV_Channels::digital_mask;
uint16_t SRV_Channels::reversible_mask;

bool SRV_Channels::disabled_passthrough;
bool SRV_Channels::initialised;
bool SRV_Channels::emergency_stop;
Bitmask<SRV_Channel::k_nr_aux_servo_functions> SRV_Channels::function_mask;
SRV_Channels::srv_function SRV_Channels::functions[SRV_Channel::k_nr_aux_servo_functions];

const AP_Param::GroupInfo SRV_Channels::var_info[] = {
#if (NUM_SERVO_CHANNELS >= 1)
    // @Group: 1_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[0], "1_",  1, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 2)
    // @Group: 2_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[1], "2_",  2, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 3)
    // @Group: 3_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[2], "3_",  3, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 4)
    // @Group: 4_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[3], "4_",  4, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 5)
    // @Group: 5_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[4], "5_",  5, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 6)
    // @Group: 6_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[5], "6_",  6, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 7)
    // @Group: 7_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[6], "7_",  7, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 8)
    // @Group: 8_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[7], "8_",  8, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 9)
    // @Group: 9_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[8], "9_",  9, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 10)
    // @Group: 10_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[9], "10_",  10, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 11)
    // @Group: 11_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[10], "11_",  11, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 12)
    // @Group: 12_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[11], "12_",  12, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 13)
    // @Group: 13_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[12], "13_",  13, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 14)
    // @Group: 14_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[13], "14_",  14, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 15)
    // @Group: 15_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[14], "15_",  15, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 16)
    // @Group: 16_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[15], "16_",  16, SRV_Channels, SRV_Channel),
#endif

#ifndef  HAL_BUILD_AP_PERIPH
    // @Param{Plane}: _AUTO_TRIM
    // @DisplayName: Automatic servo trim
    // @Description: This enables automatic servo trim in flight. Servos will be trimed in stabilized flight modes when the aircraft is close to level. Changes to servo trim will be saved every 10 seconds and will persist between flights. The automatic trim won't go more than 20% away from a centered trim.
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO_FRAME("_AUTO_TRIM",  17, SRV_Channels, auto_trim, 0, AP_PARAM_FRAME_PLANE),
#endif

    // @Param: _RATE
    // @DisplayName: Servo default output rate
    // @Description: This sets the default output rate in Hz for all outputs.
    // @Range: 25 400
    // @User: Advanced
    // @Units: Hz
    AP_GROUPINFO("_RATE",  18, SRV_Channels, default_rate, 50),

#ifndef HAL_BUILD_AP_PERIPH
    // @Group: _VOLZ_
    // @Path: ../AP_Volz_Protocol/AP_Volz_Protocol.cpp
    AP_SUBGROUPINFO(volz, "_VOLZ_",  19, SRV_Channels, AP_Volz_Protocol),

    // @Group: _SBUS_
    // @Path: ../AP_SBusOut/AP_SBusOut.cpp
    AP_SUBGROUPINFO(sbus, "_SBUS_",  20, SRV_Channels, AP_SBusOut),

#if HAL_SUPPORT_RCOUT_SERIAL
    // @Group: _BLH_
    // @Path: ../AP_BLHeli/AP_BLHeli.cpp
    AP_SUBGROUPINFO(blheli, "_BLH_",  21, SRV_Channels, AP_BLHeli),
#endif

    // @Group: _ROB_
    // @Path: ../AP_RobotisServo/AP_RobotisServo.cpp
    AP_SUBGROUPINFO(robotis, "_ROB_",  22, SRV_Channels, AP_RobotisServo),
#endif // HAL_BUILD_AP_PERIPH

    // @Param: _DSHOT_RATE
    // @DisplayName: Servo DShot output rate
    // @Description: This sets the DShot output rate for all outputs as a multiple of the loop rate. 0 sets the output rate to be fixed at 1Khz for low loop rates. This value should never be set below 500Hz.
    // @Values: 0:1Khz,1:loop-rate,2:double loop-rate,3:triple loop-rate,4:quadruple loop rate
    // @User: Advanced
    AP_GROUPINFO("_DSHOT_RATE",  23, SRV_Channels, dshot_rate, 0),

    // @Param: _DSHOT_ESC
    // @DisplayName: Servo DShot ESC type
    // @Description: This sets the DShot ESC type for all outputs. The ESC type affects the range of DShot commands available. None means that no dshot commands will be executed.
    // @Values: 0:None,1:BLHeli32/BLHeli_S/Kiss
    // @User: Advanced
    AP_GROUPINFO("_DSHOT_ESC",  24, SRV_Channels, dshot_esc_type, 0),

    AP_GROUPEND
};

/*
  constructor
 */
SRV_Channels::SRV_Channels(void)
{
    _singleton = this;
    channels = obj_channels;

    // set defaults from the parameter table
    AP_Param::setup_object_defaults(this, var_info);

    // setup ch_num on channels
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        channels[i].ch_num = i;
    }

#ifndef HAL_BUILD_AP_PERIPH
    volz_ptr = &volz;
    sbus_ptr = &sbus;
    robotis_ptr = &robotis;
#if HAL_SUPPORT_RCOUT_SERIAL
    blheli_ptr = &blheli;
#endif
#endif // HAL_BUILD_AP_PERIPH
}

// SRV_Channels initialization
void SRV_Channels::init(void)
{
    // initialize BLHeli late so that all of the masks it might setup don't get trodden on by motor initialization
#if HAL_SUPPORT_RCOUT_SERIAL
    blheli_ptr->init();
#endif
    hal.rcout->set_dshot_rate(_singleton->dshot_rate, AP::scheduler().get_loop_rate_hz());
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

void SRV_Channels::setup_failsafe_trim_all_non_motors(void)
{
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        if (!SRV_Channel::is_motor(channels[i].get_function())) {
            hal.rcout->set_failsafe_pwm(1U<<channels[i].ch_num, channels[i].servo_trim);
        }
    }
}

/*
  run calc_pwm for all channels
 */
void SRV_Channels::calc_pwm(void)
{
    WITH_SEMAPHORE(_singleton->override_counter_sem);

    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        // check if channel has been locked out for this loop
        // if it has, decrement the loop count for that channel
        if (override_counter[i] == 0) {
            channels[i].set_override(false);
        } else {
            channels[i].set_override(true);
            override_counter[i]--;
        }
        channels[i].calc_pwm(functions[channels[i].function].output_scaled);
    }
}

// set output value for a specific function channel as a pwm value
void SRV_Channels::set_output_pwm_chan(uint8_t chan, uint16_t value)
{
    if (chan < NUM_SERVO_CHANNELS) {
        channels[chan].set_output_pwm(value);
    }
}

// set output value for a specific function channel as a pwm value with loop based timeout
// timeout_ms of zero will clear override of the channel
// minimum override is 1 MAIN_LOOP
void SRV_Channels::set_output_pwm_chan_timeout(uint8_t chan, uint16_t value, uint16_t timeout_ms)
{
    WITH_SEMAPHORE(_singleton->override_counter_sem);

    if (chan < NUM_SERVO_CHANNELS) {
        const uint32_t loop_period_us = AP::scheduler().get_loop_period_us();
        // round up so any non-zero requested value will result in at least one loop
        const uint32_t loop_count = ((timeout_ms * 1000U) + (loop_period_us - 1U)) / loop_period_us;
        override_counter[chan] = constrain_int32(loop_count, 0, UINT16_MAX);
        channels[chan].set_override(true);
        channels[chan].set_output_pwm(value,true);
    }
}

/*
  wrapper around hal.rcout->cork()
 */
void SRV_Channels::cork()
{
    hal.rcout->cork();
}

/*
  wrapper around hal.rcout->push()
 */
void SRV_Channels::push()
{
    hal.rcout->push();

#ifndef HAL_BUILD_AP_PERIPH
    // give volz library a chance to update
    volz_ptr->update();

    // give sbus library a chance to update
    sbus_ptr->update();

    // give robotis library a chance to update
    robotis_ptr->update();
    
#if HAL_SUPPORT_RCOUT_SERIAL
    // give blheli telemetry a chance to update
    blheli_ptr->update_telemetry();
#endif
#endif // HAL_BUILD_AP_PERIPH

#if HAL_CANMANAGER_ENABLED
    // push outputs to CAN
    uint8_t can_num_drivers = AP::can().get_num_drivers();
    for (uint8_t i = 0; i < can_num_drivers; i++) {
        switch (AP::can().get_driver_type(i)) {
            case AP_CANManager::Driver_Type_UAVCAN: {
                AP_UAVCAN *ap_uavcan = AP_UAVCAN::get_uavcan(i);
                if (ap_uavcan == nullptr) {
                    continue;
                }
                ap_uavcan->SRV_push_servos();
                break;
            }
            case AP_CANManager::Driver_Type_KDECAN: {
// To be replaced with macro saying if KDECAN library is included
#if APM_BUILD_TYPE(APM_BUILD_ArduCopter) || APM_BUILD_TYPE(APM_BUILD_ArduPlane) || APM_BUILD_TYPE(APM_BUILD_ArduSub)
                AP_KDECAN *ap_kdecan = AP_KDECAN::get_kdecan(i);
                if (ap_kdecan == nullptr) {
                    continue;
                }
                ap_kdecan->update();
#endif
                break;
            }
            case AP_CANManager::Driver_Type_ToshibaCAN: {
                AP_ToshibaCAN *ap_tcan = AP_ToshibaCAN::get_tcan(i);
                if (ap_tcan == nullptr) {
                    continue;
                }
                ap_tcan->update();
                break;
            }
#if HAL_PICCOLO_CAN_ENABLE
            case AP_CANManager::Driver_Type_PiccoloCAN: {
                AP_PiccoloCAN *ap_pcan = AP_PiccoloCAN::get_pcan(i);
                if (ap_pcan == nullptr) {
                    continue;
                }
                ap_pcan->update();
                break;
            }
#endif
            case AP_CANManager::Driver_Type_CANTester:
            case AP_CANManager::Driver_Type_None:
            default:
                break;
        }
    }
#endif // HAL_NUM_CAN_IFACES
}

void SRV_Channels::zero_rc_outputs()
{
    /* Send an invalid signal to the motors to prevent spinning due to
     * neutral (1500) pwm pulse being cut short.  For that matter,
     * send an invalid signal to all channels to prevent
     * undesired/unexpected behavior
     */
    cork();
    for (uint8_t i=0; i<NUM_RC_CHANNELS; i++) {
        hal.rcout->write(i, 0);
    }
    push();
}
