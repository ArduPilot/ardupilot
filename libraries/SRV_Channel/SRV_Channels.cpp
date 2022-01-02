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
  #if APM_BUILD_COPTER_OR_HELI || APM_BUILD_TYPE(APM_BUILD_ArduPlane) || APM_BUILD_TYPE(APM_BUILD_ArduSub)
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

#if AP_FETTEC_ONEWIRE_ENABLED
AP_FETtecOneWire *SRV_Channels::fetteconwire_ptr;
#endif

uint16_t SRV_Channels::override_counter[NUM_SERVO_CHANNELS];

#if HAL_SUPPORT_RCOUT_SERIAL
AP_BLHeli *SRV_Channels::blheli_ptr;
#endif

uint32_t SRV_Channels::disabled_mask;
uint32_t SRV_Channels::digital_mask;
uint32_t SRV_Channels::reversible_mask;
uint32_t SRV_Channels::invalid_mask;

bool SRV_Channels::disabled_passthrough;
bool SRV_Channels::initialised;
bool SRV_Channels::emergency_stop;
Bitmask<SRV_Channel::k_nr_aux_servo_functions> SRV_Channels::function_mask;
SRV_Channels::srv_function SRV_Channels::functions[SRV_Channel::k_nr_aux_servo_functions];
SRV_Channels::slew_list *SRV_Channels::_slew;

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
#endif // HAL_BUILD_AP_PERIPH

#if HAL_SUPPORT_RCOUT_SERIAL
    // @Group: _BLH_
    // @Path: ../AP_BLHeli/AP_BLHeli.cpp
    AP_SUBGROUPINFO(blheli, "_BLH_",  21, SRV_Channels, AP_BLHeli),
#endif

#ifndef HAL_BUILD_AP_PERIPH
    // @Group: _ROB_
    // @Path: ../AP_RobotisServo/AP_RobotisServo.cpp
    AP_SUBGROUPINFO(robotis, "_ROB_",  22, SRV_Channels, AP_RobotisServo),

#if AP_FETTEC_ONEWIRE_ENABLED
    // @Group: _FTW_
    // @Path: ../AP_FETtecOneWire/AP_FETtecOneWire.cpp
    AP_SUBGROUPINFO(fetteconwire, "_FTW_",  25, SRV_Channels, AP_FETtecOneWire),
#endif

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
    // @Values: 0:None,1:BLHeli32/Kiss,2:BLHeli_S
    // @User: Advanced
    AP_GROUPINFO("_DSHOT_ESC",  24, SRV_Channels, dshot_esc_type, 0),

    // @Param: _GPIO_MASK
    // @DisplayName: Servo GPIO mask
    // @Description: This sets a bitmask of outputs which will be available as GPIOs. Any auxillary output with either the function set to -1 or with the corresponding bit set in this mask will be available for use as a GPIO pin
    // @Bitmask: 0:Servo 1, 1:Servo 2, 2:Servo 3, 3:Servo 4, 4:Servo 5, 5:Servo 6, 6:Servo 7, 7:Servo 8, 8:Servo 9, 9:Servo 10, 10:Servo 11, 11:Servo 12, 12:Servo 13, 13:Servo 14, 14:Servo 15, 15:Servo 16, 16:Servo 17, 17:Servo 18, 18:Servo 19, 19:Servo 20, 20:Servo 21, 21:Servo 22, 22:Servo 23, 23:Servo 24, 24:Servo 25, 25:Servo 26, 26:Servo 27, 27:Servo 28, 28:Servo 29, 29:Servo 30, 30:Servo 31, 31:Servo 32
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("_GPIO_MASK",  26, SRV_Channels, gpio_mask, 0),
    
#if (NUM_SERVO_CHANNELS >= 17)
    // @Group: 17_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[16], "17_",  27, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 18)
    // @Group: 18_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[17], "18_", 28, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 19)
    // @Group: 19_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[18], "19_",  29, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 20)
    // @Group: 20_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[19], "20_",  30, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 21)
    // @Group: 21_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[20], "21_",  31, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 22)
    // @Group: 22_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[21], "22_",  32, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 23)
    // @Group: 23_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[22], "23_",  33, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 24)
    // @Group: 24_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[23], "24_",  34, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 25)
    // @Group: 25_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[24], "25_",  35, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 26)
    // @Group: 26_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[25], "26_",  36, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 27)
    // @Group: 27_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[26], "27_",  37, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 28)
    // @Group: 28_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[27], "28_",  38, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 29)
    // @Group: 29_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[28], "29_",  39, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 30)
    // @Group: 30_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[29], "30_",  40, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 31)
    // @Group: 31_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[30], "31_",  41, SRV_Channels, SRV_Channel),
#endif

#if (NUM_SERVO_CHANNELS >= 32)
    // @Group: 32_
    // @Path: SRV_Channel.cpp
    AP_SUBGROUPINFO(obj_channels[31], "32_",  42, SRV_Channels, SRV_Channel),
#endif

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
#if NUM_SERVO_CHANNELS > 16
        if (i >= 16) {
            // default to GPIO, this disables the pin and stops logging
            channels[i].function.set_default(SRV_Channel::k_GPIO);
        }
#endif
    }

#if AP_FETTEC_ONEWIRE_ENABLED
    fetteconwire_ptr = &fetteconwire;
#endif

#ifndef HAL_BUILD_AP_PERIPH
    volz_ptr = &volz;
    sbus_ptr = &sbus;
    robotis_ptr = &robotis;
#endif // HAL_BUILD_AP_PERIPH
#if HAL_SUPPORT_RCOUT_SERIAL
    blheli_ptr = &blheli;
#endif
}

// SRV_Channels initialization
void SRV_Channels::init(void)
{
    // initialize BLHeli late so that all of the masks it might setup don't get trodden on by motor initialization
#if HAL_SUPPORT_RCOUT_SERIAL
    blheli_ptr->init();
#endif
#ifndef HAL_BUILD_AP_PERIPH
    hal.rcout->set_dshot_rate(_singleton->dshot_rate, AP::scheduler().get_loop_rate_hz());
#endif
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
    // slew rate limit functions
    for (slew_list *slew = _slew; slew; slew = slew->next) {
        if (is_positive(slew->max_change)) {
            // treat negative or zero slew rate as disabled
            functions[slew->func].output_scaled = constrain_float(functions[slew->func].output_scaled, slew->last_scaled_output - slew->max_change, slew->last_scaled_output + slew->max_change);
        }
        slew->last_scaled_output = functions[slew->func].output_scaled;
    }

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
        if (channels[i].valid_function()) {
            channels[i].calc_pwm(functions[channels[i].function.get()].output_scaled);
        }
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
        const bool had_pwm = SRV_Channel::have_pwm_mask & (1U<<chan);
        channels[chan].set_output_pwm(value,true);
        if (!had_pwm) {
            // clear the have PWM mask so the channel will default back to the scaled value when timeout expires
            // this is also cleared by set_output_scaled but that requires it to be re-called as some point
            // after the timeout is applied
            // note that we can't default back to a pre-override PWM value as it is not stored
            // checking had_pwm means the PWM will not change after the timeout, this was the existing behaviour
            SRV_Channel::have_pwm_mask &= ~(1U<<chan);
        }
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

#endif // HAL_BUILD_AP_PERIPH

#if HAL_SUPPORT_RCOUT_SERIAL
    // give blheli telemetry a chance to update
    blheli_ptr->update_telemetry();
#endif

#if AP_FETTEC_ONEWIRE_ENABLED
    fetteconwire_ptr->update();
#endif

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
#if APM_BUILD_COPTER_OR_HELI || APM_BUILD_TYPE(APM_BUILD_ArduPlane) || APM_BUILD_TYPE(APM_BUILD_ArduSub)
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
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        hal.rcout->write(i, 0);
    }
    push();
}

/*
  return true if a channel should be available as a GPIO
 */
bool SRV_Channels::is_GPIO(uint8_t channel)
{
    if (channel_function(channel) == SRV_Channel::k_GPIO) {
        return true;
    }
    if (_singleton != nullptr && (_singleton->gpio_mask & (1U<<channel)) != 0) {
        // user has set this channel in SERVO_GPIO_MASK
        return true;
    }
    return false;
}
