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

#if HAL_WITH_UAVCAN

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <AP_Common/AP_Common.h>
#include <AP_Common/Semaphore.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <SRV_Channel/SRV_Channel.h>
#include "AP_ToshibaCAN.h"

extern const AP_HAL::HAL& hal;

#define debug_can(level_debug, fmt, args...) do { if ((level_debug) <= AP::can().get_debug_level_driver(_driver_index)) { printf(fmt, ##args); }} while (0)

// data format for messages from flight controller
static const uint8_t COMMAND_STOP = 0x0;
static const uint8_t COMMAND_LOCK = 0x10;
static const uint8_t COMMAND_REQUEST_DATA = 0x20;
static const uint8_t COMMAND_MOTOR3 = 0x3B;
static const uint8_t COMMAND_MOTOR2 = 0x3D;
static const uint8_t COMMAND_MOTOR1 = 0x3F;

// data format for messages from ESC
static const uint8_t MOTOR_DATA1 = 0x40;
static const uint8_t MOTOR_DATA2 = 0x50;
static const uint8_t MOTOR_DATA3 = 0x60;
static const uint8_t MOTOR_DATA5 = 0x80;

// processing definitions
static const uint16_t TOSHIBACAN_OUTPUT_MIN = 6300;
static const uint16_t TOSHIBACAN_OUTPUT_MAX = 32000;
static const uint16_t TOSHIBACAN_SEND_TIMEOUT_US = 500;
static const uint8_t CAN_IFACE_INDEX = 0;

AP_ToshibaCAN::AP_ToshibaCAN()
{
    debug_can(2, "ToshibaCAN: constructed\n\r");
}

AP_ToshibaCAN *AP_ToshibaCAN::get_tcan(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_protocol_type(driver_index) != AP_BoardConfig_CAN::Protocol_Type_ToshibaCAN) {
        return nullptr;
    }
    return static_cast<AP_ToshibaCAN*>(AP::can().get_driver(driver_index));
}

// initialise ToshibaCAN bus
void AP_ToshibaCAN::init(uint8_t driver_index)
{
    _driver_index = driver_index;

    debug_can(2, "ToshibaCAN: starting init\n\r");

    if (_initialized) {
        debug_can(1, "ToshibaCAN: already initialized\n\r");
        return;
    }

    AP_HAL::CANManager* can_mgr = hal.can_mgr[driver_index];

    if (can_mgr == nullptr) {
        debug_can(1, "ToshibaCAN: no mgr for this driver\n\r");
        return;
    }

    if (!can_mgr->is_initialized()) {
        debug_can(1, "ToshibaCAN: mgr not initialized\n\r");
        return;
    }

    _can_driver = can_mgr->get_driver();

    if (_can_driver == nullptr) {
        debug_can(1, "ToshibaCAN: no CAN driver\n\r");
        return;
    }

    // start calls to loop in separate thread
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ToshibaCAN::loop, void), _thread_name, 4096, AP_HAL::Scheduler::PRIORITY_MAIN, 1)) {
        debug_can(1, "ToshibaCAN: couldn't create thread\n\r");
        return;
    }

    _initialized = true;

    debug_can(2, "ToshibaCAN: init done\n\r");

    return;
}

// loop to send output to ESCs in background thread
void AP_ToshibaCAN::loop()
{
    uavcan::MonotonicTime timeout;
    const uint32_t timeout_us = MIN(AP::scheduler().get_loop_period_us(), TOSHIBACAN_SEND_TIMEOUT_US);

    while (true) {
        if (!_initialized) {
            // if not initialised wait 2ms
            debug_can(2, "ToshibaCAN: not initialized\n\r");
            hal.scheduler->delay_microseconds(2000);
            continue;
        }

        // check for updates
        if (update_count == update_count_sent) {
            hal.scheduler->delay_microseconds(50);
            continue;
        }

        // prepare commands and frames
        if (send_stage == 0) {
            motor_lock_cmd_t unlock_cmd = {};
            motor_rotation_cmd_t mot_rot_cmd1;
            motor_rotation_cmd_t mot_rot_cmd2;
            motor_rotation_cmd_t mot_rot_cmd3;
            {
                // take semaphore to read scaled motor outputs
                WITH_SEMAPHORE(_rc_out_sem);

                // prepare command to lock or unlock motors
                unlock_cmd.motor1 = (_scaled_output[0] == 0) ? 2 : 1;
                unlock_cmd.motor2 = (_scaled_output[1] == 0) ? 2 : 1;
                unlock_cmd.motor3 = (_scaled_output[2] == 0) ? 2 : 1;
                unlock_cmd.motor4 = (_scaled_output[3] == 0) ? 2 : 1;
                unlock_cmd.motor5 = (_scaled_output[4] == 0) ? 2 : 1;
                unlock_cmd.motor6 = (_scaled_output[5] == 0) ? 2 : 1;
                unlock_cmd.motor7 = (_scaled_output[6] == 0) ? 2 : 1;
                unlock_cmd.motor8 = (_scaled_output[7] == 0) ? 2 : 1;
                unlock_cmd.motor9 = (_scaled_output[8] == 0) ? 2 : 1;
                unlock_cmd.motor10 = (_scaled_output[9] == 0) ? 2 : 1;
                unlock_cmd.motor11 = (_scaled_output[10] == 0) ? 2 : 1;
                unlock_cmd.motor12 = (_scaled_output[11] == 0) ? 2 : 1;

                // prepare command to spin motors in bank1
                mot_rot_cmd1.motor1 = htobe16(_scaled_output[0]);
                mot_rot_cmd1.motor2 = htobe16(_scaled_output[1]);
                mot_rot_cmd1.motor3 = htobe16(_scaled_output[2]);
                mot_rot_cmd1.motor4 = htobe16(_scaled_output[3]);

                // prepare message to spin motors in bank2
                mot_rot_cmd2.motor1 = htobe16(_scaled_output[4]);
                mot_rot_cmd2.motor2 = htobe16(_scaled_output[5]);
                mot_rot_cmd2.motor3 = htobe16(_scaled_output[6]);
                mot_rot_cmd2.motor4 = htobe16(_scaled_output[7]);

                // prepare message to spin motors in bank3
                mot_rot_cmd3.motor1 = htobe16(_scaled_output[8]);
                mot_rot_cmd3.motor2 = htobe16(_scaled_output[9]);
                mot_rot_cmd3.motor3 = htobe16(_scaled_output[10]);
                mot_rot_cmd3.motor4 = htobe16(_scaled_output[11]);

                // copy update time
                update_count_buffered = update_count;
            }
            unlock_frame = {(uint8_t)COMMAND_LOCK, unlock_cmd.data, sizeof(unlock_cmd.data)};
            mot_rot_frame1 = {((uint8_t)COMMAND_MOTOR1 & uavcan::CanFrame::MaskStdID), mot_rot_cmd1.data, sizeof(mot_rot_cmd1.data)};
            mot_rot_frame2 = {((uint8_t)COMMAND_MOTOR2 & uavcan::CanFrame::MaskStdID), mot_rot_cmd2.data, sizeof(mot_rot_cmd2.data)};
            mot_rot_frame3 = {((uint8_t)COMMAND_MOTOR3 & uavcan::CanFrame::MaskStdID), mot_rot_cmd3.data, sizeof(mot_rot_cmd3.data)};

            // advance to next stage
            send_stage++;
        }

        // send unlock command
        if (send_stage == 1) {
            timeout = uavcan::MonotonicTime::fromUSec(AP_HAL::micros64() + timeout_us);
            if (!write_frame(unlock_frame, timeout)) {
                continue;
            }
            send_stage++;
        }

        // send output to motor bank3
        if (send_stage == 2) {
            timeout = uavcan::MonotonicTime::fromUSec(AP_HAL::micros64() + timeout_us);
            if (!write_frame(mot_rot_frame3, timeout)) {
                continue;
            }
            send_stage++;
        }

        // send output to motor bank2
        if (send_stage == 3) {
            timeout = uavcan::MonotonicTime::fromUSec(AP_HAL::micros64() + timeout_us);
            if (!write_frame(mot_rot_frame2, timeout)) {
                continue;
            }
            send_stage++;
        }

        // send output to motor bank1
        if (send_stage == 4) {
            timeout = uavcan::MonotonicTime::fromUSec(AP_HAL::micros64() + timeout_us);
            if (!write_frame(mot_rot_frame1, timeout)) {
                continue;
            }
        }

        // success!
        send_stage = 0;

        // record success so we don't send this frame again
        update_count_sent = update_count_buffered;
    }
}

// write frame on CAN bus
bool AP_ToshibaCAN::write_frame(uavcan::CanFrame &out_frame, uavcan::MonotonicTime timeout)
{
    // wait for space in buffer to send unlock command
    uavcan::CanSelectMasks inout_mask;
    do {
        inout_mask.read = 0;
        inout_mask.write = 1 << CAN_IFACE_INDEX;
        _select_frames[CAN_IFACE_INDEX] = &out_frame;
        _can_driver->select(inout_mask, _select_frames, timeout);

        // delay if no space is available to send
        if (!inout_mask.write) {
            hal.scheduler->delay_microseconds(50);
        }
    } while (!inout_mask.write);

    // send frame and return success
    return (_can_driver->getIface(CAN_IFACE_INDEX)->send(out_frame, timeout, 0) == 1);
}

// called from SRV_Channels
void AP_ToshibaCAN::update()
{
    // take semaphore and update outputs
    WITH_SEMAPHORE(_rc_out_sem);
    for (uint8_t i = 0; i < MIN(TOSHIBACAN_MAX_NUM_ESCS, 16); i++) {
        SRV_Channel *c = SRV_Channels::srv_channel(i);
        if (c == nullptr) {
            _scaled_output[i] = 0;
        } else {
            uint16_t pwm_out = c->get_output_pwm();
            if (pwm_out <= 1000) {
                _scaled_output[i] = 0;
            } else if (pwm_out >= 2000) {
                _scaled_output[i] = TOSHIBACAN_OUTPUT_MAX;
            } else {
                _scaled_output[i] = TOSHIBACAN_OUTPUT_MIN + (pwm_out - 1000) * 0.001f * (TOSHIBACAN_OUTPUT_MAX - TOSHIBACAN_OUTPUT_MIN);
            }
        }
    }
    update_count++;
}

#endif // HAL_WITH_UAVCAN
