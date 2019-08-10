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

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <AP_Common/AP_Common.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_ToshibaCAN.h"
#include <AP_Logger/AP_Logger.h>

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

// telemetry definitions
static const uint32_t TOSHIBA_CAN_ESC_UPDATE_MS = 100;

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
void AP_ToshibaCAN::init(uint8_t driver_index, bool enable_filters)
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
            send_stage++;
        }

        // check if we should request update from ESCs
        if (send_stage == 5) {
            uint32_t now_ms = AP_HAL::millis();
            uint32_t diff_ms = now_ms - _telemetry_req_ms;

            // check if 100ms has passed since last update request
            if (diff_ms >= TOSHIBA_CAN_ESC_UPDATE_MS) {
                // set _telem_req_ms to time we ideally should have requested update
                if (diff_ms >= 2 * TOSHIBA_CAN_ESC_UPDATE_MS) {
                    _telemetry_req_ms = now_ms;
                } else {
                    _telemetry_req_ms += TOSHIBA_CAN_ESC_UPDATE_MS;
                }

                // prepare command to request data1 (rpm and voltage) from all ESCs
                motor_request_data_cmd_t request_data_cmd = {};
                request_data_cmd.motor1 = 1;
                request_data_cmd.motor2 = 1;
                request_data_cmd.motor3 = 1;
                request_data_cmd.motor4 = 1;
                request_data_cmd.motor5 = 1;
                request_data_cmd.motor6 = 1;
                request_data_cmd.motor7 = 1;
                request_data_cmd.motor8 = 1;
                request_data_cmd.motor9 = 1;
                request_data_cmd.motor10 = 1;
                request_data_cmd.motor11 = 1;
                request_data_cmd.motor12 = 1;
                uavcan::CanFrame request_data_frame;
                request_data_frame = {(uint8_t)COMMAND_REQUEST_DATA, request_data_cmd.data, sizeof(request_data_cmd.data)};

                // send request data command
                timeout = uavcan::MonotonicTime::fromUSec(AP_HAL::micros64() + timeout_us);
                if (!write_frame(request_data_frame, timeout)) {
                    continue;
                }

                // increment count to request temperature
                _telemetry_temp_req_counter++;
            }

            send_stage++;
        }

        // check if we should request temperature from ESCs
        if (send_stage == 6) {
            if (_telemetry_temp_req_counter > 10) {
                _telemetry_temp_req_counter = 0;

                // prepare command to request data2 (temperature) from all ESCs
                motor_request_data_cmd_t request_data_cmd = {};
                request_data_cmd.motor1 = 2;
                request_data_cmd.motor2 = 2;
                request_data_cmd.motor3 = 2;
                request_data_cmd.motor4 = 2;
                request_data_cmd.motor5 = 2;
                request_data_cmd.motor6 = 2;
                request_data_cmd.motor7 = 2;
                request_data_cmd.motor8 = 2;
                request_data_cmd.motor9 = 2;
                request_data_cmd.motor10 = 2;
                request_data_cmd.motor11 = 2;
                request_data_cmd.motor12 = 2;
                uavcan::CanFrame request_data_frame;
                request_data_frame = {(uint8_t)COMMAND_REQUEST_DATA, request_data_cmd.data, sizeof(request_data_cmd.data)};

                // send request data command
                timeout = uavcan::MonotonicTime::fromUSec(AP_HAL::micros64() + timeout_us);
                if (!write_frame(request_data_frame, timeout)) {
                    continue;
                }
            }

            send_stage++;
        }

        // check for replies from ESCs
        if (send_stage == 7) {
            uavcan::CanFrame recv_frame;
            while (read_frame(recv_frame, timeout)) {
                // decode rpm and voltage data
                if ((recv_frame.id >= MOTOR_DATA1) && (recv_frame.id <= MOTOR_DATA1 + 12)) {
                    // copy contents to our structure
                    motor_reply_data1_t reply_data;
                    memcpy(reply_data.data, recv_frame.data, sizeof(reply_data.data));
                    // store response in telemetry array
                    const uint8_t esc_id = recv_frame.id - MOTOR_DATA1;
                    if (esc_id < TOSHIBACAN_MAX_NUM_ESCS) {
                        WITH_SEMAPHORE(_telem_sem);
                        _telemetry[esc_id].rpm = be16toh(reply_data.rpm);
                        _telemetry[esc_id].millivolts = be16toh(reply_data.millivolts);
                        _telemetry[esc_id].count++;
                        _telemetry[esc_id].new_data = true;
                        _esc_present_bitmask |= ((uint32_t)1 << esc_id);
                    }
                }

                // decode temperature data
                if ((recv_frame.id >= MOTOR_DATA2) && (recv_frame.id <= MOTOR_DATA2 + 12)) {
                    // motor data2 data format is 8 bytes (64 bits)
                    //    10 bits: U temperature
                    //    10 bits: V temperature
                    //    10 bits: W temperature
                    //    10 bits: motor temperature
                    //    remaining 24 bits: reserved
                    const uint16_t u_temp = ((uint16_t)recv_frame.data[0] << 2) | ((uint16_t)recv_frame.data[1] >> 6);
                    const uint16_t v_temp = (((uint16_t)recv_frame.data[1] & (uint16_t)0x3F) << 4) | (((uint16_t)recv_frame.data[2] & (uint16_t)0xF0) >> 4);
                    const uint16_t w_temp = (((uint16_t)recv_frame.data[2] & (uint16_t)0x0F) << 6) | (((uint16_t)recv_frame.data[3] & (uint16_t)0xFC) >> 2);
                    const uint16_t temp_max = MAX(u_temp, MAX(v_temp, w_temp));

                    // store repose in telemetry array
                    uint8_t esc_id = recv_frame.id - MOTOR_DATA2;
                    if (esc_id < TOSHIBACAN_MAX_NUM_ESCS) {
                        WITH_SEMAPHORE(_telem_sem);
                        _telemetry[esc_id].temperature = temp_max < 20 ? 0 : temp_max / 5 - 20;
                        _esc_present_bitmask |= ((uint32_t)1 << esc_id);
                    }
                }
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
    // wait for space in buffer to send command
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
    return (_can_driver->getIface(CAN_IFACE_INDEX)->send(out_frame, timeout, uavcan::CanIOFlagAbortOnError) == 1);
}

// read frame on CAN bus, returns true on success
bool AP_ToshibaCAN::read_frame(uavcan::CanFrame &recv_frame, uavcan::MonotonicTime timeout)
{
    // wait for space in buffer to read
    uavcan::CanSelectMasks inout_mask;
    inout_mask.read = 1 << CAN_IFACE_INDEX;
    inout_mask.write = 0;
    _select_frames[CAN_IFACE_INDEX] = &recv_frame;
    _can_driver->select(inout_mask, _select_frames, timeout);

    // return false if no data is available to read
    if (!inout_mask.read) {
        return false;
    }
    uavcan::MonotonicTime time;
    uavcan::UtcTime utc_time;
    uavcan::CanIOFlags flags {};

    // read frame and return success
    return (_can_driver->getIface(CAN_IFACE_INDEX)->receive(recv_frame, time, utc_time, flags) == 1);
}

// called from SRV_Channels
void AP_ToshibaCAN::update()
{
    // take semaphore and update outputs
    {
        WITH_SEMAPHORE(_rc_out_sem);
        const bool armed = hal.util->get_soft_armed();
        for (uint8_t i = 0; i < MIN(TOSHIBACAN_MAX_NUM_ESCS, 16); i++) {
            const SRV_Channel *c = SRV_Channels::srv_channel(i);
            if (!armed || (c == nullptr)) {
                _scaled_output[i] = 0;
            } else {
                const uint16_t pwm_out = c->get_output_pwm();
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

    // log ESCs telemetry info
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger && logger->logging_enabled()) {
        WITH_SEMAPHORE(_telem_sem);

        // log if any new data received.  Logging only supports up to 8 ESCs
        const uint64_t time_us = AP_HAL::micros64();
        for (uint8_t i = 0; i < MIN(TOSHIBACAN_MAX_NUM_ESCS, 8); i++) {
            if (_telemetry[i].new_data) {
                logger->Write_ESC(i, time_us,
                              _telemetry[i].rpm * 100U,
                              _telemetry[i].millivolts * 0.1f,
                              0,
                              _telemetry[i].temperature * 100.0f,
                              0);
                _telemetry[i].new_data = false;
            }
        }
    }
}

// send ESC telemetry messages over MAVLink
void AP_ToshibaCAN::send_esc_telemetry_mavlink(uint8_t mav_chan)
{
    // compile time check this method handles the correct number of motors
    static_assert(TOSHIBACAN_MAX_NUM_ESCS == 12, "update AP_ToshibaCAN::send_esc_telemetry_mavlink only handles 12 motors");

    // return immediately if no ESCs have been found
    if (_esc_present_bitmask == 0) {
        return;
    }

    // return if no space in output buffer to send mavlink messages
    if (!HAVE_PAYLOAD_SPACE((mavlink_channel_t)mav_chan, ESC_TELEMETRY_1_TO_4)) {
        return;
    }

    // output telemetry messages
    {
        // take semaphore to access telemetry data
        WITH_SEMAPHORE(_telem_sem);

        // loop through 3 groups of 4 ESCs
        for (uint8_t i = 0; i < 3; i++) {

            // skip this group of ESCs if no data to send
            if ((_esc_present_bitmask & ((uint32_t)0x0F << i*4)) == 0) {
                continue;
            }

            // arrays to hold output
            uint8_t temperature[4] {};
            uint16_t voltage[4] {};
            uint16_t rpm[4] {};
            uint16_t count[4] {};
            uint16_t nosup[4] {};   // single empty array for unsupported current and current_tot

            // fill in output arrays
            for (uint8_t j = 0; j < 4; j++) {
                uint8_t esc_id = i * 4 + j;
                temperature[j] = _telemetry[esc_id].temperature;
                voltage[j] = _telemetry[esc_id].millivolts * 0.1f;
                rpm[j] = _telemetry[esc_id].rpm;
                count[j] = _telemetry[esc_id].count;
            }

            // send messages
            switch (i) {
                case 0:
                    mavlink_msg_esc_telemetry_1_to_4_send((mavlink_channel_t)mav_chan, temperature, voltage, nosup, nosup, rpm, count);
                    break;
                case 1:
                    mavlink_msg_esc_telemetry_5_to_8_send((mavlink_channel_t)mav_chan, temperature, voltage, nosup, nosup, rpm, count);
                    break;
                case 2:
                    mavlink_msg_esc_telemetry_9_to_12_send((mavlink_channel_t)mav_chan, temperature, voltage, nosup, nosup, rpm, count);
                    break;
                default:
                    break;
            }
        }
    }
}

#endif // HAL_WITH_UAVCAN
