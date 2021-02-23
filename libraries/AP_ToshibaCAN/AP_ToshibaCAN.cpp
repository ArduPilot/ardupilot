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

#if HAL_MAX_CAN_PROTOCOL_DRIVERS

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Common/AP_Common.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_ToshibaCAN.h"
#include <AP_Logger/AP_Logger.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define debug_can(level_debug, fmt, args...) do { AP::can().log_text(level_debug, "ToshibaCAN",  fmt, #args); } while (0)


// stupid compiler is not able to optimise this under gnu++11
// move this back when moving to gnu++17
const uint16_t AP_ToshibaCAN::TOSHIBACAN_SEND_TIMEOUT_US = 500;

AP_ToshibaCAN::AP_ToshibaCAN()
{
    debug_can(AP_CANManager::LOG_INFO, "ToshibaCAN: constructed\n\r");
    (void)COMMAND_STOP;
    (void)MOTOR_DATA5;
}

AP_ToshibaCAN *AP_ToshibaCAN::get_tcan(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) != AP_CANManager::Driver_Type_ToshibaCAN) {
        return nullptr;
    }
    return static_cast<AP_ToshibaCAN*>(AP::can().get_driver(driver_index));
}


bool AP_ToshibaCAN::add_interface(AP_HAL::CANIface* can_iface) {
    if (_can_iface != nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "ToshibaCAN: Multiple Interface not supported\n\r");
        return false;
    }

    _can_iface = can_iface;

    if (_can_iface == nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "ToshibaCAN: CAN driver not found\n\r");
        return false;
    }

    if (!_can_iface->is_initialized()) {
        debug_can(AP_CANManager::LOG_ERROR, "ToshibaCAN: Driver not initialized\n\r");
        return false;
    }

    if (!_can_iface->set_event_handle(&_event_handle)) {
        debug_can(AP_CANManager::LOG_ERROR, "ToshibaCAN: Cannot add event handle\n\r");
        return false;
    }
    return true;
}


// initialise ToshibaCAN bus
void AP_ToshibaCAN::init(uint8_t driver_index, bool enable_filters)
{
    _driver_index = driver_index;

    debug_can(AP_CANManager::LOG_DEBUG, "ToshibaCAN: starting init\n\r");

    if (_initialized) {
        debug_can(AP_CANManager::LOG_ERROR, "ToshibaCAN: already initialized\n\r");
        return;
    }

    if (_can_iface == nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "ToshibaCAN: Interface not found\n\r");
        return;
    }

    // start calls to loop in separate thread
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ToshibaCAN::loop, void), _thread_name, 4096, AP_HAL::Scheduler::PRIORITY_MAIN, 1)) {
        debug_can(AP_CANManager::LOG_ERROR, "ToshibaCAN: couldn't create thread\n\r");
        return;
    }

    _initialized = true;

    debug_can(AP_CANManager::LOG_DEBUG, "ToshibaCAN: init done\n\r");

    return;
}

// loop to send output to ESCs in background thread
void AP_ToshibaCAN::loop()
{
    uint64_t timeout = 0;
    const uint32_t timeout_us = MIN(AP::scheduler().get_loop_period_us(), TOSHIBACAN_SEND_TIMEOUT_US);

    while (true) {
        if (!_initialized) {
            // if not initialised wait 2ms
            debug_can(AP_CANManager::LOG_DEBUG, "ToshibaCAN: not initialized\n\r");
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
            mot_rot_frame1 = {((uint8_t)COMMAND_MOTOR1 & AP_HAL::CANFrame::MaskStdID), mot_rot_cmd1.data, sizeof(mot_rot_cmd1.data)};
            mot_rot_frame2 = {((uint8_t)COMMAND_MOTOR2 & AP_HAL::CANFrame::MaskStdID), mot_rot_cmd2.data, sizeof(mot_rot_cmd2.data)};
            mot_rot_frame3 = {((uint8_t)COMMAND_MOTOR3 & AP_HAL::CANFrame::MaskStdID), mot_rot_cmd3.data, sizeof(mot_rot_cmd3.data)};

            // advance to next stage
            send_stage++;
        }

        // send unlock command
        if (send_stage == 1) {
            timeout = AP_HAL::native_micros64() + timeout_us;
            if (!write_frame(unlock_frame, timeout)) {
                continue;
            }
            send_stage++;
        }

        // send output to motor bank3
        if (send_stage == 2) {
            timeout = AP_HAL::native_micros64() + timeout_us;
            if (!write_frame(mot_rot_frame3, timeout)) {
                continue;
            }
            send_stage++;
        }

        // send output to motor bank2
        if (send_stage == 3) {
            timeout = AP_HAL::native_micros64() + timeout_us;
            if (!write_frame(mot_rot_frame2, timeout)) {
                continue;
            }
            send_stage++;
        }

        // send output to motor bank1
        if (send_stage == 4) {
            timeout = AP_HAL::native_micros64() + timeout_us;
            if (!write_frame(mot_rot_frame1, timeout)) {
                continue;
            }
            send_stage++;
        }

        // check if we should request update from ESCs
        if (send_stage == 5) {
            uint32_t now_ms = AP_HAL::native_millis();
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
                motor_request_data_cmd_t request_data_cmd = get_motor_request_data_cmd(1);
                AP_HAL::CANFrame request_data_frame;
                request_data_frame = {(uint8_t)COMMAND_REQUEST_DATA, request_data_cmd.data, sizeof(request_data_cmd.data)};

                // send request data command
                timeout = AP_HAL::native_micros64() + timeout_us;
                if (!write_frame(request_data_frame, timeout)) {
                    continue;
                }

                // increment count to request temperature and usage
                _telemetry_temp_req_counter++;
                _telemetry_usage_req_counter++;
            }

            send_stage++;
        }

        // check if we should request temperature from ESCs
        if (send_stage == 6) {
            if (_telemetry_temp_req_counter > 10) {
                _telemetry_temp_req_counter = 0;

                // prepare command to request data2 (temperature) from all ESCs
                motor_request_data_cmd_t request_data_cmd = get_motor_request_data_cmd(2);
                AP_HAL::CANFrame request_data_frame;
                request_data_frame = {(uint8_t)COMMAND_REQUEST_DATA, request_data_cmd.data, sizeof(request_data_cmd.data)};

                // send request data command
                timeout = AP_HAL::native_micros64() + timeout_us;
                if (!write_frame(request_data_frame, timeout)) {
                    continue;
                }
            }

            send_stage++;
        }

        // check if we should request usage from ESCs
        if (send_stage == 7) {
            if (_telemetry_usage_req_counter > 100) {
                _telemetry_usage_req_counter = 0;

                // prepare command to request data2 (temperature) from all ESCs
                motor_request_data_cmd_t request_data_cmd = get_motor_request_data_cmd(3);
                AP_HAL::CANFrame request_data_frame;
                request_data_frame = {(uint8_t)COMMAND_REQUEST_DATA, request_data_cmd.data, sizeof(request_data_cmd.data)};

                // send request data command
                timeout = AP_HAL::native_micros64() + timeout_us;
                if (!write_frame(request_data_frame, timeout)) {
                    continue;
                }
            }
            send_stage++;
        }

        // check for replies from ESCs
        if (send_stage == 8) {
            AP_HAL::CANFrame recv_frame;
            while (read_frame(recv_frame, timeout)) {
                // decode rpm and voltage data
                if ((recv_frame.id >= MOTOR_DATA1) && (recv_frame.id <= MOTOR_DATA1 + TOSHIBACAN_MAX_NUM_ESCS)) {
                    // copy contents to our structure
                    motor_reply_data1_t reply_data;
                    memcpy(reply_data.data, recv_frame.data, sizeof(reply_data.data));
                    // store response in telemetry array
                    const uint8_t esc_id = recv_frame.id - MOTOR_DATA1;
                    if (esc_id < TOSHIBACAN_MAX_NUM_ESCS) {
                        update_rpm(esc_id, (int16_t)be16toh(reply_data.rpm));

                        // update total current
                        const uint32_t now_ms = AP_HAL::native_millis();
                        const uint32_t diff_ms = now_ms - _telemetry[esc_id].last_update_ms;
                        TelemetryData t {};
                        t.voltage = float(be16toh(reply_data.voltage_mv)) * 0.001f;  // millivolts to volts
                        t.current = MAX((int16_t)be16toh(reply_data.current_ma), 0) * (4.0f * 0.001f); // milli-amps to amps
                        if (diff_ms <= 1000) {
                            // convert centi-amps miliseconds to mAh
                            _telemetry[esc_id].current_tot_mah += t.current * diff_ms * amp_ms_to_mah;
                        }
                        t.consumption_mah = _telemetry[esc_id].current_tot_mah;
                        update_telem_data(esc_id, t,
                            AP_ESC_Telem_Backend::TelemetryType::CURRENT
                                | AP_ESC_Telem_Backend::TelemetryType::VOLTAGE
                                | AP_ESC_Telem_Backend::TelemetryType::CONSUMPTION);

                        _telemetry[esc_id].last_update_ms = now_ms;
                        _esc_present_bitmask_recent |= ((uint32_t)1 << esc_id);
                    }
                }

                // decode temperature data
                if ((recv_frame.id >= MOTOR_DATA2) && (recv_frame.id <= MOTOR_DATA2 + TOSHIBACAN_MAX_NUM_ESCS)) {
                    // motor data2 data format is 8 bytes (64 bits)
                    //    10 bits: U temperature
                    //    10 bits: V temperature
                    //    10 bits: W temperature
                    //    10 bits: motor temperature
                    //    remaining 24 bits: reserved
                    const uint16_t u_temp = ((uint16_t)recv_frame.data[0] << 2) | ((uint16_t)recv_frame.data[1] >> 6);
                    const uint16_t v_temp = (((uint16_t)recv_frame.data[1] & (uint16_t)0x3F) << 4) | (((uint16_t)recv_frame.data[2] & (uint16_t)0xF0) >> 4);
                    const uint16_t w_temp = (((uint16_t)recv_frame.data[2] & (uint16_t)0x0F) << 6) | (((uint16_t)recv_frame.data[3] & (uint16_t)0xFC) >> 2);
                    const uint16_t motor_temp = (((uint16_t)recv_frame.data[3] & (uint16_t)0x03) << 8) | ((uint16_t)recv_frame.data[4]);
                    const uint16_t temp_max = MAX(u_temp, MAX(v_temp, w_temp));

                    // store response in telemetry array
                    uint8_t esc_id = recv_frame.id - MOTOR_DATA2;
                    if (esc_id < TOSHIBACAN_MAX_NUM_ESCS) {
                        const int16_t esc_temp_deg = temp_max < 100 ? 0 : temp_max / 5 - 20;
                        const int16_t motor_temp_deg = motor_temp < 100 ? 0 : motor_temp / 5 - 20;
                        _esc_present_bitmask_recent |= ((uint32_t)1 << esc_id);

                        TelemetryData t {
                            .temperature_cdeg = int16_t(esc_temp_deg * 100)
                        };
                        t.motor_temp_cdeg = int16_t(motor_temp_deg * 100);
                        update_telem_data(esc_id, t, AP_ESC_Telem_Backend::TelemetryType::MOTOR_TEMPERATURE |
                            AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE);
                    }
                }

                // decode cumulative usage data
                if ((recv_frame.id >= MOTOR_DATA3) && (recv_frame.id <= MOTOR_DATA3 + TOSHIBACAN_MAX_NUM_ESCS)) {
                    // motor data3 data format is 8 bytes (64 bits)
                    //    3 bytes: usage in seconds
                    //    2 bytes: number of times rotors started and stopped
                    //    3 bytes: reserved
                    const uint32_t usage_sec = ((uint32_t)recv_frame.data[0] << 16) | ((uint32_t)recv_frame.data[1] << 8) | (uint32_t)recv_frame.data[2];

                    // store response in telemetry array
                    uint8_t esc_id = recv_frame.id - MOTOR_DATA3;
                    if (esc_id < TOSHIBACAN_MAX_NUM_ESCS) {
                        _esc_present_bitmask_recent |= ((uint32_t)1 << esc_id);

                        TelemetryData t {};
                        t.usage_s = usage_sec;
                        update_telem_data(esc_id, t, AP_ESC_Telem_Backend::TelemetryType::USAGE);
                    }
                }
            }

            // update bitmask of escs that replied
            update_esc_present_bitmask();
        }

        // success!
        send_stage = 0;

        // record success so we don't send this frame again
        update_count_sent = update_count_buffered;
    }
}

// write frame on CAN bus
bool AP_ToshibaCAN::write_frame(AP_HAL::CANFrame &out_frame, uint64_t timeout)
{
    // wait for space in buffer to send command

    bool read_select = false;
    bool write_select = true;
    bool ret;
    do {
        ret = _can_iface->select(read_select, write_select, &out_frame, timeout);
        if (!ret || !write_select) {
            // delay if no space is available to send
            hal.scheduler->delay_microseconds(50);
        }
    } while (!ret || !write_select);

    // send frame and return success
    return (_can_iface->send(out_frame, timeout, AP_HAL::CANIface::AbortOnError) == 1);
}

// read frame on CAN bus, returns true on success
bool AP_ToshibaCAN::read_frame(AP_HAL::CANFrame &recv_frame, uint64_t timeout)
{
    // wait for space in buffer to read
    bool read_select = true;
    bool write_select = false;
    int ret = _can_iface->select(read_select, write_select, nullptr, timeout);
    if (!ret || !read_select) {
        // return false if no data is available to read
        return false;
    }
    uint64_t time;
    AP_HAL::CANIface::CanIOFlags flags {};

    // read frame and return success
    return (_can_iface->receive(recv_frame, time, flags) == 1);
}

// update esc_present_bitmask
void AP_ToshibaCAN::update_esc_present_bitmask()
{
    // recently detected escs are immediately considered present
    _esc_present_bitmask |= _esc_present_bitmask_recent;

    // escs that don't respond disappear in 1 to 2 seconds
    // set the _esc_present_bitmask to the "recent" bitmask and
    // clear the "recent" bitmask every second
    uint32_t now_ms = AP_HAL::native_millis();
    if (now_ms - _esc_present_update_ms > 1000) {
        _esc_present_bitmask = _esc_present_bitmask_recent;
        _esc_present_bitmask_recent = 0;
        _esc_present_update_ms = now_ms;
    }
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
}

// helper function to create motor_request_data_cmd_t
AP_ToshibaCAN::motor_request_data_cmd_t AP_ToshibaCAN::get_motor_request_data_cmd(uint8_t request_id) const
{
    motor_request_data_cmd_t req_data_cmd = {};
    req_data_cmd.motor1 = request_id;
    req_data_cmd.motor2 = request_id;
    req_data_cmd.motor3 = request_id;
    req_data_cmd.motor4 = request_id;
    req_data_cmd.motor5 = request_id;
    req_data_cmd.motor6 = request_id;
    req_data_cmd.motor7 = request_id;
    req_data_cmd.motor8 = request_id;
    req_data_cmd.motor9 = request_id;
    req_data_cmd.motor10 = request_id;
    req_data_cmd.motor11 = request_id;
    req_data_cmd.motor12 = request_id;
    return req_data_cmd;
}

#endif // HAL_NUM_CAN_IFACES
