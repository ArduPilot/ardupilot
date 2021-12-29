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
 * AP_KDECAN.cpp
 *
 *      Author: Francisco Ferreira
 */

#include "AP_KDECAN.h"

#if AP_KDECAN_ENABLED
#include <stdio.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Motors/AP_Motors.h>

extern const AP_HAL::HAL& hal;

#if HAL_CANMANAGER_ENABLED
#define debug_can(level_debug, fmt, args...) do { gcs().send_text(MAV_SEVERITY_DEBUG, fmt, ##args); } while (0)
//#define debug_can(level_debug, fmt, args...) do { AP::can().log_text(level_debug, "KDECAN", fmt, ##args); } while (0)
#else
#define debug_can(level_debug, fmt, args...)
#endif

// table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_KDECAN::var_info[] = {
    // @Param: NPOLE
    // @DisplayName: Number of motor poles
    // @Description: Sets the number of motor poles to calculate the correct RPM value
    AP_GROUPINFO("NPOLE", 1, AP_KDECAN, _telemetry.num_poles, DEFAULT_NUM_POLES),

    AP_GROUPEND
};

AP_KDECAN::AP_KDECAN(const uint8_t driver_index) : CANSensor("KDECAN")
{
    _init.driver_index = driver_index;

    AP_Param::setup_object_defaults(this, var_info);

    register_driver(AP_CANManager::Driver_Type_KDECAN);

    // start thread for receiving and sending CAN frames
    snprintf(_init.thread_name, sizeof(_init.thread_name), "kdecan_%u", _init.driver_index);
    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_KDECAN::loop, void), _init.thread_name, 4096, AP_HAL::Scheduler::PRIORITY_CAN, 0);

    debug_can(AP_CANManager::LOG_INFO, "constructed");
}

AP_KDECAN *AP_KDECAN::get_kdecan(uint8_t driver_index)
{
#if HAL_CANMANAGER_ENABLED
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) != AP_CANManager::Driver_Type_KDECAN) {
        return nullptr;
    }
    return static_cast<AP_KDECAN*>(AP::can().get_driver(driver_index));
#else
    return nullptr;
#endif
}

// parse inbound frames
void AP_KDECAN::handle_frame(AP_HAL::CANFrame &frame)
{
    if (!frame.isExtended()) {
        return;
    }

    const frame_id_t id = { .value = frame.id & AP_HAL::CANFrame::MaskExtID };

    // if (id.object_address != TELEMETRY_OBJ_ADDR) {
    //     debug_can(AP_CANManager::LOG_DEBUG, "rx id:%d, src:%d, dest:%d, len:%d", (int)id.object_address, (int)id.source_id, (int)id.destination_id, (int)frame.dlc);
    // }

    // check if frame is valid: directed at autopilot, doesn't come from broadcast and ESC was detected before
    switch (id.object_address) {
        case ESC_INFO_OBJ_ADDR:
            if (frame.dlc == 5 &&
                id.destination_id == AUTOPILOT_NODE_ID &&
                id.source_id >= ESC_NODE_ID_FIRST &&
                id.source_id < (KDECAN_MAX_NUM_ESCS + ESC_NODE_ID_FIRST))
            {
                const uint16_t bitmask = _output.present_bitmask;
                _output.present_bitmask |= (1 << (id.source_id - ESC_NODE_ID_FIRST));
                _output.max_node_id = MAX(_output.max_node_id, id.source_id - ESC_NODE_ID_FIRST + 1);

                if (bitmask != _output.present_bitmask) {
                    debug_can(AP_CANManager::LOG_DEBUG, "Found ESC id %u", id.source_id);
                }
            }
        break;

#if HAL_WITH_ESC_TELEM
        case TELEMETRY_OBJ_ADDR:
            if (id.destination_id == AUTOPILOT_NODE_ID &&
                id.source_id != BROADCAST_NODE_ID &&
                (1 << (id.source_id - ESC_NODE_ID_FIRST) & _output.present_bitmask) &&
                frame.dlc == 8)
            {
                const uint8_t idx = id.source_id - ESC_NODE_ID_FIRST;
                const uint8_t num_poles = _telemetry.num_poles > 0 ? _telemetry.num_poles : DEFAULT_NUM_POLES;
                update_rpm(idx, uint16_t(uint16_t(frame.data[4] << 8 | frame.data[5]) * 60UL * 2 / num_poles));

                TelemetryData t {
                    .temperature_cdeg = int16_t(frame.data[6] * 100),
                    .voltage = float(uint16_t(frame.data[0] << 8 | frame.data[1])) * 0.01f,
                    .current = float(uint16_t(frame.data[2] << 8 | frame.data[3])) * 0.01f,
                };
                update_telem_data(idx, t,
                    AP_ESC_Telem_Backend::TelemetryType::CURRENT |
                    AP_ESC_Telem_Backend::TelemetryType::VOLTAGE |
                    AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE);
            }
            break;
#endif // HAL_WITH_ESC_TELEM

        case UPDATE_NODE_ID_OBJ_ADDR:
            // reply from setting new node ID
            _output.present_bitmask |= 1 << (id.source_id - ESC_NODE_ID_FIRST);
            _enumeration.max_node_id = MAX(_enumeration.max_node_id, id.source_id - ESC_NODE_ID_FIRST + 1);
            debug_can(AP_CANManager::LOG_DEBUG, "Found ESC id %u", id.source_id);
            break;

        case ENUM_OBJ_ADDR:
            if (frame.dlc == 8) {
                // ESC just told us it's MCU ID in the frame.data. Now lets's set that ESC with address _enumeration.esc_num
                if (send_packet(UPDATE_NODE_ID_OBJ_ADDR, _enumeration.esc_num, 10, (uint8_t*) &frame.data, frame.dlc)) {
                    debug_can(AP_CANManager::LOG_DEBUG, "Assigning ESC id %u", id.source_id);
                    _enumeration.esc_num++;
                }
            }
            break;

        default:
            break;
    }
}

void AP_KDECAN::update()
{
    uint16_t pwm[KDECAN_MAX_NUM_ESCS] {};

    const bool armed = hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED;

    if (armed) {
        for (uint8_t i = 0; i < KDECAN_MAX_NUM_ESCS; i++) {
            if ((_output.present_bitmask & (1<<i)) && SRV_Channels::channel_function(i)) {
                pwm[i] = SRV_Channels::srv_channel(i)->get_output_pwm();
            }
        }
    }

    // queue the PWMs for loop()
    WITH_SEMAPHORE(_output.sem);
    memcpy(&_output.pwm, &pwm, sizeof(_output.pwm));
    _output.is_new = true;

#if AP_KDECAN_USE_EVENTS
    if (_output.thread_ctx != nullptr) {
        // trigger the thread to wake up immediately
        chEvtSignal(_output.thread_ctx, 1);
    }
#endif
}

void AP_KDECAN::update_enumeration()
{
    const uint32_t now_ms = AP_HAL::millis();

    switch (_enumeration.state) {
        case ENUMERATION_STATE::START:
            // enumeration_esc_num = 0;
            _output.present_bitmask = 0;
            _output.max_node_id = 0;
            _enumeration.esc_num = ESC_NODE_ID_FIRST;

            if (send_packet_uint16(ENUM_OBJ_ADDR, BROADCAST_NODE_ID, 10, ENUMERATION_TIMEOUT_MS)) {
                _enumeration.state = ENUMERATION_STATE::RUNNING;
                _enumeration.timer_ms = now_ms;
            }
            break;

        case ENUMERATION_STATE::RUNNING:
            // work is done in handle_frame()
            if (now_ms - _enumeration.timer_ms >= ENUMERATION_TIMEOUT_MS) {
                // ok, we've waited long enough..
                _enumeration.state = ENUMERATION_STATE::STOP;
            }
            break;

        case ENUMERATION_STATE::STOP:
            // tell everone we want to do enumeration for 0ms, which stops it
            if (send_packet_uint16(ENUM_OBJ_ADDR, BROADCAST_NODE_ID, 10, 0)) {
                _enumeration.state = ENUMERATION_STATE::CHECK_STATUS;
            }
            break;

        case ENUMERATION_STATE::CHECK_STATUS:
            // have everyone check in with their IDs
            if (send_packet(MCU_ID_OBJ_ADDR, BROADCAST_NODE_ID, 10)) {
                 _enumeration.state = ENUMERATION_STATE::STOPPED;
            }
            break;

        case ENUMERATION_STATE::STOPPED:
            // nothing to do.
            break;
    }
}


void AP_KDECAN::loop()
{
    uint16_t pwm[KDECAN_MAX_NUM_ESCS];

#if AP_KDECAN_USE_EVENTS
    _output.thread_ctx = chThdGetSelfX();
#endif

    while (true) {
        if (_enumeration.state != ENUMERATION_STATE::STOPPED) {
            // run enumeration at 10Hz
            update_enumeration();
            hal.scheduler->delay(100);
            continue;
        }

        if (_output.present_bitmask == 0) {
            // if we don't know of any ESCs yet, broadcast an "anyone there?" msg at 1Hz
            send_packet(ESC_INFO_OBJ_ADDR, BROADCAST_NODE_ID, 1000);
            hal.scheduler->delay(1000);
            continue;
        }

#if AP_KDECAN_USE_EVENTS
        // sleep until we get new data, but also wake up at 400Hz to send the old data again
        chEvtWaitAnyTimeout(ALL_EVENTS, chTimeUS2I(2500));
 #else
        hal.scheduler->delay_microseconds(2500); // 400Hz
#endif

        const uint32_t now_ms = AP_HAL::millis();

        // This should run at 400Hz
        {
            WITH_SEMAPHORE(_output.sem);
            if (_output.is_new) {
                _output.last_new_ms = now_ms;
                _output.is_new = false;
                memcpy(&pwm, &_output.pwm, sizeof(pwm));

            } else if (_output.last_new_ms && now_ms - _output.last_new_ms > 1000) {
                // if we haven't gotten any PWM updates for a full second, zero it
                // out so we don't just keep sending the same values forever
                memset(&pwm, 0, sizeof(pwm));
                _output.last_new_ms = 0;
            }
        }

        uint8_t index = 0;
        uint8_t retry = 0;

        while (index < KDECAN_MAX_NUM_ESCS) {
            if ((_output.present_bitmask & (1 << index)) == 0) {
                // we're not sending this index so skip it
                index++;
            } else if (send_packet_uint16(SET_PWM_OBJ_ADDR, (index + ESC_NODE_ID_FIRST), 1, pwm[index]) || retry++ >= 10) {
                // sent successfully or we've retried too many times, move on to the next
                index++;
                retry = 0;
            } else {
                // send failed, likely due to CAN TX buffer full. Delay a tiny bit and try again but only a few times
                hal.scheduler->delay_microseconds(10);
            }
        } // while index

#if HAL_WITH_ESC_TELEM
        // broadcast as request-telemetry msg to everyone
        if (now_ms - _telemetry.timer_ms >= TELEMETRY_INTERVAL_MS) {
            if (send_packet(TELEMETRY_OBJ_ADDR, BROADCAST_NODE_ID, 10)) {
                _telemetry.timer_ms = now_ms;
            }
        }
#endif // HAL_WITH_ESC_TELEM

    } // while true
}

bool AP_KDECAN::send_packet_uint16(const uint8_t address, const uint8_t dest_id, const uint32_t timeout_ms, const uint16_t data)
{
    const be16_t data_be16 = htobe16(data);
    return send_packet(address, dest_id, timeout_ms, (uint8_t*)&data_be16, sizeof(data_be16));
}

bool AP_KDECAN::send_packet(const uint8_t address, const uint8_t dest_id, const uint32_t timeout_ms, const uint8_t *data, const uint8_t data_len)
{
        // broadcast telemetry request frame
    frame_id_t id = { { .object_address = address,
                        .destination_id = dest_id,
                        .source_id = AUTOPILOT_NODE_ID,
                        .priority = 0,
                        .unused = 0 } };

    AP_HAL::CANFrame frame { (id.value | AP_HAL::CANFrame::FlagEFF), data, data_len };

    const uint64_t timeout_us = uint64_t(timeout_ms) * 1000UL;
    return write_frame(frame, timeout_us);
}

bool AP_KDECAN::pre_arm_check(char* reason, uint8_t reason_len)
{
#if defined(AP_MOTORS_ENABLE) && AP_MOTORS_ENABLE == 1
    uint16_t motors_mask = 0;
    AP_Motors *motors = AP::motors();

    if (motors) {
        motors_mask = motors->get_motor_mask();
    }

    const uint8_t num_expected_motors = __builtin_popcount(motors_mask);
    const uint8_t num_present_escs = __builtin_popcount(_output.present_bitmask);

    if (num_present_escs < num_expected_motors) {
        snprintf(reason, reason_len, "Too few ESCs detected (%u of %u)", (int)num_present_escs, (int)num_expected_motors);
        return false;
    }

    if (num_present_escs > num_expected_motors) {
        snprintf(reason, reason_len, "Too many ESCs detected (%u > %u)", (int)num_present_escs, (int)num_expected_motors);
        return false;
    }

    if (_output.max_node_id != num_expected_motors) {
        snprintf(reason, reason_len, "ESC count (%u) does not match expected count (%u), run enumeration or check motor config", (int)_output.max_node_id, (int)num_expected_motors);
        return false;
    }
#endif

    return true;
}

bool AP_KDECAN::run_enumeration(bool start_stop)
{
    if (start_stop) {
        _enumeration.state = ENUMERATION_STATE::START;
    } else if (_enumeration.state != ENUMERATION_STATE::STOPPED) {
        _enumeration.state = ENUMERATION_STATE::STOP;
    }

    return true;
}

#endif // AP_KDECAN_ENABLED

