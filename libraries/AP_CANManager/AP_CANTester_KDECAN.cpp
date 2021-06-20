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
 * Author: Francisco Ferreira
 * Modified for CANManager by Siddharth B Purohit
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>

#if HAL_MAX_CAN_PROTOCOL_DRIVERS > 1 && !HAL_MINIMIZE_FEATURES && HAL_CANMANAGER_ENABLED
#include "AP_CANTester_KDECAN.h"
#include "AP_CANManager.h"
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Vehicle/AP_Vehicle.h>

#define debug_can(level_debug, fmt, args...) do { AP::can().log_text(level_debug, "TestKDECAN",  fmt, #args); } while (0)
extern const AP_HAL::HAL& hal;

void AP_CANTester_KDECAN::count_msg(uint32_t frame_id)
{
    for (uint16_t i=0; i<ARRAY_SIZE(counters); i++) {
        if (counters[i].frame_id == frame_id) {
            counters[i].count++;
            break;
        }
        if (counters[i].frame_id == 0) {
            counters[i].frame_id = frame_id;
            counters[i].count++;
            break;
        }
    }
}

bool AP_CANTester_KDECAN::init(AP_HAL::CANIface* can_iface)
{
    _can_iface = can_iface;
    if (_can_iface == nullptr) {
        debug_can(AP_CANManager::LOG_ERROR, "CANIface is null, abort!");
        return false;
    }

    if (!_can_iface->is_initialized()) {
        debug_can(AP_CANManager::LOG_ERROR, "Can not initialised");
        return false;
    }

    if (!_can_iface->set_event_handle(&_event_handle)) {
        debug_can(AP_CANManager::LOG_ERROR, "Failed to set Event Handle");
        return false;
    }
    debug_can(AP_CANManager::LOG_ERROR, "init done");
    return true;
}

void AP_CANTester_KDECAN::loop(void)
{
    if (_can_iface == nullptr) {
        return;
    }

    AP_HAL::CANFrame empty_frame { (0 | AP_HAL::CANFrame::FlagEFF), nullptr, 0 };
    bool read_select = true;
    bool write_select = false;
    bool select_ret = _can_iface->select(read_select, write_select, nullptr, AP_HAL::micros64() + 1000);

    if (select_ret && read_select) {
        AP_HAL::CANFrame recv_frame;
        uint64_t rx_time;
        AP_HAL::CANIface::CanIOFlags flags {};
        int16_t res = _can_iface->receive(recv_frame, rx_time, flags);
        if (res == 1) {
            uint32_t id = recv_frame.id & AP_HAL::CANFrame::MaskExtID;
            uint8_t object_address = id & 0xFF;
            uint8_t esc_num = uint8_t((id >> 8) & 0xFF);

            count_msg(id);

            uint8_t i = 0;
            uint8_t n = NUM_ESCS;

            if (esc_num != BROADCAST_NODE_ID) {
                for (; i < NUM_ESCS; i++) {
                    if (object_address == UPDATE_NODE_ID_OBJ_ADDR) {
                        if (_esc_info[i].mcu_id == be64toh(*((be64_t*) &(recv_frame.data[0])))) {
                            n = i + 1;
                            break;
                        }
                    } else if (_esc_info[i].node_id == esc_num) {
                        n = i + 1;
                        break;
                    }
                }
            }

            while (i < n) {
                AP_HAL::CANFrame res_frame;

                switch (object_address) {
                case ESC_INFO_OBJ_ADDR: {
                    uint8_t info[5] { 1, 2, 3, 4, 0 };

                    res_frame.dlc = 5;
                    memcpy(res_frame.data, info, 5);

                    break;
                }
                case SET_PWM_OBJ_ADDR: {
                    if ((1 << (esc_num - 2) & _mask_received_pwm) && _mask_received_pwm != ((1 << _max_node_id) - 1)) {
                        count_msg(0xFFFFFFF0);
                        _mask_received_pwm = 0;
                    }

                    _mask_received_pwm |= 1 << (esc_num - 2);

                    if (_mask_received_pwm == ((1 << _max_node_id) - 1)) {
                        count_msg(0xFFFFFFFF);
                        _mask_received_pwm = 0;
                    }

                    res_frame.dlc = 0;

                    break;
                }
                case UPDATE_NODE_ID_OBJ_ADDR: {
                    if (_esc_info[i].enum_timeout_ms != 0
                        && _esc_info[i].enum_timeout_ms >= AP_HAL::millis()) {
                        _esc_info[i].node_id = esc_num;
                        _max_node_id = MAX(_max_node_id, esc_num - 2 + 1);
                        gcs().send_text(MAV_SEVERITY_ALERT, "KDECANTester: Set node ID %d for ESC %d\n", esc_num, i);
                    }

                    _esc_info[i].enum_timeout_ms = 0;

                    res_frame.dlc = 1;
                    memcpy(res_frame.data, &(_esc_info[i].node_id), 1);

                    break;
                }
                case START_ENUM_OBJ_ADDR: {
                    _esc_info[i].enum_timeout_ms = AP_HAL::millis() + be16toh(*((be16_t*) &(recv_frame.data[0])));
                    gcs().send_text(MAV_SEVERITY_ALERT, "KDECANTester: Starting enumeration for ESC %d, timeout %u", i, (unsigned)_esc_info[i].enum_timeout_ms);
                    i++;
                    continue;
                }
                case TELEMETRY_OBJ_ADDR: {
                    uint8_t data[8] {};
                    *((be16_t*) &data[0]) = htobe16(get_random16());
                    *((be16_t*) &data[2]) = htobe16(get_random16());
                    *((be16_t*) &data[4]) = htobe16(get_random16());
                    data[6] = uint8_t(float(rand()) / RAND_MAX * 40.0f + 15);

                    res_frame.dlc = 8;
                    memcpy(res_frame.data, data, 8);
                    break;
                }
                case VOLTAGE_OBJ_ADDR:
                case CURRENT_OBJ_ADDR:
                case RPM_OBJ_ADDR:
                case TEMPERATURE_OBJ_ADDR:
                case GET_PWM_INPUT_OBJ_ADDR:
                case GET_PWM_OUTPUT_OBJ_ADDR:
                case MCU_ID_OBJ_ADDR:
                default:
                    // discard frame
                    return;
                }

                res_frame.id = (_esc_info[i].node_id << 16) | object_address | AP_HAL::CANFrame::FlagEFF;
                read_select = false;
                write_select = true;
                select_ret = _can_iface->select(read_select, write_select, &res_frame, AP_HAL::micros64() + 1000);
                if (!select_ret) {
                    break;
                }
                int16_t res2 = _can_iface->send(res_frame, AP_HAL::micros64() + 500000, 0);
                if (res2 == 1) {
                    i++;
                } else {
                    gcs().send_text(MAV_SEVERITY_ALERT, "KDECANTester: Failed to transmit frame Err: %d 0x%lx", res2, (long unsigned)res_frame.id);
                }
            }
        }
    }
}

void AP_CANTester_KDECAN::print_stats(void)
{
    hal.console->printf("KDECANTester: TimeStamp: %u\n", (unsigned)AP_HAL::micros());
    for (uint16_t i=0; i<100; i++) {
        if (counters[i].frame_id == 0) {
            break;
        }
        hal.console->printf("0x%08x: %u\n", (unsigned)counters[i].frame_id, (unsigned)counters[i].count);
        counters[i].count = 0;
    }
}

bool AP_CANTester_KDECAN::send_enumeration(uint8_t num)
{
    if (_esc_info[num].enum_timeout_ms == 0 ||
        AP_HAL::millis() > _esc_info[num].enum_timeout_ms) {
        _esc_info[num].enum_timeout_ms = 0;
        gcs().send_text(MAV_SEVERITY_ALERT, "KDECANTester: Not running enumeration for ESC %d\n", num);
        return false;
    }

    while (true) {
        uint8_t mcu[8] {};
        *((be64_t*) mcu) = htobe64(_esc_info[num].mcu_id);
        AP_HAL::CANFrame res_frame { (_esc_info[num].node_id << 16) | START_ENUM_OBJ_ADDR | AP_HAL::CANFrame::FlagEFF,
                                     mcu,
                                     8 };
        int16_t res = _can_iface->send(res_frame, AP_HAL::micros64() + 1000, 0);
        if (res == 1) {
            return true;
        }
    }
}
#endif
