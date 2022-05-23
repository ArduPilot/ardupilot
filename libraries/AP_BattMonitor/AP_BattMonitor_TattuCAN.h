/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Tom Pittenger
 */

#pragma once

#include "AP_BattMonitor.h"

#ifndef AP_BATT_MONITOR_TATTU_ENABLED
#define AP_BATT_MONITOR_TATTU_ENABLED !HAL_MINIMIZE_FEATURES && (BOARD_FLASH_SIZE > 1024) && (HAL_MAX_CAN_PROTOCOL_DRIVERS > 1)
#endif

#if AP_BATT_MONITOR_TATTU_ENABLED

#include "AP_BattMonitor_Backend.h"
#include <AP_CANManager/AP_CANSensor.h>

class AP_BattMonitor_TattuCAN : public AP_BattMonitor_Backend, public CANSensor {
public:

    /// Constructor
    AP_BattMonitor_TattuCAN(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    /// capacity_remaining_pct - returns true if the percentage is valid and writes to percentage argument
    bool capacity_remaining_pct(uint8_t &percentage) const override;

    bool has_temperature() const override { return _have_received_a_msg; }

    bool has_current() const override { return _have_received_a_msg; }

    bool has_consumed_energy() const override { return _have_received_a_msg && (_message.remaining_capacity_mAh < _message.standard_capacity_mAh); }

    bool has_time_remaining() const override { return false; }

    bool has_cell_voltages() const override { return _have_received_a_msg; }

    bool get_cycle_count(uint16_t &cycles) const override;

    // return mavlink fault bitmask (see MAV_BATTERY_FAULT enum)
    uint32_t get_mavlink_fault_bitmask() const override;

private:
    void handle_frame(AP_HAL::CANFrame &frame) override;

    static constexpr uint8_t TAIL_BYTE_BITS_START_OF_TRANSFER = (1<<7);
    static constexpr uint8_t TAIL_BYTE_BITS_END_OF_TRANSFER = (1<<6);
    static constexpr uint8_t TAIL_BYTE_BITS_TOGGLE = (1<<5);
    static constexpr uint8_t TAIL_BYTE_TRANSFER_ID_MASK = 0x1F;

    static constexpr uint8_t TATTUCAN_CELL_COUNT_12S = 12;

    HAL_Semaphore _sem_battmon;

    struct PACKED {
        uint16_t    crc;
        int16_t     manufacturer;
        int16_t     sku;
        uint16_t    voltage_mV;
        int16_t     current_mA;
        int16_t     temperature_C;
        uint16_t    remaining_percent;
        uint16_t    cycle_life;
        int16_t     health_status;  // percent
        uint16_t    cell_voltage_mV[TATTUCAN_CELL_COUNT_12S];
        uint16_t    standard_capacity_mAh;
        uint16_t    remaining_capacity_mAh;
        uint32_t    error_info;
    } _message;

    uint8_t data[sizeof(_message)];

    uint32_t _message_offset;
    bool _message_toggle_expected;

    uint32_t _frame_count_good;
    uint32_t _frame_count_bad;
    uint32_t _message_count_good;
    uint32_t _message_count_bad;
    uint32_t _message_resync;

    bool    _have_received_a_msg;
    AP_BattMonitor::BattMonitor_State _interim_state;
};

#endif // AP_BATT_MONITOR_TATTU_ENABLED
