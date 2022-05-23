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

#include "AP_BattMonitor_TattuCAN.h"

#if AP_BATT_MONITOR_TATTU_ENABLED
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

/// Constructor
AP_BattMonitor_TattuCAN::AP_BattMonitor_TattuCAN(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params) :
    CANSensor("Tattu"),
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    // starts with not healthy
    _state.healthy = false;
    register_driver(AP_CANManager::Driver_Type_Tattu);
}

void AP_BattMonitor_TattuCAN::handle_frame(AP_HAL::CANFrame &frame)
{
    if (frame.dlc == 0) {
        // sanity check for if there's no payload
        return;
    }

    const uint8_t payload_len = (frame.dlc - 1); // last payload byte is tail which is not _message data
    const uint8_t tail_byte = frame.data[frame.dlc-1];
    const bool start = (tail_byte & TAIL_BYTE_BITS_START_OF_TRANSFER) != 0;
    const bool end = (tail_byte & TAIL_BYTE_BITS_END_OF_TRANSFER) != 0;
    //const bool toggle = (tail_byte & TAIL_BYTE_BITS_TOGGLE) != 0;
    //const bool id = (tail_byte & TAIL_BYTE_TRANSFER_ID_MASK);


    if ((start && end) || (_message_offset == 0 && !start)) {
        // invalid or we're expecting a start and did't get one (re-sync)
        _message_resync++;
        return;

    } else if (start) {
        // start of a frame
        _message_offset = 0;
        _message_toggle_expected = 0;
        
    } else if ((_message_offset + frame.dlc - 1) > sizeof(_message)) {
        // buffer overflow, reset
        // TODO: this only supports 12S batteries. If a 14S is connected, 4 more bytes come in the middle of the packet
        _message_offset = 0;
        _message_resync++;
        return;
    }

    memcpy(&data[_message_offset], frame.data, payload_len);
    _message_offset += payload_len;
    _message_toggle_expected = !_message_toggle_expected;

    if (!end) {
        return;
    }

    // message complete!
    memcpy((uint8_t*)&_message, data, _message_offset);

    WITH_SEMAPHORE(_sem_battmon);

    _interim_state.last_time_micros = AP_HAL::micros();
    _interim_state.voltage = _message.voltage_mV * 0.001f;
    _interim_state.current_amps = _message.current_mA * -0.01f;

    _interim_state.temperature_time = AP_HAL::millis();
    _interim_state.temperature = _message.temperature_C;

    _interim_state.consumed_mah = MIN(_message.standard_capacity_mAh, _message.standard_capacity_mAh - _message.remaining_capacity_mAh) * 0.001f;
    
    _interim_state.healthy = true; // _message.health_status

    for (uint8_t i=0; i<TATTUCAN_CELL_COUNT_12S; i++) {
        _interim_state.cell_voltages.cells[i] = _message.cell_voltage_mV[i];
    }

    _have_received_a_msg = true;
    _message_offset = 0;
    _message_count_good++;
}

// read - read the voltage and current
void AP_BattMonitor_TattuCAN::read()
{
    if (!_have_received_a_msg) {
        return;
    }

    // timeout after 5 seconds
    if (_interim_state.healthy && (AP_HAL::micros() - _interim_state.last_time_micros) > 5000000) {
        _interim_state.healthy = false;
    }

    // Copy over relevant states over to main state
    WITH_SEMAPHORE(_sem_battmon);
    _state.temperature = _interim_state.temperature;
    _state.temperature_time = _interim_state.temperature_time;
    _state.voltage = _interim_state.voltage;
    _state.current_amps = _interim_state.current_amps;
    _state.consumed_mah = _interim_state.consumed_mah;
    //_state.consumed_wh = _interim_state.consumed_wh;
    _state.last_time_micros = _interim_state.last_time_micros;
    _state.healthy = _interim_state.healthy;
    //_state.time_remaining = _interim_state.time_remaining;
    //_state.has_time_remaining = _interim_state.has_time_remaining;
    //_state.is_powering_off = _interim_state.is_powering_off;
    memcpy(_state.cell_voltages.cells, _interim_state.cell_voltages.cells, sizeof(_state.cell_voltages));
}

/// capacity_remaining_pct - returns true if the percentage is valid and writes to percentage argument
bool AP_BattMonitor_TattuCAN::capacity_remaining_pct(uint8_t &percentage) const
{
    if (!_have_received_a_msg) {
        return false;
    }

    percentage = _message.remaining_percent;
    return true;
}

/// get_cycle_count - return true if cycle count can be provided and fills in cycles argument
bool AP_BattMonitor_TattuCAN::get_cycle_count(uint16_t &cycles) const
{
    if (!_have_received_a_msg) {
        return false;
    }

    cycles = _message.cycle_life;
    return true;
}

// return mavlink fault bitmask (see MAV_BATTERY_FAULT enum)
uint32_t AP_BattMonitor_TattuCAN::get_mavlink_fault_bitmask() const
{
    // TODO: populate errors
    return 0;
}

#endif
