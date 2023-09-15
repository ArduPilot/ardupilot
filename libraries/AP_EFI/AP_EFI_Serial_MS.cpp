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

#include "AP_EFI_config.h"

#if AP_EFI_SERIAL_MS_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "AP_EFI_Serial_MS.h"

#include <AP_HAL/utility/sparse-endian.h>

AP_EFI_Serial_MS::AP_EFI_Serial_MS(AP_EFI &_frontend):
    AP_EFI_Backend(_frontend)
{
    port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_EFI, 0);
}


void AP_EFI_Serial_MS::update()
{
    if (!port) {
        return;
    }

    if (read_incoming_realtime_data()) {
        copy_to_frontend();
    }

    const auto now_ms = AP_HAL::millis();
    if (now_ms - last_response_ms > 100) {
        port->discard_input();
        // Request an update from the realtime table (7).
        // The data we need start at offset 6 and ends at 129
        send_request(7, RT_FIRST_OFFSET, RT_LAST_OFFSET);
    }
}

bool AP_EFI_Serial_MS::read_incoming_realtime_data()
{
    // the following is 2 length bytes, the number of bytes we request
    // in the packet we send and 4 bytes of checksum:
    constexpr uint32_t expected_bytes = 2 + (RT_LAST_OFFSET - RT_FIRST_OFFSET) + 4;

    const uint32_t available_bytes = port->available();
    if (available_bytes < expected_bytes) {
        return false;
    }

    // we must have *exactly* what we want in the buffer:
    if (available_bytes > expected_bytes) {
        port->discard_input();
        return false;
    }

    struct PACKED MSData {
        uint16_t message_length;
        uint8_t response_flag;
        uint16_t pw1;
        uint8_t unused1[2];
        uint16_t rpm;
        uint16_t advance;
        uint8_t unused2[1];
        uint8_t engine_bm;
        uint8_t unused3[4];
        uint16_t barometer;
        uint16_t map;
        uint16_t mat;
        uint16_t cht;
        uint16_t tps;
        uint8_t unused4[2];
        uint16_t afr1;
        uint16_t afr2;
        uint8_t unused5[30];
        uint16_t dwell;
        uint8_t load;
        uint8_t unused6[61];
        uint16_t fuel_pressure;
        uint32_t checksum;
    } ms_data;

    static_assert(offsetof(MSData, response_flag) == 2, "response_flag offset");
    static_assert(offsetof(MSData, pw1) == 3, "pw1 offset");
    static_assert(offsetof(MSData, barometer) == 17, "barometer offset");
    static_assert(offsetof(MSData, afr1) == 29, "afr1 offset");
    static_assert(offsetof(MSData, dwell) == 63, "dwell offset");

    assert_storage_size<MSData, expected_bytes> assert_storage_size_MSData;
    (void)assert_storage_size_MSData;

    // read exactly the right number of bytes:
    if (port->read((uint8_t*)&ms_data, available_bytes) != (ssize_t)available_bytes) {
        port->discard_input();
        return false;
    }

    // the message_length field excludes both itself and the checksum:
    constexpr uint32_t expected_message_length = expected_bytes - 4 - 2;
    if (be16toh(ms_data.message_length) != expected_message_length) {
        port->discard_input();
        return false;
    }

    // check the checksum:
    if (be32toh(ms_data.checksum) != calc_CRC32_buffer(&(((uint8_t*)&ms_data)[2]), expected_message_length)) {
        port->discard_input();
        return false;
    }

    // Response Flag (see "response_codes" enum)
    if (ms_data.response_flag != RESPONSE_WRITE_OK) {
        // abort read if we did not receive the correct response code;
        port->discard_input();
        return false;
    }

    // message is good!  Copy data to our internal state:
    internal_state.cylinder_status.injection_time_ms = be32toh(ms_data.pw1) * 0.001f;
    internal_state.engine_speed_rpm = be16toh(ms_data.rpm);
    internal_state.cylinder_status.ignition_timing_deg = be16toh(ms_data.advance) * 0.1f;
    internal_state.atmospheric_pressure_kpa = be16toh(ms_data.barometer) * 0.1f;

    internal_state.intake_manifold_pressure_kpa = be16toh(ms_data.map) * 0.1f;
    internal_state.intake_manifold_temperature = degF_to_Kelvin(be16toh(ms_data.map) * 0.1f);
    internal_state.cylinder_status.cylinder_head_temperature = degF_to_Kelvin(be16toh(ms_data.cht) * 0.1f);
    internal_state.throttle_position_percent = roundf(be16toh(ms_data.tps) * 0.1f);
    internal_state.cylinder_status.lambda_coefficient = be16toh(ms_data.afr1) * 0.1f;
    internal_state.spark_dwell_time_ms = be16toh(ms_data.dwell) * 0.1f;
    internal_state.engine_load_percent = ms_data.load;

    // MS Fuel Pressure is unitless, store as KPA anyway
    internal_state.fuel_pressure = be16toh(ms_data.fuel_pressure);


    // Calculate Fuel Consumption 
    // Duty Cycle (Percent, because that's how HFE gives us the calibration coefficients)
    float duty_cycle = (internal_state.cylinder_status.injection_time_ms * internal_state.engine_speed_rpm)/600.0f;
    uint32_t current_time = AP_HAL::millis();
    // Super Simplified integration method - Error Analysis TBD
    // This calculation gives erroneous results when the engine isn't running
    if (internal_state.engine_speed_rpm > 100) {  // min 100rpm to be running
        internal_state.fuel_consumption_rate_cm3pm = duty_cycle*get_coef1() - get_coef2();
        internal_state.estimated_consumed_fuel_volume_cm3 += internal_state.fuel_consumption_rate_cm3pm * (current_time - internal_state.last_updated_ms)/60000.0f;
    } else {
        internal_state.fuel_consumption_rate_cm3pm = 0;
    }

    internal_state.last_updated_ms = AP_HAL::millis();

    return true;
}

void AP_EFI_Serial_MS::send_request(uint8_t table, uint16_t first_offset, uint16_t last_offset)
{
    uint16_t length = last_offset - first_offset + 1;
    // Fixed message size (0x0007)
    // Command 'r' (0x72)
    // Null CANid (0x00)
    const uint8_t data[9] = {
        0x00,
        0x07,
        0x72,
        0x00,
        (uint8_t)table,
        (uint8_t)(first_offset >> 8),
        (uint8_t)(first_offset),
        (uint8_t)(length >> 8),
        (uint8_t)(length)   
    };
    
    uint32_t crc = 0;
    
    // Write the request and calc CRC
    for (uint8_t i = 0;  i != sizeof(data) ; i++) {
        // Message size is excluded from CRC
        if (i > 1) {
            crc = CRC32_compute_byte(crc, data[i]);
        }
        port->write(data[i]);
    }
    
    // Write the CRC32
    port->write((uint8_t)(crc >> 24));
    port->write((uint8_t)(crc >> 16));
    port->write((uint8_t)(crc >> 8));
    port->write((uint8_t)crc);

}

// CRC32 matching MegaSquirt
uint32_t AP_EFI_Serial_MS::CRC32_compute_byte(uint32_t crc, uint8_t data) const
{
    crc ^= ~0U;
    crc = crc_crc32(crc, &data, 1);
    crc ^= ~0U;
    return crc;
}

uint32_t AP_EFI_Serial_MS::calc_CRC32_buffer(uint8_t *buffer, uint8_t len) const
{
    uint32_t checksum = 0;
    for (uint8_t i=0; i<len; i++) {
        checksum = CRC32_compute_byte(checksum, buffer[i]);
    }
    return checksum;
}

#endif  // AP_EFI_SERIAL_MS_ENABLED
