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
#include "AP_EFI_Serial_MS.h"

#if EFI_ENABLED
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL &hal;

AP_EFI_Serial_MS::AP_EFI_Serial_MS(AP_EFI &_frontend):
    AP_EFI_Backend(_frontend)
{
    internal_state.estimated_consumed_fuel_volume_cm3 = 0; // Just to be sure
    port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_EFI_MS, 0);
}


void AP_EFI_Serial_MS::update()
{
    if (!port) {
        return;
    }

    uint32_t now = AP_HAL::millis();

    const uint32_t expected_bytes = 2 + (RT_LAST_OFFSET - RT_FIRST_OFFSET) + 4;
    if (port->available() >= expected_bytes && read_incoming_realtime_data()) {
        last_response_ms = now;
        copy_to_frontend();
    }

    if (port->available() == 0 || now - last_response_ms > 200) {
        // clear the input buffer
        uint32_t buffered_data_size = port->available();
        for (uint32_t i = 0; i < buffered_data_size; i++) {
            port->read();
        }
        // Request an update from the realtime table (7).
        // The data we need start at offset 6 and ends at 129
        send_request(7, RT_FIRST_OFFSET, RT_LAST_OFFSET);
    }
}

bool AP_EFI_Serial_MS::read_incoming_realtime_data() 
{
    // Data is parsed directly from the buffer, otherwise we would need to allocate
    // several hundred bytes for the entire realtime data table or request every
    // value individiually
    uint16_t message_length = 0;

    // reset checksum before reading new data
    checksum = 0;
    
    // Message length field begins the message (16 bits, excluded from CRC calculation)
    // Message length value excludes the message length and CRC bytes 
    message_length = port->read() << 8;
    message_length += port->read();

    if (message_length >= 256) {
        // don't process invalid messages
        // hal.console->printf("message_length: %u\n", message_length);
        return false;
    }

    // Response Flag (see "response_codes" enum)
    response_flag = read_byte_CRC32();
    if (response_flag != RESPONSE_WRITE_OK) {
        // abort read if we did not receive the correct response code;
        return false;
    }
    
    // Iterate over the payload bytes 
    for ( uint8_t offset=RT_FIRST_OFFSET; offset < (RT_FIRST_OFFSET + message_length - 1); offset++) {
        uint8_t data = read_byte_CRC32();
        float temp_float;
        switch (offset) {
            case PW1_MSB:
                internal_state.cylinder_status[0].injection_time_ms = (float)((data << 8) + read_byte_CRC32())/1000.0f;
                offset++;  // increment the counter because we read a byte in the previous line
                break;
            case RPM_MSB:
                // Read 16 bit RPM
                internal_state.engine_speed_rpm = (data << 8) + read_byte_CRC32();
                offset++;
                break;
            case ADVANCE_MSB:
                internal_state.cylinder_status[0].ignition_timing_deg = (float)((data << 8) + read_byte_CRC32())/10.0f;
                offset++;
                break;
            case ENGINE_BM:
                break;
            case BAROMETER_MSB:
                internal_state.atmospheric_pressure_kpa = (float)((data << 8) + read_byte_CRC32())/10.0f;
                offset++;
                break;
            case MAP_MSB:
                internal_state.intake_manifold_pressure_kpa = (float)((data << 8) + read_byte_CRC32())/10.0f;
                offset++;
                break;
            case MAT_MSB:
                temp_float = (float)((data << 8) + read_byte_CRC32())/10.0f;
                offset++;
                internal_state.intake_manifold_temperature = f_to_k(temp_float);
                break;
            case CHT_MSB:
                temp_float = (float)((data << 8) + read_byte_CRC32())/10.0f;
                offset++;
                internal_state.cylinder_status[0].cylinder_head_temperature = f_to_k(temp_float);
                break;
            case TPS_MSB:
                temp_float = (float)((data << 8) + read_byte_CRC32())/10.0f;
                offset++;
                internal_state.throttle_position_percent = roundf(temp_float);
                break;
            case AFR1_MSB:
                temp_float = (float)((data << 8) + read_byte_CRC32())/10.0f;
                offset++;
                internal_state.cylinder_status[0].lambda_coefficient = temp_float;
                break;
            case DWELL_MSB:
                temp_float = (float)((data << 8) + read_byte_CRC32())/10.0f;
                internal_state.spark_dwell_time_ms = temp_float;
                offset++;
                break;
            case LOAD:
                internal_state.engine_load_percent = data;
                break;
            case FUEL_PRESSURE_MSB:
                // MS Fuel Pressure is unitless, store as KPA anyway
                temp_float = (float)((data << 8) + read_byte_CRC32());
                internal_state.fuel_pressure = temp_float;
                offset++;
                break;   
                
        }
    }
    
    // Read the four CRC bytes
    uint32_t received_CRC;
    received_CRC = port->read() << 24;
    received_CRC += port->read() << 16;
    received_CRC += port->read() << 8;
    received_CRC += port->read();
                        
    if (received_CRC != checksum) {
        // hal.console->printf("EFI CRC: 0x%08x 0x%08x\n", received_CRC, checksum);
        return false;
    }

    // Calculate Fuel Consumption 
    // Duty Cycle (Percent, because that's how HFE gives us the calibration coefficients)
    float duty_cycle = (internal_state.cylinder_status[0].injection_time_ms * internal_state.engine_speed_rpm)/600.0f;
    uint32_t current_time = AP_HAL::millis();
    // Super Simplified integration method - Error Analysis TBD
    // This calcualtion gives erroneous results when the engine isn't running
    if (internal_state.engine_speed_rpm > RPM_THRESHOLD) {
        internal_state.fuel_consumption_rate_cm3pm = duty_cycle*get_coef1() - get_coef2();
        internal_state.estimated_consumed_fuel_volume_cm3 += internal_state.fuel_consumption_rate_cm3pm * (current_time - internal_state.last_updated_ms)/60000.0f;
    } else {
        internal_state.fuel_consumption_rate_cm3pm = 0;
    }
    internal_state.last_updated_ms = current_time;
    
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

uint8_t AP_EFI_Serial_MS::read_byte_CRC32()
{   
    // Read a byte and update the CRC 
    uint8_t data = port->read();
    checksum = CRC32_compute_byte(checksum, data);
    return data;
}

// CRC32 matching MegaSquirt
uint32_t AP_EFI_Serial_MS::CRC32_compute_byte(uint32_t crc, uint8_t data)
{
    crc ^= ~0U;
    crc = crc_crc32(crc, &data, 1);
    crc ^= ~0U;
    return crc;
}

#endif // EFI_ENABLED
