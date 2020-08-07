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
#include "AP_EFI_Serial_EcoEFI.h"

#if EFI_ENABLED
#include <AP_SerialManager/AP_SerialManager.h>

#include <AP_HAL/utility/sparse-endian.h>

AP_EFI_Serial_EcoEFI::AP_EFI_Serial_EcoEFI(AP_EFI &_frontend):
    AP_EFI_Backend(_frontend)
{
    uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_EcoEFI, 0);
    if (uart == nullptr) {
        return;
    }
    uart->begin(115200);
}


extern const AP_HAL::HAL &hal;


// find an EcoEFI message in the buffer, starting at initial_offset.
// If found, that message (or partial message) will be moved to the
// start of the buffer.
void AP_EFI_Serial_EcoEFI::move_header_in_buffer(uint8_t initial_offset)
{
    uint8_t header_offset;
    for (header_offset=initial_offset; header_offset<body_length; header_offset++) {
        if (u.parse_buffer[header_offset] == HEADER_MAGIC1) {
            break;
        }
    }
    if (header_offset != 0) {
        // header was found, but not at index 0; move it back to start of array
        memmove(u.parse_buffer, &u.parse_buffer[header_offset], body_length - header_offset);
        body_length -= header_offset;
    }
}

void AP_EFI_Serial_EcoEFI::update()
{
    if (uart == nullptr) {
        return;
    }

    for (uint8_t msg=0; msg<5; msg++) {  // process a maximum of 5 messages
        uint32_t nbytes = uart->read(&u.parse_buffer[body_length],
                                     ARRAY_SIZE(u.parse_buffer)-body_length);
        if (nbytes == 0) {
            return;
        }
        body_length += nbytes;

        move_header_in_buffer(0);

        // header byte 1 is correct.
        if (body_length < ARRAY_SIZE(u.parse_buffer)) {
            // need a full buffer to have a valid message...
            return;
        }

        if (u.packet.headermagic2 != HEADER_MAGIC2) {
            move_header_in_buffer(2);
            return;
        }

        if (u.packet.headermagic3 != HEADER_MAGIC3) {
            move_header_in_buffer(3); // we know MAGIC1 != MAGIC2
            return;
        }

        if (u.packet.data_field_length != DATA_FIELD_LENGTH) {
            move_header_in_buffer(3);
            return;
        }

        if (u.packet.service_id != SERVICE_ID) {
            move_header_in_buffer(3);
            return;
        }

        // calculate checksum....
        uint8_t checksum = 0;
        for (uint8_t i=0; i<ARRAY_SIZE(u.parse_buffer)-1; i++) {
            checksum += u.parse_buffer[i];
        }
        if (checksum != u.packet.checksum) {
            move_header_in_buffer(3);
            return;
        }

        internal_state.engine_speed_rpm = be16toh(u.packet.RPM) * 4;
        internal_state.intake_manifold_pressure_kpa = be16toh(u.packet.MAP) * 0.0039;
        internal_state.throttle_position_percent = be16toh(u.packet.TPS) * 0.0015;
        internal_state.coolant_temperature = be16toh(u.packet.ECT) * 1.25 - 40 + 273;
        internal_state.intake_manifold_temperature = be16toh(u.packet.IAT) * 1.25 - 40 + 273;
//        internal_state.oxygen_sensor_volts = be16toh(u.packet.O2S) * 0.0012;
//        internal_state.spark_current_amps = be16toh(u.packet.SPARK) * 0.75;  // just a guess at units....
//        internal_state.something_something_ms = be16toh(u.packet.FUELPW1) * 0.001;
//        internal_state.something_something_else_ms = be16toh(u.packet.FUELPW2) * 0.001;
//        internal_state.something_something_volts = be16toh(u.packet.ubAdc) * 0.00625;
//        internal_state.something_something_percent = u.packet.FuelLvl * 0.4;
        internal_state.atmospheric_pressure_kpa = be16toh(u.packet.BARO) * 0.0039;
//        internal_state.something_something_grams_per_minute = u.packet.Field_Consumption * 0.0116;

        copy_to_frontend();

        body_length = 0;
    }
}

#endif // EFI_ENABLED
