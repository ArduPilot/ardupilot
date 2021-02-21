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
  Simulator for the LeddarOne rangefinder
*/

#include "SIM_RF_LeddarOne.h"

#include <GCS_MAVLink/GCS.h>
#include <stdio.h>

using namespace SITL;

uint32_t RF_LeddarOne::packet_for_alt(uint16_t alt_cm, uint8_t *buffer, uint8_t buflen)
{
    const uint8_t response_size = 25;
    const uint16_t internal_temperature = 1245;
    const uint16_t num_detections = 1;
    const uint16_t first_distance = alt_cm * 10;
    const uint16_t first_amplitude = 37;
    const uint16_t second_distance = alt_cm * 10;
    const uint16_t second_amplitude = 37;
    const uint16_t third_distance = alt_cm * 10;
    const uint16_t third_amplitude = 37;
    const uint32_t now = AP_HAL::millis();

    buffer[0] = 0x01;
    buffer[1] = 0x04; // magic function number! LEDDARONE_MODOBUS_FUNCTION_CODE
    buffer[2] = response_size;
    buffer[3] = (now >> 16) & 0xff;
    buffer[4] = (now >> 24) & 0xff;
    buffer[5] = (now >> 0) & 0xff;
    buffer[6] = (now >> 8) & 0xff;
    buffer[7] = internal_temperature & 0xff;
    buffer[8] = internal_temperature >> 8;
    buffer[9] = num_detections >> 8;
    buffer[10] = num_detections & 0xff;
    buffer[11] = first_distance >> 8;
    buffer[12] = first_distance & 0xff;
    buffer[13] = first_amplitude >> 8;
    buffer[14] = first_amplitude & 0xff;
    buffer[15] = second_distance >> 8;
    buffer[16] = second_distance & 0xff;
    buffer[17] = second_amplitude >> 8;
    buffer[18] = second_amplitude & 0xff;
    buffer[19] = third_distance >> 8;
    buffer[20] = third_distance & 0xff;
    buffer[21] = third_amplitude >> 8;
    buffer[22] = third_amplitude & 0xff;

    const uint16_t crc = calc_crc_modbus(buffer, 23);
    buffer[23] = crc & 0xff;
    buffer[24] = crc >> 8;

    return response_size;
}
