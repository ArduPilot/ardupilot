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
  Simulator for the BLping rangefinder
*/

#include "SIM_RF_BLping.h"

#include <GCS_MAVLink/GCS.h>
#include <stdio.h>


using namespace SITL;

uint32_t RF_BLping::packet_for_alt(uint16_t alt_cm, uint8_t *buffer, uint8_t buflen)
{
#define BLPING_MSGID_ACK                    1
#define BLPING_MSGID_NACK                   2
#define BLPING_MSGID_SET_PING_INTERVAL      1004
#define BLPING_MSGID_GET_DEVICE_ID          1201
#define BLDPIN_MSGID_DISTANCE_SIMPLE        1211
#define BLPING_MSGID_CONTINUOUS_START       1400

    const uint32_t alt_mm = uint32_t(alt_cm * 10);
    const uint8_t payload[] = {
        uint8_t(alt_mm & 0xff),
        uint8_t((alt_mm >> 8) & 0xff),
        uint8_t((alt_mm >> 16) & 0xff),
        uint8_t((alt_mm >> 24) & 0xff),
    };
    const uint16_t message_id = BLDPIN_MSGID_DISTANCE_SIMPLE;
    const uint8_t src_device_id = 1;
    const uint8_t dst_device_id = 0;
    uint16_t offs = 0;
    buffer[offs++] = 0x42;
    buffer[offs++] = 0x52;
    buffer[offs++] = ARRAY_SIZE(payload) & 0xff;
    buffer[offs++] = ARRAY_SIZE(payload) >> 8;
    buffer[offs++] = message_id & 0xff;
    buffer[offs++] = message_id >> 8;
    buffer[offs++] = src_device_id;
    buffer[offs++] = dst_device_id;
    memcpy(&buffer[offs], payload, ARRAY_SIZE(payload));
    offs += ARRAY_SIZE(payload);
    uint16_t crc = 0;
    for (uint8_t i=0; i<offs; i++) {
        crc += buffer[i];
    }

    buffer[offs++] = crc & 0xff;
    buffer[offs++] = crc >> 8;

    return offs;
}
