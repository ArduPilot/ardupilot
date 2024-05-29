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
  GPS detection state structures. These need to be in a separate
  header to prevent a circular dependency between AP_GPS and the
  backend drivers.

  These structures are allocated as a single block in AP_GPS during
  driver detection, then freed once the detection is finished. Each
  GPS driver needs to implement a static _detect() function which uses
  this state information to detect if the attached GPS is of the
  specific type that it handles.
 */

#include "AP_GPS_config.h"

#if AP_GPS_NMEA_ENABLED
struct NMEA_detect_state {
    uint8_t step;
    uint8_t ck;
};
#endif

#if AP_GPS_SIRF_ENABLED
struct SIRF_detect_state {
    uint16_t checksum;
    uint8_t step, payload_length, payload_counter;
};
#endif

#if AP_GPS_UBLOX_ENABLED
struct UBLOX_detect_state {
    uint8_t payload_length, payload_counter;
    uint8_t step;
    uint8_t ck_a, ck_b;
};
#endif

#if AP_GPS_ERB_ENABLED
struct ERB_detect_state {
    uint8_t payload_length, payload_counter;
    uint8_t step;
    uint8_t ck_a, ck_b;
};
#endif

#if AP_GPS_SBP_ENABLED
struct SBP_detect_state {
    enum {
        WAITING = 0,
        GET_TYPE = 1,
        GET_SENDER = 2,
        GET_LEN = 3,
        GET_MSG = 4,
        GET_CRC = 5
    } state:8;
    uint16_t msg_type;
    uint8_t n_read;
    uint8_t msg_len;
    uint16_t crc_so_far;
    uint16_t crc;
    uint8_t heartbeat_buff[4];
};
#endif

#if AP_GPS_SBP2_ENABLED
struct SBP2_detect_state {
    enum {
        WAITING = 0,
        GET_TYPE = 1,
        GET_SENDER = 2,
        GET_LEN = 3,
        GET_MSG = 4,
        GET_CRC = 5
    } state:8;
    uint16_t msg_type;
    uint8_t n_read;
    uint8_t msg_len;
    uint16_t crc_so_far;
    uint16_t crc;
    uint8_t heartbeat_buff[4];
};
#endif
