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
  simulate GPS sensors

  Usage example:
param set SERIAL5_PROTOCOL 5

     sim_vehicle.py -D --console --map -A --uartB=sim:gps:2
*/

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_SIM_GPS_ENABLED
#define HAL_SIM_GPS_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH))
#endif

#if HAL_SIM_GPS_ENABLED

#ifndef HAL_SIM_GPS_EXTERNAL_FIFO_ENABLED
#define HAL_SIM_GPS_EXTERNAL_FIFO_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#include "SIM_SerialDevice.h"

namespace SITL {

class GPS : public SerialDevice {
public:

    CLASS_NO_COPY(GPS);

    enum Type {
        NONE  =  0,
        UBLOX =  1,
        NMEA  =  5,
        SBP   =  6,
        FILE  =  7,
        NOVA  =  8,
        SBP2  =  9,
    };

    GPS(uint8_t _instance);

    // update state
    void update();

    ssize_t write_to_autopilot(const char *p, size_t size) const override;

private:

    uint8_t instance;

    int ext_fifo_fd;

    uint32_t last_update; // milliseconds

    // for delay simulation:
    uint8_t next_index;
    uint8_t delay;
    struct gps_data {
        double latitude;
        double longitude;
        float altitude;
        double speedN;
        double speedE;
        double speedD;
        double yaw;
        bool have_lock;
    };
#define MAX_GPS_DELAY 100
    gps_data _gps_data[MAX_GPS_DELAY];


#if HAL_SIM_GPS_EXTERNAL_FIFO_ENABLED
    // this will be allocated if needed:
    char *_gps_fifo;
#endif

    bool _gps_has_basestation_position;
    gps_data _gps_basestation_data;

    void send_ubx(uint8_t msgid, uint8_t *buf, uint16_t size);
    void update_ubx(const struct gps_data *d);

    uint8_t nmea_checksum(const char *s);
    void nmea_printf(const char *fmt, ...);
    void update_nmea(const struct gps_data *d);

    void sbp_send_message(uint16_t msg_type, uint16_t sender_id, uint8_t len, uint8_t *payload);

    void update_sbp(const struct gps_data *d);
    void update_sbp2(const struct gps_data *d);

    void update_file();

    void update_nova(const struct gps_data *d);
    void nova_send_message(uint8_t *header, uint8_t headerlength, uint8_t *payload, uint8_t payloadlen);
    uint32_t CRC32Value(uint32_t icrc);
    uint32_t CalculateBlockCRC32(uint32_t length, uint8_t *buffer, uint32_t crc);
};

}

#endif  // HAL_SIM_GPS_ENABLED
