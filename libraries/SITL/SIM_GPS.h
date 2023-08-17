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
#define HAL_SIM_GPS_ENABLED AP_SIM_ENABLED
#endif

#if HAL_SIM_GPS_ENABLED

#ifndef AP_SIM_GPS_FILE_ENABLED
// really need to use AP_FileSystem for this.
#define AP_SIM_GPS_FILE_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX)
#endif

#include "SIM_SerialDevice.h"

namespace SITL {

// for delay simulation:
struct GPS_Data {
    uint32_t timestamp_ms;
    double latitude;
    double longitude;
    float altitude;
    double speedN;
    double speedE;
    double speedD;
    double yaw_deg;
    double roll_deg;
    double pitch_deg;
    bool have_lock;

    // Get heading [rad], where 0 = North in WGS-84 coordinate system
    float heading() const WARN_IF_UNUSED;

    // Get 2D speed [m/s] in WGS-84 coordinate system
    float speed_2d() const WARN_IF_UNUSED;
};


class GPS_Backend {
public:
    CLASS_NO_COPY(GPS_Backend);

    GPS_Backend(class GPS &front, uint8_t _instance);
    virtual ~GPS_Backend() {}

    void update(const GPS_Data &d);

    // 0 baud means "unset" i.e. baud-rate checks should not apply
    virtual uint32_t device_baud() const { return 0; }

    ssize_t write_to_autopilot(const char *p, size_t size) const;
    ssize_t read_from_autopilot(char *buffer, size_t size) const;

protected:

    uint8_t instance;
    GPS &front;

    class SIM *_sitl;

private:

    // read and process config from autopilot (e.g.)
    virtual void update_read(const GPS_Data *d);
    // writing fix information to autopilot (e.g.)
    virtual void update_write(const GPS_Data *d) = 0;

};

class GPS_FILE : public GPS_Backend {
public:
    CLASS_NO_COPY(GPS_FILE);

    using GPS_Backend::GPS_Backend;

    void update_write(const GPS_Data *d) override;
};

class GPS_GSOF : public GPS_Backend {
public:
    CLASS_NO_COPY(GPS_GSOF);

    using GPS_Backend::GPS_Backend;

    void update_write(const GPS_Data *d) override;

private:
    void send_gsof(const uint8_t *buf, const uint16_t size);

    uint64_t pack_double_into_gsof_packet(const double& src) WARN_IF_UNUSED;
    uint32_t pack_float_into_gsof_packet(const float& src) WARN_IF_UNUSED;
};

class GPS_NMEA : public GPS_Backend {
public:
    CLASS_NO_COPY(GPS_NMEA);

    using GPS_Backend::GPS_Backend;

    void update_write(const GPS_Data *d) override;

private:

    uint8_t nmea_checksum(const char *s);
    void nmea_printf(const char *fmt, ...);
    void update_nmea(const GPS_Data *d);

};

class GPS_NOVA : public GPS_Backend {
public:
    CLASS_NO_COPY(GPS_NOVA);

    using GPS_Backend::GPS_Backend;

    void update_write(const GPS_Data *d) override;

    uint32_t device_baud() const override { return 19200; }

private:

    void nova_send_message(uint8_t *header, uint8_t headerlength, uint8_t *payload, uint8_t payloadlen);
    uint32_t CRC32Value(uint32_t icrc);
    uint32_t CalculateBlockCRC32(uint32_t length, uint8_t *buffer, uint32_t crc);
};

class GPS_MSP : public GPS_Backend {
public:
    CLASS_NO_COPY(GPS_MSP);

    using GPS_Backend::GPS_Backend;

    void update_write(const GPS_Data *d) override;
};

class GPS_SBP_Common : public GPS_Backend {
public:
    CLASS_NO_COPY(GPS_SBP_Common);

    using GPS_Backend::GPS_Backend;

protected:

    void sbp_send_message(uint16_t msg_type, uint16_t sender_id, uint8_t len, uint8_t *payload);

};

class GPS_SBP : public GPS_SBP_Common {
public:
    CLASS_NO_COPY(GPS_SBP);

    using GPS_SBP_Common::GPS_SBP_Common;

    void update_write(const GPS_Data *d) override;

};

class GPS_SBP2 : public GPS_SBP_Common {
public:
    CLASS_NO_COPY(GPS_SBP2);

    using GPS_SBP_Common::GPS_SBP_Common;

    void update_write(const GPS_Data *d) override;

};

class GPS_UBlox : public GPS_Backend {
public:
    CLASS_NO_COPY(GPS_UBlox);

    using GPS_Backend::GPS_Backend;

    void update_write(const GPS_Data *d) override;

private:
    void send_ubx(uint8_t msgid, uint8_t *buf, uint16_t size);
};

class GPS : public SerialDevice {
public:

    CLASS_NO_COPY(GPS);

    enum Type {
        NONE  =  0,
        UBLOX =  1,
        NMEA  =  5,
        SBP   =  6,
#if AP_SIM_GPS_FILE_ENABLED
        FILE  =  7,
#endif
        NOVA  =  8,
        SBP2  =  9,
        GSOF  = 11, // matches GPS_TYPE
        MSP   = 19,
    };

    GPS(uint8_t _instance);

    // update state
    void update();

    ssize_t write_to_autopilot(const char *p, size_t size) const override;

    uint32_t device_baud() const override;  // 0 meaning unset

private:

    uint8_t instance;

    int ext_fifo_fd;

    uint32_t last_update; // milliseconds

    // last 20 samples, allowing for up to 20 samples of delay
    GPS_Data _gps_history[20];

    bool _gps_has_basestation_position;
    GPS_Data _gps_basestation_data;

    // get delayed data
    GPS_Data interpolate_data(const GPS_Data &d, uint32_t delay_ms);

    uint8_t allocated_type;
    GPS_Backend *backend;
    void check_backend_allocation();
};

}

#endif  // HAL_SIM_GPS_ENABLED
