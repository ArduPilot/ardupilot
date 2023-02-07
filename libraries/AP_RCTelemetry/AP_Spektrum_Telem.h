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
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_SPEKTRUM_TELEM_ENABLED
#define HAL_SPEKTRUM_TELEM_ENABLED !HAL_MINIMIZE_FEATURES
#endif

#if HAL_SPEKTRUM_TELEM_ENABLED

#include <AP_HAL/utility/RingBuffer.h>
#include "AP_RCTelemetry.h"

#define UINT8 uint8_t
#define UINT16 uint16_t
#define UINT32 uint32_t
#define UINT64 uint64_t
#define INT8 int8_t
#define INT16 int16_t
#define INT32 int32_t
extern "C" {
#include "spektrumTelemetrySensors.h"
}
#undef UINT8
#undef UINT16
#undef UINT32
#undef UINT64
#undef INT8
#undef INT16
#undef INT32

class AP_Spektrum_Telem : public AP_RCTelemetry {
public:
    AP_Spektrum_Telem();
    ~AP_Spektrum_Telem() override;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Spektrum_Telem);

    // init - perform required initialisation
    virtual bool init() override;

    static AP_Spektrum_Telem *get_singleton(void) {
        return singleton;
    }

    // get next telemetry data for external consumers of SPort data
    static bool get_telem_data(uint8_t* data);

private:

    enum SensorType {
        QOS,
        RPM,
        TEXT,
        ATTITUDE,
        GPS_LOC,
        ESC,
        ALTITUDE,
        AIRSPEED,
        GPS_STATUS,
        VOLTAGE,
        AMPS,
        MAH,
        TEMP,
        NUM_SENSORS
    };

    struct MessageChunk
    {
        uint8_t chunk[13]; // a "chunk" (13 characters/bytes) at a time of the queued message to be sent
        uint8_t linenumber;
        uint8_t char_index; // index of which character to get in the message
        uint8_t repeats;
    } _msg_chunk;

    float _max_speed = 0.0f;
    float _max_alt = 0.0f;

    // passthrough WFQ scheduler
    // Text Generator
    bool get_next_msg_chunk(void) override;
    bool repeat_msg_chunk(void);
    void send_msg_chunk(const MessageChunk& message);
    bool is_packet_ready(uint8_t idx, bool queue_empty) override;
    void process_packet(uint8_t idx) override;
    void adjust_packet_weight(bool queue_empty) override;
    // RxV + flight log data
    void calc_qos();
    // High-Voltage sensor
    void calc_batt_volts(uint8_t instance);
    // Temperature Sensor
    void calc_temperature(uint8_t instance);
    // Amps
    void calc_batt_amps(uint8_t instance);
    // Flight Battery Capacity (Dual)
    void calc_batt_mah();
    // Altitude (Eagle Tree Sensor)
    void calc_altitude();
    // Air Speed (Eagle Tree Sensor)
    void calc_airspeed();
    // Attitude and Magnetic Compass
    void calc_attandmag();
    // GPS Location Data (Eagle Tree)
    void calc_gps_location();
    // GPS Status (Eagle Tree)
    void calc_gps_status();
    // Electronic Speed Control
    void calc_esc();
    // RPM sensor
    void calc_rpm();

     // setup ready for passthrough operation
    void setup_wfq_scheduler(void) override;

     // get next telemetry data for external consumers of SPort data (internal function)
    bool _get_telem_data(uint8_t* data);

    // all Spektrum telemtry packets are big-endian!
    PACKED UN_TELEMETRY _telem;
    bool _telem_pending;

    static AP_Spektrum_Telem *singleton;
};

namespace AP {
    AP_Spektrum_Telem *spektrum_telem();
};

#endif
