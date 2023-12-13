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

#include "AP_RCTelemetry_config.h"

#if AP_GHST_TELEM_ENABLED

#include <AP_RCProtocol/AP_RCProtocol_GHST.h>
#include "AP_RCTelemetry.h"
#include <AP_HAL/utility/sparse-endian.h>

class AP_GHST_Telem : public AP_RCTelemetry {
public:
    AP_GHST_Telem();
    ~AP_GHST_Telem() override;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_GHST_Telem);

    // init - perform required initialisation
    virtual bool init() override;

    static AP_GHST_Telem *get_singleton(void);

    // Broadcast frame definitions courtesy of TBS
    struct PACKED GPSFrame {
        uint32_t latitude; // ( degree * 1e7 )
        uint32_t longitude; // (degree * 1e7 )
        int16_t altitude; // ( meter )
    };

    struct PACKED GPSSecondaryFrame {
        uint16_t groundspeed; // ( cm/s )
        uint16_t gps_heading; // ( degree * 10 )
        uint8_t satellites; // in use ( counter )
        uint16_t home_dist; // ( m / 10 )
        uint16_t home_heading; // ( degree * 10 )
        uint8_t flags; // GPS_FLAGS_FIX 0x01, GPS_FLAGS_FIX_HOME 0x2
    };

    struct PACKED BatteryFrame {
        uint16_t voltage; // ( mV / 10 )
        uint16_t current; // ( mA / 10 )
        uint16_t consumed; // ( mAh / 10 )
        uint8_t rx_voltage; // ( mV / 100 )
        uint8_t spare[3];
    };

    struct PACKED VTXFrame {
#if __BYTE_ORDER != __LITTLE_ENDIAN
#error "Only supported on little-endian architectures"
#endif
        uint8_t flags;
        uint16_t frequency;         // frequency in Mhz
        uint16_t power;              // power in mw, 0 == off
        uint8_t band : 4;               // A, B, E, AirWave, Race
        uint8_t channel : 4;            // 1x-8x
        uint8_t spare[3];
    };

    struct PACKED SensorFrame {
        uint16_t compass_heading; // ( deg * 10 )
        int16_t baro_alt; // ( m )
        int16_t vario; // ( m/s * 100 )
        uint8_t spare[3];
        uint8_t flags; // MISC_FLAGS_MAGHEAD 0x01, MISC_FLAGS_BAROALT 0x02, MISC_FLAGS_VARIO 0x04
    };

    union PACKED TelemetryPayload {
        GPSFrame gps;
        GPSSecondaryFrame gps2;
        BatteryFrame battery;
        VTXFrame vtx;
        SensorFrame sensor;
    };

    // get the protocol string
    const char* get_protocol_string() const { return AP::ghost()->get_protocol_string(); }

    // Process a frame from the CRSF protocol decoder
    static bool process_frame(AP_RCProtocol_GHST::FrameType frame_type, void* data);
    // process any changed settings and schedule for transmission
    void update();
    // get next telemetry data for external consumers of SPort data
    static bool get_telem_data(AP_RCProtocol_GHST::Frame* frame, bool is_tx_active);

private:

    enum SensorType {
        ATTITUDE,
        VTX_PARAMETERS,
        BATTERY,
        GPS,
        GPS2,
        NUM_SENSORS
    };

    // passthrough WFQ scheduler
    bool is_packet_ready(uint8_t idx, bool queue_empty) override;
    void process_packet(uint8_t idx) override;
    void setup_custom_telemetry();
    void update_custom_telemetry_rates(const AP_RCProtocol_GHST::RFMode rf_mode);

    void calc_battery();
    void calc_gps();
    void calc_gps2();
    void calc_attitude();
    void process_pending_requests();
    bool process_rf_mode_changes();
    AP_RCProtocol_GHST::RFMode get_rf_mode() const;
    uint16_t get_telemetry_rate() const;
    bool is_high_speed_telemetry(const AP_RCProtocol_GHST::RFMode rf_mode) const;

    // setup ready for passthrough operation
    void setup_wfq_scheduler(void) override;

    // get next telemetry data for external consumers
    bool _get_telem_data(AP_RCProtocol_GHST::Frame* data, bool is_tx_active);
    bool _process_frame(AP_RCProtocol_GHST::FrameType frame_type, void* data);

    TelemetryPayload _telem;
    uint8_t _telem_size;
    uint8_t _telem_type;

    AP_RCProtocol_GHST::RFMode _rf_mode;
    bool _enable_telemetry;

    // reporting telemetry rate
    uint32_t _telem_last_report_ms;
    uint16_t _telem_last_avg_rate;
    // do we need to report the initial state
    bool _telem_bootstrap_msg_pending;

    bool _telem_is_high_speed;
    bool _telem_pending;
    // used to limit telemetry when in a failsafe condition
    bool _is_tx_active;

    struct {
        uint8_t destination = 0;
        uint8_t frame_type;
    } _pending_request;

    bool _noted_lq_as_rssi_active;

    static AP_GHST_Telem *singleton;
};

namespace AP {
    AP_GHST_Telem *ghost_telem();
};

#endif // AP_GHST_TELEM_ENABLED
