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

#include <AP_HAL/AP_HAL.h>

#ifndef HAL_CRSF_TELEM_ENABLED
#define HAL_CRSF_TELEM_ENABLED !HAL_MINIMIZE_FEATURES
#endif

#if HAL_CRSF_TELEM_ENABLED

#include <AP_Notify/AP_Notify.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_RCProtocol/AP_RCProtocol_CRSF.h>
#include "AP_RCTelemetry.h"
#include <AP_HAL/utility/sparse-endian.h>

class AP_CRSF_Telem : public AP_RCTelemetry {
public:
    AP_CRSF_Telem();
    ~AP_CRSF_Telem() override;

    /* Do not allow copies */
    AP_CRSF_Telem(const AP_CRSF_Telem &other) = delete;
    AP_CRSF_Telem &operator=(const AP_CRSF_Telem&) = delete;

    // init - perform required initialisation
    virtual bool init() override;

    static AP_CRSF_Telem *get_singleton(void);

    // Broadcast frame definitions courtesy of TBS
    struct GPSFrame {   // curious fact, calling this GPS makes sizeof(GPS) return 1!
        int32_t latitude; // ( degree / 10`000`000 )
        int32_t longitude; // (degree / 10`000`000 )
        uint16_t groundspeed; // ( km/h / 100 )
        uint16_t gps_heading; // ( degree / 100 )
        uint16_t altitude; // ( meter - 1000m offset )
        uint8_t satellites; // in use ( counter )
    } PACKED;

    struct HeartbeatFrame {
        uint8_t origin; // Device addres
    };

    struct BatteryFrame {
        uint16_t voltage; // ( mV * 100 )
        uint16_t current; // ( mA * 100 )
        uint8_t capacity[3]; // ( mAh )
        uint8_t remaining; // ( percent )
    } PACKED;

    struct VTXFrame {
#if __BYTE_ORDER != __LITTLE_ENDIAN
#error "Only supported on little-endian architectures"
#endif
        uint8_t origin; // address
        // status
        uint8_t is_in_pitmode : 1;
        uint8_t is_in_user_frequency_mode : 1;
        uint8_t unused : 2;
        uint8_t is_vtx_available : 1;
        uint8_t smart_audio_ver : 3;    // SmartAudio_V1 = 0, SmartAudio_V2 = 1
        // band / channel
        uint8_t channel : 3;            // 1x-8x
        uint8_t band : 5;               // A, B, E, AirWave, Race
        uint16_t user_frequency;
        uint8_t power : 4;              // 25mW = 0, 200mW = 1, 500mW = 2, 800mW = 3
        uint8_t pitmode : 4;            // off = 0, In_Band = 1, Out_Band = 2;
    } PACKED;

    struct VTXTelemetryFrame {
        uint8_t origin; // address
        uint8_t power;              // power in dBm
        uint16_t frequency;         // frequency in Mhz
        uint8_t pitmode;            // disable 0, enable 1
    } PACKED;

    struct AttitudeFrame {
        int16_t pitch_angle; // ( rad * 10000 )
        int16_t roll_angle; // ( rad * 10000 )
        int16_t yaw_angle; // ( rad * 10000 )
    } PACKED;

    struct FlightModeFrame {
        char flight_mode[16]; // ( Null-terminated string )
    } PACKED;

    // CRSF_FRAMETYPE_COMMAND
    struct CommandFrame {
        uint8_t destination;
        uint8_t origin;
        uint8_t command_id;
        uint8_t payload[9]; // 8 maximum for LED command + crc8
    } PACKED;

    // CRSF_FRAMETYPE_PARAM_DEVICE_PING
    struct ParameterPingFrame {
        uint8_t destination;
        uint8_t origin;
    } PACKED;

    union BroadcastFrame {
        GPSFrame gps;
        HeartbeatFrame heartbeat;
        BatteryFrame battery;
        VTXFrame vtx;
        AttitudeFrame attitude;
        FlightModeFrame flightmode;
    } PACKED;

    union ExtendedFrame {
        CommandFrame command;
        ParameterPingFrame ping;
    } PACKED;

    union TelemetryPayload {
        BroadcastFrame bcast;
        ExtendedFrame ext;
    } PACKED;

    // Process a frame from the CRSF protocol decoder
    static bool process_frame(AP_RCProtocol_CRSF::FrameType frame_type, void* data);
    // process any changed settings and schedule for transmission
    void update();
    // get next telemetry data for external consumers of SPort data
    static bool get_telem_data(AP_RCProtocol_CRSF::Frame* frame);

private:

    enum SensorType {
        HEARTBEAT,
        ATTITUDE,
        PARAMETERS,
        BATTERY,
        GPS,
        FLIGHT_MODE,
        NUM_SENSORS
    };

    // passthrough WFQ scheduler
    bool is_packet_ready(uint8_t idx, bool queue_empty) override;
    void process_packet(uint8_t idx) override;
    void adjust_packet_weight(bool queue_empty) override;

    void calc_parameter_ping();
    void calc_heartbeat();
    void calc_battery();
    void calc_gps();
    void calc_attitude();
    void calc_flight_mode();
    void update_params();

    void process_vtx_frame(VTXFrame* vtx);
    void process_vtx_telem_frame(VTXTelemetryFrame* vtx);

     // setup ready for passthrough operation
    void setup_wfq_scheduler(void) override;

     // get next telemetry data for external consumers
    bool _get_telem_data(AP_RCProtocol_CRSF::Frame* data);
    bool _process_frame(AP_RCProtocol_CRSF::FrameType frame_type, void* data);

    TelemetryPayload _telem;
    uint8_t _telem_size;
    uint8_t _telem_type;

    bool _telem_pending;
    bool _enable_telemetry;

    // vtx state
    bool _vtx_freq_update;  // update using the frequency method or not
    bool _vtx_dbm_update; // update using the dbm method or not
    bool _vtx_freq_change_pending; // a vtx command has been issued but not confirmed by a vtx broadcast frame
    bool _vtx_power_change_pending;
    bool _vtx_options_change_pending;

    static AP_CRSF_Telem *singleton;
};

namespace AP {
    AP_CRSF_Telem *crsf_telem();
};

#endif
