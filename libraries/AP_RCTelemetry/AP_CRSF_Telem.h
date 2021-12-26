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
#include <AP_OSD/AP_OSD.h>

#ifndef HAL_CRSF_TELEM_ENABLED
#define HAL_CRSF_TELEM_ENABLED !HAL_MINIMIZE_FEATURES
#endif

#ifndef HAL_CRSF_TELEM_TEXT_SELECTION_ENABLED
#define HAL_CRSF_TELEM_TEXT_SELECTION_ENABLED OSD_ENABLED && OSD_PARAM_ENABLED && HAL_CRSF_TELEM_ENABLED && BOARD_FLASH_SIZE > 1024
#endif

#if HAL_CRSF_TELEM_ENABLED

#include <AP_RCProtocol/AP_RCProtocol_CRSF.h>
#include <AP_VideoTX/AP_VideoTX.h>
#include "AP_RCTelemetry.h"
#include <AP_HAL/utility/sparse-endian.h>

class AP_OSD_ParamSetting;

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
    void queue_message(MAV_SEVERITY severity, const char *text) override;

    static const uint8_t PASSTHROUGH_STATUS_TEXT_FRAME_MAX_SIZE = 50U;
    static const uint8_t PASSTHROUGH_MULTI_PACKET_FRAME_MAX_SIZE = 9U;
    static const uint8_t CRSF_RX_DEVICE_PING_MAX_RETRY = 50U;

    // Broadcast frame definitions courtesy of TBS
    struct PACKED GPSFrame {   // curious fact, calling this GPS makes sizeof(GPS) return 1!
        int32_t latitude; // ( degree / 10`000`000 )
        int32_t longitude; // (degree / 10`000`000 )
        uint16_t groundspeed; // ( km/h / 100 )
        uint16_t gps_heading; // ( degree / 100 )
        uint16_t altitude; // ( meter - 1000m offset )
        uint8_t satellites; // in use ( counter )
    };

    struct HeartbeatFrame {
        uint8_t origin; // Device address
    };

    struct PACKED BatteryFrame {
        uint16_t voltage; // ( mV * 100 )
        uint16_t current; // ( mA * 100 )
        uint8_t capacity[3]; // ( mAh )
        uint8_t remaining; // ( percent )
    };

    struct PACKED VTXTelemetryFrame {
#if __BYTE_ORDER != __LITTLE_ENDIAN
#error "Only supported on little-endian architectures"
#endif
        uint8_t origin; // address
        uint8_t power;              // power in dBm
        uint16_t frequency;         // frequency in Mhz
        uint8_t pitmode : 1;        // disable 0, enable 1
        uint8_t unused : 7;
    };

    struct PACKED AttitudeFrame {
        int16_t pitch_angle; // ( rad * 10000 )
        int16_t roll_angle; // ( rad * 10000 )
        int16_t yaw_angle; // ( rad * 10000 )
    };

    struct PACKED FlightModeFrame {
        char flight_mode[16]; // ( Null-terminated string )
    };

    // CRSF_FRAMETYPE_COMMAND
    struct PACKED CommandFrame {
        uint8_t destination;
        uint8_t origin;
        uint8_t command_id;
        uint8_t payload[9]; // 8 maximum for LED command + crc8
    };

    // CRSF_FRAMETYPE_PARAM_DEVICE_PING
    struct PACKED ParameterPingFrame {
        uint8_t destination;
        uint8_t origin;
    };

    // CRSF_FRAMETYPE_PARAM_DEVICE_INFO
    struct PACKED ParameterDeviceInfoFrame {
        uint8_t destination;
        uint8_t origin;
        uint8_t payload[58];   // largest possible frame is 60
    };

    enum ParameterType : uint8_t
    {
        UINT8 = 0,
        INT8 = 1,
        UINT16 = 2,
        INT16 = 3,
        FLOAT = 8,
        TEXT_SELECTION = 9,
        STRING = 10,
        FOLDER = 11,
        INFO = 12,
        COMMAND = 13,
        OUT_OF_RANGE = 127
    };

    // CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY
    struct PACKED ParameterSettingsEntryHeader {
        uint8_t destination;
        uint8_t origin;
        uint8_t param_num;
        uint8_t chunks_left;
    };

    // CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY
    struct PACKED ParameterSettingsEntry {
        ParameterSettingsEntryHeader header;
        uint8_t payload[56];   // largest possible frame is 60
    };

    // CRSF_FRAMETYPE_PARAMETER_READ
    struct PACKED ParameterSettingsReadFrame {
        uint8_t destination;
        uint8_t origin;
        uint8_t param_num;
        uint8_t param_chunk;
    } _param_request;

    // CRSF_FRAMETYPE_PARAMETER_WRITE
    struct PACKED ParameterSettingsWriteFrame {
        uint8_t destination;
        uint8_t origin;
        uint8_t param_num;
        uint8_t payload[57];   // largest possible frame is 60
    };

    // Frame to hold passthrough telemetry
    struct PACKED PassthroughSinglePacketFrame {
        uint8_t sub_type;
        uint16_t appid;
        uint32_t data;
    };

    // Frame to hold passthrough telemetry
    struct PACKED PassthroughMultiPacketFrame {
        uint8_t sub_type;
        uint8_t size;
        struct PACKED PassthroughTelemetryPacket {
            uint16_t appid;
            uint32_t data;
        } packets[PASSTHROUGH_MULTI_PACKET_FRAME_MAX_SIZE];
    };

    // Frame to hold status text message
    struct PACKED StatusTextFrame {
        uint8_t sub_type;
        uint8_t severity;
        char text[PASSTHROUGH_STATUS_TEXT_FRAME_MAX_SIZE];  // ( Null-terminated string )
    };

    // ardupilot frametype container
    union PACKED APCustomTelemFrame {
        PassthroughSinglePacketFrame single_packet_passthrough;
        PassthroughMultiPacketFrame multi_packet_passthrough;
        StatusTextFrame status_text;
    };


    union PACKED BroadcastFrame {
        GPSFrame gps;
        HeartbeatFrame heartbeat;
        BatteryFrame battery;
        AttitudeFrame attitude;
        FlightModeFrame flightmode;
        APCustomTelemFrame custom_telem;
    };

    union PACKED ExtendedFrame {
        CommandFrame command;
        ParameterPingFrame ping;
        ParameterDeviceInfoFrame info;
        ParameterSettingsEntry param_entry;
        ParameterSettingsReadFrame param_read;
        ParameterSettingsWriteFrame param_write;
    };

    union PACKED TelemetryPayload {
        BroadcastFrame bcast;
        ExtendedFrame ext;
    };

    // get the protocol string
    const char* get_protocol_string() const {
        if (_crsf_version.is_elrs) {
            return "ELRS";
        } else {
            const AP_RCProtocol_CRSF* crsf = AP::crsf();
            if (crsf && crsf->is_crsf_v3_active()) {
                return "CRSFv3";
            }
            return "CRSFv2";
        }
    };
    // Process a frame from the CRSF protocol decoder
    static bool process_frame(AP_RCProtocol_CRSF::FrameType frame_type, void* data);
    // process any changed settings and schedule for transmission
    void update();

    // setup the scheduler for doing other things
    void disable_telemetry(uint32_t enable_at_ms);
    void enable_telemetry();

    // get next telemetry data for external consumers of SPort data
    static bool get_telem_data(AP_RCProtocol_CRSF::Frame* frame, bool is_tx_active);

private:

    enum SensorType {
        HEARTBEAT,
        PARAMETERS,
        ATTITUDE,
        VTX_PARAMETERS,
        BATTERY,
        GPS,
        FLIGHT_MODE,
        PASSTHROUGH,
        STATUS_TEXT,
        GENERAL_COMMAND,
        VERSION_PING,
        DEVICE_PING,
        NUM_SENSORS
    };

    // passthrough WFQ scheduler
    bool is_packet_ready(uint8_t idx, bool queue_empty) override;
    void process_packet(uint8_t idx) override;
    void adjust_packet_weight(bool queue_empty) override;
    void setup_custom_telemetry();
    void update_custom_telemetry_rates(const AP_RCProtocol_CRSF::RFMode rf_mode);

    void calc_parameter_ping();
    void calc_heartbeat();
    void calc_battery();
    void calc_gps();
    void calc_attitude();
    void calc_flight_mode();
    void calc_device_info();
    void calc_device_ping(uint8_t destination);
    void calc_command_response();
    void calc_parameter();
#if HAL_CRSF_TELEM_TEXT_SELECTION_ENABLED
    void calc_text_selection( AP_OSD_ParamSetting* param, uint8_t chunk);
#endif
    void process_pending_requests();
    void update_vtx_params();
    void get_single_packet_passthrough_telem_data();
    void get_multi_packet_passthrough_telem_data(uint8_t size = PASSTHROUGH_MULTI_PACKET_FRAME_MAX_SIZE);
    void calc_status_text();
    bool process_rf_mode_changes();
    uint8_t get_custom_telem_frame_id() const;
    AP_RCProtocol_CRSF::RFMode get_rf_mode() const;
    uint16_t get_telemetry_rate() const;
    bool is_high_speed_telemetry(const AP_RCProtocol_CRSF::RFMode rf_mode) const;

    void process_vtx_telem_frame(VTXTelemetryFrame* vtx);
    void process_vtx_frame(bool pitmode);
    void process_ping_frame(ParameterPingFrame* ping);
    void process_param_read_frame(ParameterSettingsReadFrame* read);
    void process_param_write_frame(ParameterSettingsWriteFrame* write);
    void process_device_info_frame(ParameterDeviceInfoFrame* info);
    void process_command_frame(CommandFrame* command);
    bool is_vtx_change_pending() const;

    // setup ready for passthrough operation
    void setup_wfq_scheduler(void) override;

    // setup the scheduler for parameters download
    void enter_scheduler_params_mode();
    void exit_scheduler_params_mode();
    void disable_tx_entries();
    void enable_tx_entries();

    // get next telemetry data for external consumers
    bool _get_telem_data(AP_RCProtocol_CRSF::Frame* data, bool is_tx_active);
    bool _process_frame(AP_RCProtocol_CRSF::FrameType frame_type, void* data);

    TelemetryPayload _telem;
    uint8_t _telem_size;
    uint8_t _telem_type;
    AP_RCProtocol_CRSF::RFMode _telem_rf_mode;
    // reporting telemetry rate
    uint32_t _telem_last_report_ms;
    uint16_t _telem_last_avg_rate;
    // do we need to report the initial state
    bool _telem_bootstrap_msg_pending;

    bool _telem_is_high_speed;
    bool _telem_pending;
    bool _enable_telemetry;
    // used to limit telemetry when in a failsafe condition
    bool _is_tx_active;

    struct {
        uint8_t destination = AP_RCProtocol_CRSF::CRSF_ADDRESS_BROADCAST;
        uint8_t frame_type;
    } _pending_request;

    struct {
        uint8_t minor;
        uint8_t major;
        uint8_t retry_count;
        bool use_rf_mode;
        bool is_tracer;
        bool pending = true;
        bool is_elrs;
    } _crsf_version;

    struct {
        bool init_done;
        uint32_t disable_telemetry_timeout_ms;
        bool disable_telemetry;
    } _custom_telem;

    struct {
        bool pending;
        bool valid;
        uint8_t port_id;
    } _baud_rate_request;

    // vtx state
    // work around a bug in most versions of CRSF where the status message does not reflect the VTX status
    bool ignore_vtx_status() const { return AP::vtx().has_option(AP_VideoTX::VideoOptions::VTX_CRSF_IGNORE_STAT); }
    // still use the status message for determining liveness
    bool vtx_is_alive;

    uint8_t _vtx_freq_change_pending; // a vtx command has been issued but not confirmed by a vtx broadcast frame
    uint8_t _vtx_power_change_pending;
    uint8_t _vtx_options_change_pending;
    enum class PitmodeChangeState {
        PitmodeOn,
        PitmodeOff,
        PowerupFromPitmode
    };

    // number of times to set a parameter before stopping
    static const uint8_t VTX_SETTINGS_RETRIES = 3;
    PitmodeChangeState _vtx_pitmode_change;

    bool _noted_lq_as_rssi_active;

    static AP_CRSF_Telem *singleton;
};

namespace AP {
    AP_CRSF_Telem *crsf_telem();
};

#endif
