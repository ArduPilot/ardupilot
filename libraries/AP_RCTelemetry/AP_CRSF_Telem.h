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

#if HAL_CRSF_TELEM_ENABLED

#include <AP_OSD/AP_OSD.h>
#include <AP_RCProtocol/AP_RCProtocol_CRSF.h>
#include "AP_RCTelemetry.h"
#include <AP_HAL/utility/sparse-endian.h>

class AP_OSD_ParamSetting;

class AP_CRSF_Telem : public AP_RCTelemetry {
public:
    AP_CRSF_Telem();
    ~AP_CRSF_Telem() override;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_CRSF_Telem);

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

    struct PACKED BaroVarioFrame {
        uint16_t altitude_packed; // Altitude above start (calibration) point.
        int8_t vertical_speed_packed; // vertical speed.
    };

    struct PACKED VarioFrame {
        int16_t v_speed; // vertical speed cm/s
    };

    struct PACKED VTXFrame {
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
    };

    struct PACKED VTXTelemetryFrame {
        uint8_t origin; // address
        uint8_t power;              // power in dBm
        uint16_t frequency;         // frequency in Mhz
        uint8_t pitmode;            // disable 0, enable 1
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
    struct PACKED ParameterSettingsHeader {
        uint8_t destination;
        uint8_t origin;
        uint8_t param_num;
    };

    struct PACKED ParameterSettingsEntryHeader : public ParameterSettingsHeader {
        uint8_t chunks_left;
    };

    // CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY
    struct PACKED ParameterSettingsEntry {
        ParameterSettingsEntryHeader header;
        uint8_t payload[56];   // largest possible frame is 60
    };

    // CRSF_FRAMETYPE_PARAMETER_READ
    struct PACKED ParameterSettingsReadFrame : public ParameterSettingsHeader {
        uint8_t param_chunk;
    };

    // CRSF_FRAMETYPE_PARAMETER_WRITE
    struct PACKED ParameterSettingsWriteFrame : public ParameterSettingsHeader {
        uint8_t payload[57];   // largest possible frame is 60
    };

    struct ParameterPayload {
        uint8_t payload_length;
        uint8_t payload[57];
    };

    // Generic pending parameter request, used internally
    struct PendingParameterRequest : public ParameterSettingsReadFrame {
        ParameterPayload payload;
    } _param_request;

    const static uint8_t PARAMETER_MENU_ID = 1; // id of the parameter menu

#if AP_CRSF_SCRIPTING_ENABLED
    // scripted CRSF menus
    // menus follow the predefined ardupilot parameter menu
    // to avoid a lot of id shuffling at most 10 menus each with at most 20 parameters are allowed
    // menu indexes are SCRIPTED_MENU_START_ID -> SCRIPTED_MENU_START_ID + 10
    // parameter indexes are SCRIPTED_MENU_START_ID + 10 + menu_id * MAX_SCRIPTED_MENU_SIZE
    const static uint8_t MAX_SCRIPTED_MENUS = 10U;
    const static uint8_t MAX_SCRIPTED_MENU_SIZE = 20U;
    const static uint8_t MAX_SCRIPTED_PARAMETERS = 255U;
    const static uint8_t MAX_SCRIPTED_PARAMETER_SIZE = 255U;
    const static uint8_t MAX_SCRIPTED_MENU_NAME_LEN = 16;
    const static uint8_t SCRIPTED_MENU_START_ID = AP_OSD_ParamScreen::NUM_PARAMS * AP_OSD_NUM_PARAM_SCREENS + 2;

    // 8-bit parameter ids must be unique within the whole menu structure
    // each parameter has an id, length and packed data
    // to avoid heavy flash usage in the CRSF protocol implementation, the data encoding is
    // managed in lua
    struct ScriptedEntry {
        uint8_t id; // indexed from the menu id + 1 to menu id + MAX_SCRIPTED_MENU_SIZE
        uint8_t parent_id;
    };

    struct ScriptedParameter : public ScriptedEntry {
        uint16_t length;
        const char* data;
    };

    // each menu contains a number of parameters and has a name
    struct ScriptedMenu : public ScriptedEntry {
        friend class AP_CRSF_Telem;

        uint8_t num_params;
        const char* name;
        ScriptedParameter* params;
        ScriptedMenu* next_menu;    // linked list of menus to make addition/removal/modification easy

        // public API called from lua
        ScriptedMenu* add_menu(const char* menu_name, uint8_t size, uint8_t parent_menu);
        ScriptedParameter* add_parameter(uint8_t length, const char* data);

        // private API
        ScriptedMenu(const char* menu_name, uint8_t size, uint8_t parent_menu);
        ~ScriptedMenu();
        ScriptedParameter* find_parameter(uint8_t param_num);
        ScriptedMenu* find_menu(uint8_t param_num);
        bool remove_menu(uint8_t param_num);

        void dump_structure(uint8_t indent);
        ScriptedMenu() {}
    };

    enum ScriptedParameterEvents : uint8_t {
        PARAMETER_READ = 1<<0,
        PARAMETER_WRITE = 1<<1
    };

    ScriptedMenu scripted_menus;

    typedef ParameterPayload ScriptedPayload;

    struct ScriptedParameterWrite {
        ScriptedParameterEvents type;
        uint8_t destination;
        uint8_t origin;
        uint8_t param_num;
        uint8_t param_chunk;
        ScriptedParameter* param;
        ScriptedPayload payload;
    };

    // scripted menus represent a complicated dance between the transmitter, flight controller and lua
    // lua-in-the-loop requires mediation via outbound_params whereas direct responses can pass straight to
    // ready_params. This is made more complicated by the protocol not being request-response - requests to
    // read can be interleaved with ongoing requests to write
    ObjectBuffer<ScriptedParameterWrite> inbound_params{8}; // parameter requests waiting to be processed
    ObjectBuffer<ScriptedParameterWrite> outbound_params{8};// parameter repsonses waiting to be processed
    ObjectBuffer<ScriptedParameterWrite> ready_params{8};   // parameter responses ready to be scheduled
    HAL_Semaphore scr_sem; // semaphore guarding access to the inbound and outbound queues
    // access so that submenus can serialize access from scripting
    HAL_Semaphore& get_semaphore() { return scr_sem; }

    void clear_menus();
    bool process_scripted_param_write(ParameterSettingsWriteFrame* write, uint8_t length);
    bool process_scripted_param_read(ParameterSettingsReadFrame* read);
    void dump_menu_structure();

    // public API called from lua
    // add a scripted sub-menu to this menu
    ScriptedMenu* add_menu(const char* name);
    // get a menu event filtered by event type, this adds the event to the outbound queue for processing
    uint8_t get_menu_event(uint8_t menu_events, uint8_t& param_id, ScriptedPayload& payload);
    // peek for a menu event filtered by event type, this makes no changes to the outbound queue
    // returns the type of event at the head of the queue and the CRSF payload data from that event
    uint8_t peek_menu_event(uint8_t& param_id, ScriptedPayload& payload, uint8_t& events);
    // pop a menu event from the inbound queue and add to the outbound queue for processing via lua
    void pop_menu_event();
    // send a new response from the first item in the outbound queueu
    bool send_write_response(uint8_t length, const char* data);
    // send a generic response from the first item in the outbound queue
    bool send_response();
#endif

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
        BaroVarioFrame baro_vario;
        VarioFrame vario;
        BatteryFrame battery;
        VTXFrame vtx;
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
    const char* get_protocol_string() const { return AP::crsf()->get_protocol_string(_crsf_version.protocol); }

    // is the current protocol ELRS?
    bool is_elrs() const { return _crsf_version.protocol == AP_RCProtocol_CRSF::ProtocolType::PROTOCOL_ELRS; }
    // is the current protocol Tracer?
    bool is_tracer() const { return _crsf_version.protocol == AP_RCProtocol_CRSF::ProtocolType::PROTOCOL_TRACER; }

    // Process a frame from the CRSF protocol decoder
    static bool process_frame(AP_RCProtocol_CRSF::FrameType frame_type, void* data, uint8_t length);
    // get next telemetry data for external consumers of SPort data
    static bool get_telem_data(AP_RCProtocol_CRSF::Frame* frame, bool is_tx_active);
    // start bind request
    void start_bind() { _bind_request_pending = true; }
    bool bind_in_progress() { return _bind_request_pending;}

private:

    enum SensorType {
        HEARTBEAT,
        PARAMETERS,
        BARO_VARIO,
        VARIO,
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
    uint16_t get_altitude_packed();
    int8_t get_vertical_speed_packed();
    void calc_baro_vario();
    void calc_vario();
    void calc_gps();
    void calc_attitude();
    void calc_flight_mode();
    void calc_device_info();
    void calc_device_ping(uint8_t destination);
    void calc_command_response();
    void calc_bind();
    void calc_parameter();
#if AP_CRSF_SCRIPTING_ENABLED
    bool calc_scripted_parameter();
#endif
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

    void process_vtx_frame(VTXFrame* vtx);
    void process_vtx_telem_frame(VTXTelemetryFrame* vtx);
    void process_ping_frame(ParameterPingFrame* ping);
    void process_param_read_frame(ParameterSettingsReadFrame* read);
    void process_param_write_frame(ParameterSettingsWriteFrame* write, uint8_t length);
    void process_device_info_frame(ParameterDeviceInfoFrame* info);
    void process_command_frame(CommandFrame* command);

    // setup ready for passthrough operation
    void setup_wfq_scheduler(void) override;

    // setup the scheduler for parameters download
    void enter_scheduler_params_mode();
    void exit_scheduler_params_mode();
    bool should_enter_scheduler_params_mode() const;
    void disable_tx_entries();
    void enable_tx_entries();

    // get next telemetry data for external consumers
    bool _get_telem_data(AP_RCProtocol_CRSF::Frame* data, bool is_tx_active);
    bool _process_frame(AP_RCProtocol_CRSF::FrameType frame_type, void* data, uint8_t length);

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
        AP_RCProtocol_CRSF::ProtocolType protocol;
        bool pending = true;
        uint32_t last_request_info_ms;
    } _crsf_version;

    struct {
        bool init_done;
        uint32_t params_mode_start_ms;
        bool params_mode_active;
    } _custom_telem;

    struct {
        bool pending;
        bool valid;
        uint8_t port_id;
    } _baud_rate_request;

    bool _bind_request_pending;

    // vtx state
    bool _vtx_freq_update;  // update using the frequency method or not
    bool _vtx_dbm_update; // update using the dbm method or not
    bool _vtx_freq_change_pending; // a vtx command has been issued but not confirmed by a vtx broadcast frame
    bool _vtx_power_change_pending;
    bool _vtx_options_change_pending;

    bool _noted_lq_as_rssi_active;

    static AP_CRSF_Telem *singleton;
};

namespace AP {
    AP_CRSF_Telem *crsf_telem();
};

#endif
