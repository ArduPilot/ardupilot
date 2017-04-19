/// @file	GCS.h
/// @brief	Interface definition for the various Ground Control System
// protocols.
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include "GCS_MAVLink.h"
#include <DataFlash/DataFlash.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <stdint.h>
#include "MAVLink_routing.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Mount/AP_Mount.h>
#include <AP_Avoidance/AP_Avoidance.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_Frsky_Telem/AP_Frsky_Telem.h>

// check if a message will fit in the payload space available
#define HAVE_PAYLOAD_SPACE(chan, id) (comm_get_txspace(chan) >= GCS_MAVLINK::packet_overhead_chan(chan)+MAVLINK_MSG_ID_ ## id ## _LEN)
#define CHECK_PAYLOAD_SIZE(id) if (comm_get_txspace(chan) < packet_overhead()+MAVLINK_MSG_ID_ ## id ## _LEN) return false
#define CHECK_PAYLOAD_SIZE2(id) if (!HAVE_PAYLOAD_SPACE(chan, id)) return false

//  GCS Message ID's
/// NOTE: to ensure we never block on sending MAVLink messages
/// please keep each MSG_ to a single MAVLink message. If need be
/// create new MSG_ IDs for additional messages on the same
/// stream
enum ap_message {
    MSG_HEARTBEAT,
    MSG_ATTITUDE,
    MSG_LOCATION,
    MSG_EXTENDED_STATUS1,
    MSG_EXTENDED_STATUS2,
    MSG_NAV_CONTROLLER_OUTPUT,
    MSG_CURRENT_WAYPOINT,
    MSG_VFR_HUD,
    MSG_SERVO_OUTPUT_RAW,
    MSG_RADIO_IN,
    MSG_RAW_IMU1,
    MSG_RAW_IMU2,
    MSG_RAW_IMU3,
    MSG_GPS_RAW,
    MSG_SYSTEM_TIME,
    MSG_SERVO_OUT,
    MSG_NEXT_WAYPOINT,
    MSG_NEXT_PARAM,
    MSG_STATUSTEXT,
    MSG_LIMITS_STATUS,
    MSG_FENCE_STATUS,
    MSG_AHRS,
    MSG_SIMSTATE,
    MSG_HWSTATUS,
    MSG_WIND,
    MSG_RANGEFINDER,
    MSG_TERRAIN,
    MSG_BATTERY2,
    MSG_CAMERA_FEEDBACK,
    MSG_MOUNT_STATUS,
    MSG_OPTICAL_FLOW,
    MSG_GIMBAL_REPORT,
    MSG_MAG_CAL_PROGRESS,
    MSG_MAG_CAL_REPORT,
    MSG_EKF_STATUS_REPORT,
    MSG_LOCAL_POSITION,
    MSG_PID_TUNING,
    MSG_VIBRATION,
    MSG_RPM,
    MSG_MISSION_ITEM_REACHED,
    MSG_POSITION_TARGET_GLOBAL_INT,
    MSG_ADSB_VEHICLE,
    MSG_BATTERY_STATUS,
    MSG_RETRY_DEFERRED // this must be last
};

///
/// @class	GCS_MAVLINK
/// @brief	MAVLink transport control class
///
class GCS_MAVLINK
{
public:
    GCS_MAVLINK();
    FUNCTOR_TYPEDEF(run_cli_fn, void, AP_HAL::UARTDriver*);
    void        update(run_cli_fn run_cli);
    void        init(AP_HAL::UARTDriver *port, mavlink_channel_t mav_chan);
    void        setup_uart(const AP_SerialManager& serial_manager, AP_SerialManager::SerialProtocol protocol, uint8_t instance);
    void        send_message(enum ap_message id);
    void        send_text(MAV_SEVERITY severity, const char *str);
    virtual void        data_stream_send(void) = 0;
    void        queued_param_send();
    void        queued_waypoint_send();
    void        set_snoop(void (*_msg_snoop)(const mavlink_message_t* msg)) {
        msg_snoop = _msg_snoop;
    }
    // packetReceived is called on any successful decode of a mavlink message
    virtual void packetReceived(const mavlink_status_t &status,
                                mavlink_message_t &msg);

    // accessor for uart
    AP_HAL::UARTDriver *get_uart() { return _port; }

    static const struct AP_Param::GroupInfo        var_info[];

    // set to true if this GCS link is active
    bool            initialised;

    // NOTE! The streams enum below and the
    // set of AP_Int16 stream rates _must_ be
    // kept in the same order
    enum streams {STREAM_RAW_SENSORS,
                  STREAM_EXTENDED_STATUS,
                  STREAM_RC_CHANNELS,
                  STREAM_RAW_CONTROLLER,
                  STREAM_POSITION,
                  STREAM_EXTRA1,
                  STREAM_EXTRA2,
                  STREAM_EXTRA3,
                  STREAM_PARAMS,
                  STREAM_ADSB,
                  NUM_STREAMS};

    // see if we should send a stream now. Called at 50Hz
    bool        stream_trigger(enum streams stream_num);

    // call to reset the timeout window for entering the cli
    void reset_cli_timeout();

    uint32_t        last_heartbeat_time; // milliseconds

    // last time we got a non-zero RSSI from RADIO_STATUS
    static uint32_t last_radio_status_remrssi_ms;

    // mission item index to be sent on queued msg, delayed or not
    uint16_t mission_item_reached_index = AP_MISSION_CMD_INDEX_NONE;

    // common send functions
    void send_meminfo(void);
    void send_power_status(void);
    void send_battery_status(const AP_BattMonitor &battery, const uint8_t instance) const;
    bool send_battery_status(const AP_BattMonitor &battery) const;
    void send_ahrs2(AP_AHRS &ahrs);
    bool send_gps_raw(AP_GPS &gps);
    void send_system_time(AP_GPS &gps);
    void send_radio_in(uint8_t receiver_rssi);
    void send_raw_imu(const AP_InertialSensor &ins, const Compass &compass);
    void send_scaled_pressure(AP_Baro &barometer);
    void send_sensor_offsets(const AP_InertialSensor &ins, const Compass &compass, AP_Baro &barometer);
    void send_ahrs(AP_AHRS &ahrs);
    void send_battery2(const AP_BattMonitor &battery);
#if AP_AHRS_NAVEKF_AVAILABLE
    void send_opticalflow(AP_AHRS_NavEKF &ahrs, const OpticalFlow &optflow);
#endif
    void send_autopilot_version(uint8_t major_version, uint8_t minor_version, uint8_t patch_version, uint8_t version_type) const;
    void send_local_position(const AP_AHRS &ahrs) const;
    void send_vibration(const AP_InertialSensor &ins) const;
    void send_home(const Location &home) const;
    static void send_home_all(const Location &home);
    void send_heartbeat(uint8_t type, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status);
    void send_servo_output_raw(bool hil);
    static void send_collision_all(const AP_Avoidance::Obstacle &threat, MAV_COLLISION_ACTION behaviour);
    void send_accelcal_vehicle_position(uint8_t position);

    // return a bitmap of active channels. Used by libraries to loop
    // over active channels to send to all active channels    
    static uint8_t active_channel_mask(void) { return mavlink_active; }

    // return a bitmap of streaming channels
    static uint8_t streaming_channel_mask(void) { return chan_is_streaming; }

    /*
    send a statustext message to active MAVLink connections, or a specific
    one. This function is static so it can be called from any library.
    */
    static void send_statustext_all(MAV_SEVERITY severity, const char *fmt, ...);
    static void send_statustext_chan(MAV_SEVERITY severity, uint8_t dest_chan, const char *fmt, ...);

    // send a PARAM_VALUE message to all active MAVLink connections.
    static void send_parameter_value_all(const char *param_name, ap_var_type param_type, float param_value);
    
    /*
      send a MAVLink message to all components with this vehicle's system id
      This is a no-op if no routes to components have been learned
    */
    static void send_to_components(const mavlink_message_t* msg) { routing.send_to_components(msg); }
    
    /*
      allow forwarding of packets / heartbeats to be blocked as required by some components to reduce traffic
    */
    static void disable_channel_routing(mavlink_channel_t chan) { routing.no_route_mask |= (1U<<(chan-MAVLINK_COMM_0)); }
    
    /*
      search for a component in the routing table with given mav_type and retrieve it's sysid, compid and channel
      returns if a matching component is found
     */
    static bool find_by_mavtype(uint8_t mav_type, uint8_t &sysid, uint8_t &compid, mavlink_channel_t &channel) { return routing.find_by_mavtype(mav_type, sysid, compid, channel); }

    // update signing timestamp on GPS lock
    static void update_signing_timestamp(uint64_t timestamp_usec);

    // return current packet overhead for a channel
    static uint8_t packet_overhead_chan(mavlink_channel_t chan);

    // FIXME: move this to be private/protected once possible
    bool telemetry_delayed(mavlink_channel_t chan);
    virtual uint32_t telem_delay() const = 0;

protected:

    // overridable method to check for packet acceptance. Allows for
    // enforcement of GCS sysid
    virtual bool accept_packet(const mavlink_status_t &status, mavlink_message_t &msg) { return true; }
    
    bool            waypoint_receiving; // currently receiving
    // the following two variables are only here because of Tracker
    uint16_t        waypoint_request_i; // request index
    uint16_t        waypoint_request_last; // last request index

    AP_Param *                  _queued_parameter;      ///< next parameter to
                                                        // be sent in queue
    mavlink_channel_t           chan;
    uint8_t packet_overhead(void) const { return packet_overhead_chan(chan); }

    void handle_log_send(DataFlash_Class &dataflash);

    // saveable rate of each stream
    AP_Int16        streamRates[NUM_STREAMS];

    void handle_request_data_stream(mavlink_message_t *msg, bool save);
    FUNCTOR_TYPEDEF(set_mode_fn, bool, uint8_t);
    void handle_set_mode(mavlink_message_t* msg, set_mode_fn set_mode);

    void handle_mission_request_list(AP_Mission &mission, mavlink_message_t *msg);
    void handle_mission_request(AP_Mission &mission, mavlink_message_t *msg);
    void handle_mission_clear_all(AP_Mission &mission, mavlink_message_t *msg);
    void handle_mission_set_current(AP_Mission &mission, mavlink_message_t *msg);
    void handle_mission_count(AP_Mission &mission, mavlink_message_t *msg);
    void handle_mission_write_partial_list(AP_Mission &mission, mavlink_message_t *msg);
    bool handle_mission_item(mavlink_message_t *msg, AP_Mission &mission);

    void handle_param_set(mavlink_message_t *msg, DataFlash_Class *DataFlash);
    void handle_param_request_list(mavlink_message_t *msg);
    void handle_param_request_read(mavlink_message_t *msg);

    void handle_gimbal_report(AP_Mount &mount, mavlink_message_t *msg) const;
    void handle_radio_status(mavlink_message_t *msg, DataFlash_Class &dataflash, bool log_radio);
    void handle_serial_control(mavlink_message_t *msg, AP_GPS &gps);

    void handle_gps_inject(const mavlink_message_t *msg, AP_GPS &gps);

    void handle_common_message(mavlink_message_t *msg);
    void handle_log_message(mavlink_message_t *msg, DataFlash_Class &dataflash);
    void handle_setup_signing(const mavlink_message_t *msg);
    uint8_t handle_preflight_reboot(const mavlink_command_long_t &packet, bool disable_overrides);
    uint8_t handle_rc_bind(const mavlink_command_long_t &packet);

    void handle_device_op_read(mavlink_message_t *msg);
    void handle_device_op_write(mavlink_message_t *msg);

    void handle_timesync(mavlink_message_t *msg);
    
private:

    float       adjust_rate_for_stream_trigger(enum streams stream_num);

    virtual void        handleMessage(mavlink_message_t * msg) = 0;

    /// The stream we are communicating over
    AP_HAL::UARTDriver *_port;

    /// Perform queued sending operations
    ///
    enum ap_var_type            _queued_parameter_type; ///< type of the next
                                                        // parameter
    AP_Param::ParamToken        _queued_parameter_token; ///AP_Param token for
                                                         // next() call
    uint16_t                    _queued_parameter_index; ///< next queued
                                                         // parameter's index
    uint16_t                    _queued_parameter_count; ///< saved count of
                                                         // parameters for
                                                         // queued send
    uint32_t                    _queued_parameter_send_time_ms;

    /// Count the number of reportable parameters.
    ///
    /// Not all parameters can be reported via MAVlink.  We count the number
    // that are
    /// so that we can report to a GCS the number of parameters it should
    // expect when it
    /// requests the full set.
    ///
    /// @return         The number of reportable parameters.
    ///
    uint16_t                    packet_drops;

    // this allows us to detect the user wanting the CLI to start
    uint8_t        crlf_count;

    // waypoints
    uint16_t        waypoint_dest_sysid; // where to send requests
    uint16_t        waypoint_dest_compid; // "
    uint16_t        waypoint_count;
    uint32_t        waypoint_timelast_receive; // milliseconds
    uint32_t        waypoint_timelast_request; // milliseconds
    const uint16_t  waypoint_receive_timeout = 8000; // milliseconds

    // number of 50Hz ticks until we next send this stream
    uint8_t         stream_ticks[NUM_STREAMS];

    // number of extra ticks to add to slow things down for the radio
    uint8_t         stream_slowdown;

    // millis value to calculate cli timeout relative to.
    // exists so we can separate the cli entry time from the system start time
    uint32_t _cli_timeout;

    uint8_t  _log_listing:1; // sending log list
    uint8_t  _log_sending:1; // sending log data

    // next log list entry to send
    uint16_t _log_next_list_entry;

    // last log list entry to send
    uint16_t _log_last_list_entry;

    // number of log files
    uint16_t _log_num_logs;

    // log number for data send
    uint16_t _log_num_data;

    // offset in log
    uint32_t _log_data_offset;

    // size of log file
    uint32_t _log_data_size;

    // number of bytes left to send
    uint32_t _log_data_remaining;

    // start page of log data
    uint16_t _log_data_page;

    // deferred message handling
    enum ap_message deferred_messages[MSG_RETRY_DEFERRED];
    uint8_t next_deferred_message;
    uint8_t num_deferred_messages;

    // time when we missed sending a parameter for GCS
    static uint32_t reserve_param_space_start_ms;
    
    // bitmask of what mavlink channels are active
    static uint8_t mavlink_active;

    // bitmask of what mavlink channels are streaming
    static uint8_t chan_is_streaming;

    // mavlink routing object
    static MAVLink_routing routing;

    // pointer to static frsky_telem for queueing of text messages
    static AP_Frsky_Telem *frsky_telemetry_p;
 
    static const AP_SerialManager *serialmanager_p;

    // a vehicle can optionally snoop on messages for other systems
    static void (*msg_snoop)(const mavlink_message_t* msg);

    // vehicle specific message send function
    virtual bool try_send_message(enum ap_message id) = 0;

    virtual bool handle_guided_request(AP_Mission::Mission_Command &cmd) = 0;
    virtual void handle_change_alt_request(AP_Mission::Mission_Command &cmd) = 0;

    void handle_log_request_list(mavlink_message_t *msg, DataFlash_Class &dataflash);
    void handle_log_request_data(mavlink_message_t *msg, DataFlash_Class &dataflash);
    void handle_log_request_erase(mavlink_message_t *msg, DataFlash_Class &dataflash);
    void handle_log_request_end(mavlink_message_t *msg, DataFlash_Class &dataflash);
    void handle_log_send_listing(DataFlash_Class &dataflash);
    bool handle_log_send_data(DataFlash_Class &dataflash);


    void lock_channel(mavlink_channel_t chan, bool lock);

    // return true if this channel has hardware flow control
    bool have_flow_control(void);

    mavlink_signing_t signing;
    static mavlink_signing_streams_t signing_streams;
    static uint32_t last_signing_save_ms;
    
    static StorageAccess _signing_storage;
    static bool signing_key_save(const struct SigningKey &key);
    static bool signing_key_load(struct SigningKey &key);
    void load_signing_key(void);
    bool signing_enabled(void) const;
    static void save_signing_timestamp(bool force_save_now);
};

/// @class GCS
/// @brief global GCS object
class GCS
{

public:

    GCS() {
        if (_singleton  == nullptr) {
            _singleton = this;
        } else {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            // this is a serious problem, but we don't need to kill a
            // real vehicle
            AP_HAL::panic("GCS must be singleton");
#endif
        }
    };

    static class GCS *instance() {
        return _singleton;
    }

    virtual void send_statustext(MAV_SEVERITY severity, uint8_t dest_bitmask, const char *text);
    void service_statustext(void);

    /*
      set a dataflash pointer for logging
     */
    void set_dataflash(DataFlash_Class *dataflash) {
        dataflash_p = dataflash;
    }

    // pointer to static dataflash for logging of text messages
    DataFlash_Class *dataflash_p;


    /*
      set a frsky_telem pointer for queueing
     */
    void register_frsky_telemetry_callback(AP_Frsky_Telem *frsky_telemetry) {
        frsky_telemetry_p = frsky_telemetry;
    }

    // static frsky_telem pointer to support queueing text messages
    AP_Frsky_Telem *frsky_telemetry_p;

private:

    static GCS *_singleton;

    struct statustext_t {
        uint8_t                 bitmask;
        mavlink_statustext_t    msg;
    };

#if HAL_CPU_CLASS <= HAL_CPU_CLASS_150 || CONFIG_HAL_BOARD == HAL_BOARD_SITL
    static const uint8_t _status_capacity = 5;
#else
    static const uint8_t _status_capacity = 30;
#endif

    ObjectArray<statustext_t> _statustext_queue{_status_capacity};

};

GCS &gcs();
