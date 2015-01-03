// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	GCS.h
/// @brief	Interface definition for the various Ground Control System
// protocols.

#ifndef __GCS_H
#define __GCS_H

#include <AP_HAL.h>
#include <AP_Common.h>
#include <GCS_MAVLink.h>
#include <DataFlash.h>
#include <AP_Mission.h>
#include "../AP_BattMonitor/AP_BattMonitor.h"
#include <stdint.h>
#include <MAVLink_routing.h>

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
    MSG_RADIO_OUT,
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
    void        update(void (*run_cli)(AP_HAL::UARTDriver *));
    void        init(AP_HAL::UARTDriver *port);
    void        setup_uart(AP_HAL::UARTDriver *port, uint32_t baudrate, uint16_t rxS, uint16_t txS);
    void        send_message(enum ap_message id);
    void        send_text(gcs_severity severity, const char *str);
    void        send_text_P(gcs_severity severity, const prog_char_t *str);
    void        data_stream_send(void);
    void        queued_param_send();
    void        queued_waypoint_send();
    void        set_snoop(void (*_msg_snoop)(const mavlink_message_t* msg)) {
        msg_snoop = _msg_snoop;
    }

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
                  NUM_STREAMS};

    // see if we should send a stream now. Called at 50Hz
    bool        stream_trigger(enum streams stream_num);

	// this costs us 51 bytes per instance, but means that low priority
	// messages don't block the CPU
    mavlink_statustext_t pending_status;

    // call to reset the timeout window for entering the cli
    void reset_cli_timeout();

    uint32_t        last_heartbeat_time; // milliseconds

    // last time we got a non-zero RSSI from RADIO_STATUS
    static uint32_t last_radio_status_remrssi_ms;

    // common send functions
    void send_meminfo(void);
    void send_power_status(void);
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

    // return a bitmap of active channels. Used by libraries to loop
    // over active channels to send to all active channels    
    static uint8_t active_channel_mask(void) { return mavlink_active; }

    /*
      send a statustext message to all active MAVLink
      connections. This function is static so it can be called from
      any library
    */
    static void send_statustext_all(const prog_char_t *msg);

private:
    void        handleMessage(mavlink_message_t * msg);

    /// The stream we are communicating over
    AP_HAL::UARTDriver *_port;

    /// Perform queued sending operations
    ///
    AP_Param *                  _queued_parameter;      ///< next parameter to
                                                        // be sent in queue
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
    uint16_t                    _count_parameters(); ///< count reportable
                                                     // parameters

    uint16_t                    _parameter_count;   ///< cache of reportable
                                                    // parameters

    mavlink_channel_t           chan;
    uint16_t                    packet_drops;

#if CLI_ENABLED == ENABLED
    // this allows us to detect the user wanting the CLI to start
    uint8_t        crlf_count;
#endif

    // waypoints
    uint16_t        waypoint_request_i; // request index
    uint16_t        waypoint_request_last; // last request index
    uint16_t        waypoint_dest_sysid; // where to send requests
    uint16_t        waypoint_dest_compid; // "
    bool            waypoint_receiving; // currently receiving
    uint16_t        waypoint_count;
    uint32_t        waypoint_timelast_receive; // milliseconds
    uint32_t        waypoint_timelast_request; // milliseconds
    const uint16_t  waypoint_receive_timeout; // milliseconds

    // saveable rate of each stream
    AP_Int16        streamRates[NUM_STREAMS];

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

    // bitmask of what mavlink channels are active
    static uint8_t mavlink_active;

    // mavlink routing object
    static MAVLink_routing routing;

    // a vehicle can optionally snoop on messages for other systems
    static void (*msg_snoop)(const mavlink_message_t* msg);

    // vehicle specific message send function
    bool try_send_message(enum ap_message id);

    void handle_guided_request(AP_Mission::Mission_Command &cmd);
    void handle_change_alt_request(AP_Mission::Mission_Command &cmd);

    void handle_log_request_list(mavlink_message_t *msg, DataFlash_Class &dataflash);
    void handle_log_request_data(mavlink_message_t *msg, DataFlash_Class &dataflash);
    void handle_log_request_erase(mavlink_message_t *msg, DataFlash_Class &dataflash);
    void handle_log_request_end(mavlink_message_t *msg, DataFlash_Class &dataflash);
    void handle_log_message(mavlink_message_t *msg, DataFlash_Class &dataflash);
    void handle_log_send(DataFlash_Class &dataflash);
    void handle_log_send_listing(DataFlash_Class &dataflash);
    bool handle_log_send_data(DataFlash_Class &dataflash);

    void handle_mission_request_list(AP_Mission &mission, mavlink_message_t *msg);
    void handle_mission_request(AP_Mission &mission, mavlink_message_t *msg);

    void handle_mission_set_current(AP_Mission &mission, mavlink_message_t *msg);
    void handle_mission_count(AP_Mission &mission, mavlink_message_t *msg);
    void handle_mission_clear_all(AP_Mission &mission, mavlink_message_t *msg);
    void handle_mission_write_partial_list(AP_Mission &mission, mavlink_message_t *msg);
    void handle_mission_item(mavlink_message_t *msg, AP_Mission &mission);

    void handle_request_data_stream(mavlink_message_t *msg, bool save);
    void handle_param_request_list(mavlink_message_t *msg);
    void handle_param_request_read(mavlink_message_t *msg);
    void handle_param_set(mavlink_message_t *msg, DataFlash_Class *DataFlash);
    void handle_radio_status(mavlink_message_t *msg, DataFlash_Class &dataflash, bool log_radio);
    void handle_serial_control(mavlink_message_t *msg, AP_GPS &gps);
    void lock_channel(mavlink_channel_t chan, bool lock);
    void handle_set_mode(mavlink_message_t* msg, bool (*set_mode)(uint8_t mode));

    // return true if this channel has hardware flow control
    bool have_flow_control(void);
};

#endif // __GCS_H
