/*
 * gcs_mavlink_stub.h
 *
 *  Created on: 29 mai 2014
 *      Author: valentin
 */

#ifndef GCS_MAVLINK_STUB_H_
#define GCS_MAVLINK_STUB_H_

#include "ap_int16_stub.h"
#include "ap_mission_stub.h"
#include "ap_gps_stub.h"
#include "ap_hal_stub.h"
#include "uartdriver_stub.h"

#define MAVLINK_COMM_NUM_BUFFERS 3

class GCS_MAVLINK
{

public:
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
    uint32_t        last_heartbeat_time;
	mavlink_statustext_t pending_status;
	bool            initialised;
    void send_power_status(void) {}
    void send_meminfo(void) {}
    void queued_param_send(void) {}
    void queued_waypoint_send(void) {}
    void update(void);
    bool stream_trigger(enum streams stream_num);
    void data_stream_send(void);
    void send_text_P(gcs_severity severity, const prog_char_t *str) {}
    void send_message(enum ap_message id) {}
    void handleMessage(mavlink_message_t * msg);
    void send_gps_raw(AP_GPS &gps) {};
    void send_system_time(AP_GPS &gps) {};
    void send_radio_in(uint8_t receiver_rssi) {};
    void update(void (*run_cli)(UARTDriver *)) {};
private:
    AP_Param *                  _queued_parameter;
	mavlink_channel_t           chan;
    bool try_send_message(enum ap_message id);
    static const struct AP_Param::GroupInfo        var_info[];
    void handle_guided_request(AP_Mission::Mission_Command &cmd);
    void handle_change_alt_request(AP_Mission::Mission_Command &cmd);
    void handle_log_send(DataFlash_Class &dataflash) {}
    void handle_param_request_list(mavlink_message_t *msg) {}
    void handle_param_request_read(mavlink_message_t *msg) {}
    void handle_param_set(mavlink_message_t *msg, DataFlash_Class *DataFlash) {}
    void handle_mission_write_partial_list(AP_Mission &mission, mavlink_message_t *msg) {}
    void handle_mission_item(mavlink_message_t *msg, AP_Mission &mission) {}
    void handle_mission_request(AP_Mission &mission, mavlink_message_t *msg) {}
    void handle_mission_set_current(AP_Mission &mission, mavlink_message_t *msg) {}
    void handle_mission_request_list(AP_Mission &mission, mavlink_message_t *msg) {}
    void handle_mission_count(AP_Mission &mission, mavlink_message_t *msg) {}
    void handle_mission_clear_all(AP_Mission &mission, mavlink_message_t *msg) {}
    void handle_request_data_stream(mavlink_message_t *msg, bool save) {}
    void handle_radio_status(mavlink_message_t *msg, DataFlash_Class &dataflash, bool log_radio) {}
    void handle_log_message(mavlink_message_t *msg, DataFlash_Class &dataflash) {}

    bool            waypoint_receiving; // currently receiving
    uint16_t        waypoint_request_i; // request index
    uint16_t        waypoint_request_last; // last request index
    uint32_t        waypoint_timelast_request; // milliseconds
    uint32_t        waypoint_timelast_receive; // milliseconds
    uint16_t        waypoint_receive_timeout; // milliseconds
    uint8_t         stream_slowdown;
    AP_Int16        streamRates[NUM_STREAMS];
    uint8_t         stream_ticks[NUM_STREAMS];
};//GCS_MAVLINK


#endif /* GCS_MAVLINK_STUB_H_ */
