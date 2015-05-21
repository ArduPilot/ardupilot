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
#include "../AP_SerialManager/AP_SerialManager.h"
#include "../AP_Mount/AP_Mount.h"
#include "GCS_send.h"


///
/// @class	GCS_MAVLINK
/// @brief	MAVLink transport control class
///
class GCS_MAVLINK : public GCS_send
{
public:
    GCS_MAVLINK();
    static const struct AP_Param::GroupInfo var_info[];
    void update(void (*run_cli)(AP_HAL::UARTDriver *));
    
public /*vehicle functions yet not in library*/: 
    // see if we should send a stream now. Called at 50Hz
    virtual bool stream_trigger(enum streams stream_num);
    virtual bool try_send_message(enum ap_message id);
    virtual void handleMessage(mavlink_message_t * msg);
    // vehicle specific message send function
    virtual void data_stream_send();
    
private:
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
    bool handle_mission_item(mavlink_message_t *msg, AP_Mission &mission);

    void handle_request_data_stream(mavlink_message_t *msg, bool save);
    void handle_param_request_list(mavlink_message_t *msg);
    void handle_param_request_read(mavlink_message_t *msg);
    void handle_param_set(mavlink_message_t *msg, DataFlash_Class *DataFlash);
    void handle_radio_status(mavlink_message_t *msg, DataFlash_Class &dataflash, bool log_radio);
    void handle_serial_control(mavlink_message_t *msg, AP_GPS &gps);

    void handle_set_mode(mavlink_message_t* msg, bool (*set_mode)(uint8_t mode));
    void handle_gimbal_report(AP_Mount &mount, mavlink_message_t *msg) const;

    void handle_gps_inject(const mavlink_message_t *msg, AP_GPS &gps);
};

#endif // __GCS_H
