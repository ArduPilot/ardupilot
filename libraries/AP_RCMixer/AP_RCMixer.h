#pragma once

#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

struct mixer_data_s{
	uint16_t group;
	uint16_t mixer;
	uint16_t submixer;
	uint16_t parameter;
	uint16_t type;
	float 	 param_value;
	uint16_t param_type;
	int32_t  int_value;
};

typedef enum
{
	AP_RCMIXER_STATUS_WAITING 	= 0,
	AP_RCMIXER_STATUS_SEND_DATA = 1,
} AP_RCMIXER_STATUS;

class AP_RCMixer
{
public:
    /// Constructor
    ///
	AP_RCMixer();
    // Pass mavlink data to message handlers (for MAV type)
    void handle_msg(const mavlink_message_t *msg);

    //MAVLink mixer data send
    void send_mixer_data(mavlink_channel_t chan);

    // Return true if waiting to send data
    bool send_queued(void) {return(_status == AP_RCMIXER_STATUS_SEND_DATA);}

//    static int mavlink_to_mission_cmd(const mavlink_mission_item_t& packet, AP_Mission::Mission_Command& cmd);
//    static int mavlink_int_to_mission_cmd(const mavlink_mission_item_int_t& packet, AP_Mission::Mission_Command& cmd);
//
//    // mission_cmd_to_mavlink - converts an AP_Mission::Mission_Command object to a mavlink message which can be sent to the GCS
//    //  return true on success, false on failure
//    static bool mission_cmd_to_mavlink(const AP_Mission::Mission_Command& cmd, mavlink_mission_item_t& packet);
//    static bool mission_cmd_to_mavlink_int(const AP_Mission::Mission_Command& cmd, mavlink_mission_item_int_t& packet);
//    AP_GPS::send_mavlink_gps_raw(mavlink_channel_t chan)

private:
    mixer_data_s _mixer_data;
    AP_RCMIXER_STATUS _status;
};
