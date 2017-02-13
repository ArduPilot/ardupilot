#include <AP_HAL/AP_HAL.h>
#include "AP_RCMixer.h"
#include <GCS_MAVLink/GCS.h>

extern AP_HAL::HAL& hal;

// object constructor.
AP_RCMixer::AP_RCMixer(void)
{
    _mixer_data.group = 0;;
    _mixer_data.mixer = 0;
    _mixer_data.submixer = 0;
    _mixer_data.parameter = 0;
    _mixer_data.type = 0;
	_mixer_data.int_value = -1;
	_mixer_data.param_value = 0.0;

	_status = AP_RCMIXER_STATUS_WAITING;
}

void AP_RCMixer::handle_msg(const mavlink_message_t *msg){
#if defined(MIXER_CONFIGURATION)
    switch (msg->msgid) {
		case MAVLINK_MSG_ID_MIXER_DATA_REQUEST:
		    mavlink_mixer_data_request_t packet;
		    mavlink_msg_mixer_data_request_decode(msg, &packet);

		    _target_system = packet.target_system;
			_target_component = packet.target_component;
		    _mixer_data.group = packet.mixer_group;
		    _mixer_data.mixer = packet.mixer_index;
		    _mixer_data.submixer = packet.mixer_sub_index;
		    _mixer_data.parameter = packet.parameter_index;
		    _mixer_data.type = packet.data_type;

		    // Make sure we are talking to px4io only
		    if(packet.mixer_group != 1){
		    	_mixer_data.int_value = -1;
				_mixer_data.param_value = 0.0;
				_status = AP_RCMIXER_STATUS_SEND_DATA;
				return;
	    	}

			switch(packet.data_type){
			case MIXER_DATA_TYPE_MIXER_COUNT:
				_mixer_data.int_value = hal.rcout->get_mixer_count();
				_mixer_data.param_value = 0.0;
				break;
			case MIXER_DATA_TYPE_SUBMIXER_COUNT:
				_mixer_data.int_value = hal.rcout->get_submixer_count(_mixer_data.mixer);
				_mixer_data.param_value = 0.0;
				break;
			default:
				break;
			}
			_status = AP_RCMIXER_STATUS_SEND_DATA;
			break;

		case MAVLINK_MSG_ID_MIXER_PARAMETER_SET:
			break;
		default:
			break;
	}
#endif 	//MIXER_CONFIGURATION
}

//MAVLink mixer data send
void AP_RCMixer::send_mixer_data(mavlink_channel_t chan){
//	GCS_MAVLINK::send_statustext_chan(MAV_SEVERITY_INFO, chan, "send_mixer_data");
	switch(int(_status)){
	case AP_RCMIXER_STATUS_SEND_DATA:
		mavlink_msg_mixer_data_send(chan,
						(uint8_t) _mixer_data.group,
						(uint8_t) _mixer_data.mixer,
						(uint8_t) _mixer_data.submixer,
						(uint8_t) _mixer_data.parameter,
						(uint8_t) _mixer_data.type,
						_mixer_data.int_value,
						_mixer_data.param_value,
						MAVLINK_TYPE_FLOAT
						);
		_status = AP_RCMIXER_STATUS_WAITING;
		break;
	default:
		break;
	}
}


