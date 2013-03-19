// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	GCS_MAVLink.cpp

/*
This provides some support code and variables for MAVLink enabled sketches

This firmware is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.
*/

#include <AP_HAL.h>
#include <AP_Common.h>
#include <GCS_MAVLink.h>

#include <math.h>		// needed for isnan, isinf
#include <AP_Math.h>	// needed for constrain_
#include <limits.h> 	// needed for SCHAR_MIN, SCHAR_MAX, SHRT_MIN, SHRT_MAX


#ifdef MAVLINK_SEPARATE_HELPERS
#include "include/mavlink/v1.0/mavlink_helpers.h"
#endif


AP_HAL::BetterStream	*mavlink_comm_0_port;
AP_HAL::BetterStream	*mavlink_comm_1_port;

mavlink_system_t mavlink_system = {7,1,0,0};

uint8_t mavlink_check_target(uint8_t sysid, uint8_t compid)
{
    if (sysid != mavlink_system.sysid)
        return 1;
    // Currently we are not checking for correct compid since APM is not passing mavlink info to any subsystem
    // If it is addressed to our system ID we assume it is for us
    return 0; // no error
}

mavlink_param_union_t store_AP_Param_in_float(const AP_Param *ptr, enum ap_var_type type)
{
     mavlink_param_union_t result;
     switch (type) {

     	// signed integers
     	case AP_PARAM_INT8:
     		result.param_int32 = static_cast<const AP_Int8*>(ptr)->get();
     		result.type = MAV_PARAM_TYPE_INT32;
     		break;
     	case AP_PARAM_INT16:
     		result.param_int32 = static_cast<const AP_Int16*>(ptr)->get();
     		result.type = MAV_PARAM_TYPE_INT32;
     		break;
     	case AP_PARAM_INT32:
     		result.param_int32 = static_cast<const AP_Int32*>(ptr)->get();
     		result.type = MAV_PARAM_TYPE_INT32;
     		break;

     	// float
     	case AP_PARAM_FLOAT:
     		result.param_float = static_cast<const AP_Float*>(ptr)->get();
     		result.type = MAV_PARAM_TYPE_REAL32;
     		break;

     	// other parameters are treated as float
     	default:
		// const_cast is needed because AP_Param::cast_to_float isn't const-correct
     		result.param_float = const_cast<AP_Param*>(ptr)->cast_to_float(type);
     		result.type = MAV_PARAM_TYPE_REAL32;
     		break;
     }
     return result;
}

bool update_AP_Param(AP_Param *ptr, enum ap_var_type ptr_type, float new_val, enum MAV_PARAM_TYPE new_val_type)
{

	// handle the three valid param_value types (uint32_t, int32_t, float) as described in the mavlink parameter protocol "documentation":
	// http://qgroundcontrol.org/mavlink/parameter_protocol#supported_data_types
	switch (new_val_type) {

		// since all integer AP-Params are signed, unsigned parameters are ignored
		case MAV_PARAM_TYPE_UINT32:
			return false;

		// signed int: must be converted to int8/16/32 according to var_type
		case MAV_PARAM_TYPE_INT32: {

			mavlink_param_union_t value;
			value.param_float = new_val;

			switch (ptr_type) {
				case AP_PARAM_INT8 : {
					static_cast<AP_Int8*>(ptr)->set_and_save( constrain<int32_t>(value.param_int32, SCHAR_MIN, SCHAR_MAX) );
					return true;
				}
				case AP_PARAM_INT16 : {
					static_cast<AP_Int16*>(ptr)->set_and_save( constrain<int32_t>(value.param_int32, SHRT_MIN, SHRT_MAX) );
					return true;
				}
				case AP_PARAM_INT32 : {
					static_cast<AP_Int32*>(ptr)->set_and_save( value.param_int32 );
					return true;
				}
				// *vp isn't an integer --> discard the received integer parameter
				default:
					return false;
			}
		} // END case MAV_PARAM_TYPE_INT32

		case MAV_PARAM_TYPE_REAL32: {
			if ( !isnan(new_val) && !isinf(new_val) ) {
				static_cast<AP_Float*>(ptr)->set_and_save( new_val );
				return true;
			}
			return false;
		}

		// invalid packet.param_type
		default:
			return false;
	}
}

/*
  send a buffer out a MAVLink channel
 */
void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint8_t len)
{
    switch(chan) {
	case MAVLINK_COMM_0:
		mavlink_comm_0_port->write(buf, len);
		break;
	case MAVLINK_COMM_1:
		mavlink_comm_1_port->write(buf, len);
		break;
	default:
		break;
	}
}

static const uint8_t mavlink_message_crc_progmem[256] PROGMEM = MAVLINK_MESSAGE_CRCS;

// return CRC byte for a mavlink message ID
uint8_t mavlink_get_message_crc(uint8_t msgid)
{
	return pgm_read_byte(&mavlink_message_crc_progmem[msgid]);
}
