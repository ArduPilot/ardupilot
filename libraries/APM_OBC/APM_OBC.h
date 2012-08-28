#ifndef APM_OBC_H
#define APM_OBC_H
/*
  Outback Challenge Failsafe module

  Andrew Tridgell and CanberraUAV, August 2012

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
*/

#include <AP_Common.h>
#include <inttypes.h>


class APM_OBC
{
public:
	enum control_mode {
		OBC_MANUAL = 0,
		OBC_FBW    = 1,
		OBC_AUTO   = 2
	};

	enum state {
		STATE_PREFLIGHT       = 0,
		STATE_AUTO            = 1,
		STATE_DATA_LINK_LOSS  = 2,
		STATE_GPS_LOSS        = 3
	};

	// Constructor
	APM_OBC(void)
	{
		_last_heartbeat_pin = -1;
		_last_manual_pin = -1;
		_state = STATE_PREFLIGHT;
		_terminate.set(0);

		// get a pointer to COMMAND_INDEX so we can resume a
		// auto mission when a failsafe condition is resolved
		enum ap_var_type var_type;
		_command_index = (AP_Int8 *)AP_Param::find("CMD_INDEX", &var_type);
		_saved_wp = 0;
	}

	void check(enum control_mode control_mode,
		   uint32_t last_heartbeat_ms,
		   uint32_t last_gps_fix_ms);

	// for holding parameters
	static const struct AP_Param::GroupInfo var_info[];

private:
	enum state _state;

	// digital output pins for communicating with the failsafe board
	AP_Int8 _heartbeat_pin;
	AP_Int8 _manual_pin;
	AP_Int8 _terminate;

	// last pins to cope with changing at runtime
	int8_t _last_heartbeat_pin;
	int8_t _last_manual_pin;

	// waypoint numbers to jump to on failsafe conditions
	AP_Int8 _wp_comms_hold;
	AP_Int8 _wp_gps_loss;

	bool _heartbeat_pin_value;

	// pointer to command index parameter in g
	AP_Int8 *_command_index;

	// saved waypoint for resuming mission
	uint8_t _saved_wp;
};

// map from ArduPlane control_mode to APM_OBC::control_mode
#define OBC_MODE(control_mode) ((control_mode==AUTO?APM_OBC::OBC_AUTO:control_mode==MANUAL?APM_OBC::OBC_MANUAL:APM_OBC::OBC_FBW))

#endif // APM_OBC_H
