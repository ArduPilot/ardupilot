/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef APM_OBC_H
#define APM_OBC_H
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
/*
  Outback Challenge Failsafe module

  Andrew Tridgell and CanberraUAV, August 2012
*/

#include <AP_Common.h>
#include <AP_Param.h>
#include <AP_Mission.h>
#include <AP_Baro.h>
#include <AP_GPS.h>
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
    APM_OBC(AP_Mission &_mission, AP_Baro &_baro, const AP_GPS &_gps) :
        mission(_mission),
        baro(_baro),
        gps(_gps)
        {
            AP_Param::setup_object_defaults(this, var_info);
            
            _last_heartbeat_pin = -1;
            _last_manual_pin = -1;
            _state = STATE_PREFLIGHT;
            _terminate.set(0);
            
            _saved_wp = 0;
        }

	void check(enum control_mode control_mode, uint32_t last_heartbeat_ms);

	// should we crash the plane? Only possible with
	// FS_TERM_ACTTION set to 42
	bool crash_plane(void) {
		return _terminate && _terminate_action == 42;
	}

	// for holding parameters
	static const struct AP_Param::GroupInfo var_info[];

private:
	enum state _state;

    AP_Mission &mission;
    AP_Baro &baro;
    const AP_GPS &gps;

	// digital output pins for communicating with the failsafe board
	AP_Int8 _heartbeat_pin;
	AP_Int8 _manual_pin;
	AP_Int8 _terminate_pin;
	AP_Int8 _terminate;
	AP_Int8 _terminate_action;

	// last pins to cope with changing at runtime
	int8_t _last_heartbeat_pin;
	int8_t _last_manual_pin;
	int8_t _last_terminate_pin;

	// waypoint numbers to jump to on failsafe conditions
	AP_Int8 _wp_comms_hold;
	AP_Int8 _wp_gps_loss;

    AP_Float _qnh_pressure;
    AP_Int32 _amsl_limit;
    AP_Int32 _amsl_margin_gps;

	bool _heartbeat_pin_value;

	// saved waypoint for resuming mission
	uint8_t _saved_wp;

	bool check_altlimit(void);
};

// map from ArduPlane control_mode to APM_OBC::control_mode
#define OBC_MODE(control_mode) ((control_mode==AUTO?APM_OBC::OBC_AUTO:control_mode==MANUAL?APM_OBC::OBC_MANUAL:APM_OBC::OBC_FBW))

#endif // APM_OBC_H
