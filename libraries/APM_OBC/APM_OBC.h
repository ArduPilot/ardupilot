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

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_RCMapper/AP_RCMapper.h>
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
    APM_OBC(AP_Mission &_mission, AP_Baro &_baro, const AP_GPS &_gps, const RCMapper &_rcmap) :
        mission(_mission),
        baro(_baro),
        gps(_gps),
        rcmap(_rcmap),
        _gps_loss_count(0),
        _comms_loss_count(0)
        {
            AP_Param::setup_object_defaults(this, var_info);
            
            _state = STATE_PREFLIGHT;
            _terminate.set(0);
            
            _saved_wp = 0;
        }

    // check that everything is OK
    void check(enum control_mode control_mode, uint32_t last_heartbeat_ms, bool geofence_breached, uint32_t last_valid_rc_ms);

    // generate heartbeat msgs, so external failsafe boards are happy
    // during sensor calibration
    void heartbeat(void);

    // called in servo output code to set servos to crash position if needed
    void check_crash_plane(void);

    // for holding parameters
    static const struct AP_Param::GroupInfo var_info[];

private:
    enum state _state;

    AP_Mission &mission;
    AP_Baro &baro;
    const AP_GPS &gps;
    const RCMapper &rcmap;

    AP_Int8 _enable;
    // digital output pins for communicating with the failsafe board
    AP_Int8 _heartbeat_pin;
    AP_Int8 _manual_pin;
    AP_Int8 _terminate_pin;
    AP_Int8 _terminate;
    AP_Int8 _terminate_action;

    // waypoint numbers to jump to on failsafe conditions
    AP_Int8 _wp_comms_hold;
    AP_Int8 _wp_gps_loss;

    AP_Float _qnh_pressure;
    AP_Int32 _amsl_limit;
    AP_Int32 _amsl_margin_gps;
    AP_Int16 _rc_fail_time;
    AP_Int8  _max_gps_loss;
    AP_Int8  _max_comms_loss;

    bool _heartbeat_pin_value;

    // saved waypoint for resuming mission
    uint8_t _saved_wp;
    
    // number of times we've lost GPS
    uint8_t _gps_loss_count;

    // number of times we've lost data link
    uint8_t _comms_loss_count;

    // last comms loss time
    uint32_t _last_comms_loss_ms;

    // last GPS loss time
    uint32_t _last_gps_loss_ms;

    // have the failsafe values been setup?
    bool _failsafe_setup:1;

    // setup failsafe values for if FMU firmware stops running
    void setup_failsafe(void);

    bool check_altlimit(void);
};

// map from ArduPlane control_mode to APM_OBC::control_mode
#define OBC_MODE(control_mode) (auto_throttle_mode?APM_OBC::OBC_AUTO:(control_mode==MANUAL?APM_OBC::OBC_MANUAL:APM_OBC::OBC_FBW))

#endif // APM_OBC_H
