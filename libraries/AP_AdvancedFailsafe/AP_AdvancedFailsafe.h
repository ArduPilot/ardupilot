#pragma once

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

#include "AP_AdvancedFailsafe_config.h"

#if AP_ADVANCEDFAILSAFE_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <inttypes.h>
#include <AP_Common/Location.h>

class AP_AdvancedFailsafe
{
public:
    enum control_mode {
        AFS_MANUAL     = 0,
        AFS_STABILIZED = 1,
        AFS_AUTO       = 2
    };

    enum state {
        STATE_PREFLIGHT       = 0,
        STATE_AUTO            = 1,
        STATE_DATA_LINK_LOSS  = 2,
        STATE_GPS_LOSS        = 3
    };

    enum terminate_action {
        TERMINATE_ACTION_TERMINATE = 42,
        TERMINATE_ACTION_LAND      = 43
    };

    /* Do not allow copies */
    CLASS_NO_COPY(AP_AdvancedFailsafe);

    // Constructor
    AP_AdvancedFailsafe()
        {
            AP_Param::setup_object_defaults(this, var_info);
            if (_singleton != nullptr) {
                AP_HAL::panic("AP_Logger must be singleton");
            }

            _singleton = this;
            _state = STATE_PREFLIGHT;
            _terminate.set(0);

            _saved_wp = 0;
        }

    // get singleton instance
    static AP_AdvancedFailsafe *get_singleton(void) {
        return _singleton;
    }

    bool enabled() { return _enable; }

    // check that everything is OK
    void check(uint32_t last_valid_rc_ms);

    // generate heartbeat msgs, so external failsafe boards are happy
    // during sensor calibration
    void heartbeat(void);

    // return true if we are terminating (deliberately crashing the vehicle)
    bool should_crash_vehicle(void);

    // enables or disables a GCS based termination, returns true if AFS is in the desired termination state
    bool gcs_terminate(bool should_terminate, const char *reason);

    // called to set all outputs to termination state
    virtual void terminate_vehicle(void) = 0;

    // for holding parameters
    static const struct AP_Param::GroupInfo var_info[];

    bool terminating_vehicle_via_landing() const {
        return _terminate_action == TERMINATE_ACTION_LAND;
    };

protected:
    // setup failsafe values for if FMU firmware stops running
    virtual void setup_IO_failsafe(void) = 0;

    // return the AFS mapped control mode
    virtual enum control_mode afs_mode(void) = 0;

    enum state _state;

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
    AP_Float _rc_fail_time_seconds;
    AP_Int8  _max_gps_loss;
    AP_Int8  _max_comms_loss;
    AP_Int8  _enable_geofence_fs;
    AP_Int8  _enable_RC_fs;
    AP_Int8  _rc_term_manual_only;
    AP_Int8  _enable_dual_loss;
    AP_Int16  _max_range_km;

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

    Location _first_location;
    bool _have_first_location;
    uint32_t _term_range_notice_ms;

    bool check_altlimit(void);

private:
    static AP_AdvancedFailsafe *_singleton;

    // update maximum range check
    void max_range_update();
};

namespace AP {
    AP_AdvancedFailsafe *advancedfailsafe();
};

#endif  // AP_ADVANCEDFAILSAFE_ENABLED
