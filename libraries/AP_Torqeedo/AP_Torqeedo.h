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
   This driver supports communicating with Torqeedo motors that implement the "TQ Bus" protocol
*/

#pragma once

#include "AP_Torqeedo_config.h"

#if HAL_TORQEEDO_ENABLED

#include <AP_Param/AP_Param.h>
#include "AP_Torqeedo_Params.h"

#define AP_TORQEEDO_MAX_INSTANCES   2   // maximum number of Torqeedo backends

// declare backend classes
class AP_Torqeedo_Backend;
class AP_Torqeedo_TQBus;

class AP_Torqeedo {

    // declare backends as friends
    friend class AP_Torqeedo_Backend;
    friend class AP_Torqeedo_TQBus;

public:
    AP_Torqeedo();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Torqeedo);

    // get singleton instance
    static AP_Torqeedo* get_singleton();

    // TYPE parameter values
    enum class ConnectionType : uint8_t {
        TYPE_DISABLED = 0,
        TYPE_TILLER = 1,
        TYPE_MOTOR = 2
    };

    // OPTIONS parameter values
    enum class options {
        LOG             = 1<<0,
        DEBUG_TO_GCS    = 1<<1,
    };

    // initialise driver
    void init();

    // returns true if at least one backend has been configured (e.g. TYPE param has been set)
    bool enabled() const;
    bool enabled(uint8_t instance) const;

    // returns true if all backends are communicating with the motor
    bool healthy();
    bool healthy(uint8_t instance);

    // run pre-arm check.  returns false on failure and fills in failure_msg
    // any failure_msg returned will not include a prefix
    bool pre_arm_checks(char *failure_msg, uint8_t failure_msg_len);

    // clear motor errors
    void clear_motor_error();

    // get latest battery status info.  returns true on success and populates arguments
    // instance is normally 0 or 1, if invalid instances are provided the first instance is used
    bool get_batt_info(uint8_t instance, float &voltage, float &current_amps, float &temp_C, uint8_t &pct_remaining) const WARN_IF_UNUSED;
    bool get_batt_capacity_Ah(uint8_t instance, uint16_t &amp_hours) const;

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

    // parameters for backends
    AP_Torqeedo_Params _params[AP_TORQEEDO_MAX_INSTANCES];

private:

    // return pointer to backend given an instance number
    AP_Torqeedo_Backend *get_instance(uint8_t instance) const;

    static AP_Torqeedo *_singleton;
    AP_Torqeedo_Backend *_backends[AP_TORQEEDO_MAX_INSTANCES];  // pointers to instantiated backends
};

namespace AP {
    AP_Torqeedo *torqeedo();
};

#endif // HAL_TORQEEDO_ENABLED
