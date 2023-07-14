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
#pragma once

#include "AP_Torqeedo_config.h"

#if HAL_TORQEEDO_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#include "AP_Torqeedo_Params.h"

// Maximum number of range finder instances available on this platform
#define TORQEEDO_MAX_INSTANCES 3

// first monitor is always the primary monitor
#define TORQEEDO_PRIMARY_INSTANCE  0

class AP_Torqeedo_Backend;

class AP_Torqeedo 
{
    friend class AP_Torqeedo_Backend;

public:
    AP_Torqeedo();

    // do not allow copies
    CLASS_NO_COPY(AP_Torqeedo);

    // Torqeedo driver types
    enum class Type {
        None = 0,

#if AP_TORQEEDO_TQBUS_ENABLED
        TQBus = 1,
#endif
    };

    enum class Status {
        NotConnected = 0,
        NoData = 1,
        Good
    };

     // The TorqeedoState structure is filled in by the backend driver
    struct Torqeedo_State { 
        uint8_t instance;      // the instance number 
        Status status;         // propellers status
        const struct AP_Param::GroupInfo *var_info;
    };

    static const struct AP_Param::GroupInfo *backend_var_info[TORQEEDO_MAX_INSTANCES];

    // parameters for each instance
    static const struct AP_Param::GroupInfo var_info[];

    // detect and initialise any available propellers
    void init();

    // update state of all propellers. Should be called at high rate from main loop
    void update();

    // return the number of propellers backends
    uint8_t num_propellers() const { return num_instances; }

    // return sensor type of a given instance
    Type get_type(uint8_t instance) const;

    // return sensor health
    Status get_instance_status(uint8_t instance) const;
    Status get_status() const;

    // prearm checks
    bool pre_arm_checks(char *failure_msg, const uint8_t failure_msg_len) const;

    // get latest battery status info.  returns true on success and populates arguments
    bool get_batt_info(float &voltage, float &current_amps, float &temp_C, uint8_t &pct_remaining) const WARN_IF_UNUSED
    { return get_batt_info(TORQEEDO_PRIMARY_INSTANCE, voltage, current_amps, temp_C, pct_remaining); }
    bool get_batt_info(uint8_t instance, float &voltage, float &current_amps, float &temp_C, uint8_t &pct_remaining) const WARN_IF_UNUSED;

    
    bool get_batt_capacity_Ah(uint16_t &amp_hours) const
    { return get_batt_capacity_Ah(TORQEEDO_PRIMARY_INSTANCE, amp_hours); }
    bool get_batt_capacity_Ah(uint8_t instance, uint16_t &amp_hours) const;

    // clear error message
    void clear_motor_error();

    static AP_Torqeedo *get_singleton(void) { return _singleton; };

protected:
    AP_Torqeedo_Params params[TORQEEDO_MAX_INSTANCES];

private:
    static AP_Torqeedo *_singleton;

    Torqeedo_State state[TORQEEDO_MAX_INSTANCES];
    AP_Torqeedo_Backend *drivers[TORQEEDO_MAX_INSTANCES];
    uint8_t num_instances;

    // return true if the given instance exists
    bool valid_instance(uint8_t i) const;

    HAL_Semaphore detect_sem;
};

namespace AP {
    AP_Torqeedo *torqeedo();
};

#endif
