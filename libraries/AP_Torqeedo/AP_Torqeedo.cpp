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

#include "AP_Torqeedo.h"

#if HAL_TORQEEDO_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_Torqeedo_Backend.h"
#include "AP_Torqeedo_TQBus.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Torqeedo::var_info[] = {

    // 1 to 7 were used for parameters before frontend/backend split

    // @Group: 1_
    // @Path: AP_Torqeedo_Params.cpp
    AP_SUBGROUPINFO(_params[0], "1_", 8, AP_Torqeedo, AP_Torqeedo_Params),

#if AP_TORQEEDO_MAX_INSTANCES > 1
    // @Group: 2_
    // @Path: AP_Torqeedo_Params.cpp
    AP_SUBGROUPINFO(_params[1], "2_", 9, AP_Torqeedo, AP_Torqeedo_Params),
#endif

    AP_GROUPEND
};

AP_Torqeedo::AP_Torqeedo()
{
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// initialise driver
void AP_Torqeedo::init()
{
    // check init has not been called before
    for (uint8_t i = 1; i < AP_TORQEEDO_MAX_INSTANCES; i++) {
        if (get_instance(i) != nullptr) {
            return;
        }
    }

    // create each instance
    uint8_t instance;
    for (instance = 0; instance < AP_TORQEEDO_MAX_INSTANCES; instance++) {
        switch ((ConnectionType)_params[instance].type.get()) {
        case ConnectionType::TYPE_DISABLED:
            // do nothing
            break;
        case ConnectionType::TYPE_TILLER:
        case ConnectionType::TYPE_MOTOR:
            _backends[instance] = NEW_NOTHROW AP_Torqeedo_TQBus(_params[instance], instance);
            break;
        }
    }

    // init each instance, do it after all instances were created, so that they all know things
    for (instance = 0; instance < AP_TORQEEDO_MAX_INSTANCES; instance++) {
        if (_backends[instance] != nullptr) {
            _backends[instance]->init();
        }
    }
}

// returns true if at least one backend has been configured (e.g. TYPE param has been set)
bool AP_Torqeedo::enabled() const
{
    for (uint8_t instance = 0; instance < AP_TORQEEDO_MAX_INSTANCES; instance++) {
        if (enabled(instance)) {
            return true;
        }
    }
    return false;
}

// returns true if the instance has been configured (e.g. TYPE param has been set)
bool AP_Torqeedo::enabled(uint8_t instance) const
{
    if (instance < AP_TORQEEDO_MAX_INSTANCES) {
        switch ((ConnectionType)_params[instance].type.get()) {
        case ConnectionType::TYPE_DISABLED:
            return false;
        case ConnectionType::TYPE_TILLER:
        case ConnectionType::TYPE_MOTOR:
            return true;
        }
    }
    return false;
}

// returns true if all backends are communicating with the motor
bool AP_Torqeedo::healthy()
{
    uint8_t num_backends = 0;
    uint8_t num_healthy = 0;
    for (uint8_t instance = 0; instance < AP_TORQEEDO_MAX_INSTANCES; instance++) {
        auto *backend = get_instance(instance);
        if (backend != nullptr) {
            num_backends++;
            if (backend->healthy()) {
                num_healthy++;
            }
        }
    }

    return ((num_backends > 0) && (num_healthy == num_backends));
}

// returns true if instance is healthy
bool AP_Torqeedo::healthy(uint8_t instance)
{
    auto *backend = get_instance(instance);
    if (backend == nullptr) {
        return false;
    }
    return backend->healthy();
}

// run pre-arm check.  returns false on failure and fills in failure_msg
// any failure_msg returned will not include a prefix
bool AP_Torqeedo::pre_arm_checks(char *failure_msg, uint8_t failure_msg_len)
{
    // exit immediately if not enabled
    if (!enabled()) {
        return true;
    }

    if (!healthy()) {
        strncpy(failure_msg, "not healthy", failure_msg_len);
        return false;
    }
    return true;
}

// clear motor errors
void AP_Torqeedo::clear_motor_error()
{
<<<<<<< HEAD
    switch (code) {
    case 2:
        return "stator high temp";
    case 5:
        return "propeller blocked";
    case 6:
        return "motor low voltage";
    case 7:
        return "motor high current";
    case 8:
        return "pcb temp high";
    case 21:
        return "tiller cal bad";
    case 22:
        return "mag bad";
    case 23:
        return "range incorrect";
    case 30:
        return "motor comm error";
    case 32:
        return "tiller comm error";
    case 33:
        return "general comm error";
    case 41:
    case 42:
        return "charge voltage bad";
    case 43:
        return "battery flat";
    case 45:
        return "battery high current";
    case 46:
        return "battery temp error";
    case 48:
        return "charging temp error";
    }

    return nullptr;
}

// report changes in error codes to user
void AP_Torqeedo::report_error_codes()
{
    // skip reporting if we have already reported status very recently
    const uint32_t now_ms = AP_HAL::millis();

    // skip reporting if no changes in flags and already reported within 10 seconds
    const bool flags_changed = (_display_system_state_flags_prev.value != _display_system_state.flags.value) ||
                               (_display_system_state_master_error_code_prev != _display_system_state.master_error_code) ||
                               (_motor_status_prev.status_flags_value != _motor_status.status_flags_value) ||
                               (_motor_status_prev.error_flags_value != _motor_status.error_flags_value);
    if (!flags_changed && ((now_ms - _last_error_report_ms) < TORQEEDO_ERROR_REPORT_INTERVAL_MAX_MS)) {
        return;
    }

    // report display system errors
    const char* msg_prefix = "Torqeedo:";
    if (_display_system_state.flags.set_throttle_stop) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "%s zero throttle required", msg_prefix);
    }
    if (_display_system_state.flags.temp_warning) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "%s high temp", msg_prefix);
    }
    if (_display_system_state.flags.batt_nearly_empty) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "%s batt nearly empty", msg_prefix);
    }
    if (_display_system_state.master_error_code > 0) {
        const char *error_string = map_master_error_code_to_string(_display_system_state.master_error_code);
        if (error_string != nullptr) {
            gcs().send_text(
                MAV_SEVERITY_CRITICAL, "%s err:%u %s",
                msg_prefix,
                _display_system_state.master_error_code,
                error_string);
        } else {
            gcs().send_text(
                MAV_SEVERITY_CRITICAL, "%s err:%u",
                msg_prefix,
                _display_system_state.master_error_code);
=======
    for (uint8_t instance = 0; instance < AP_TORQEEDO_MAX_INSTANCES; instance++) {
        auto *backend = get_instance(instance);
        if (backend != nullptr) {
            backend->clear_motor_error();
>>>>>>> 7f04c82994d82ad0004f50e47e458c63c291dd86
        }
    }
}

// get latest battery status info.  returns true on success and populates arguments
// instance is normally 0 or 1, if invalid instances are provided the first instance is used
bool AP_Torqeedo::get_batt_info(uint8_t instance, float &voltage, float &current_amps, float &temp_C, uint8_t &pct_remaining) const
{
    // for invalid instances use the first instance
    if (instance > AP_TORQEEDO_MAX_INSTANCES-1) {
        instance = 0;
    }

    // return battery info for specified instance
    auto *backend = get_instance(instance);
    if (backend != nullptr) {
        return backend->get_batt_info(voltage, current_amps, temp_C, pct_remaining);
    }
    return false;
}

// get battery capacity.  returns true on success and populates argument
// instance is normally 0 or 1, if invalid instances are provided the first instance is used
bool AP_Torqeedo::get_batt_capacity_Ah(uint8_t instance, uint16_t &amp_hours) const
{
    // for invalid instances use the first instance
    if (instance > AP_TORQEEDO_MAX_INSTANCES-1) {
        instance = 0;
    }

    // return battery capacity from specified instance
    auto *backend = get_instance(instance);
    if (backend != nullptr) {
        return backend->get_batt_capacity_Ah(amp_hours);
    }
    return false;
}

// return pointer to backend given an instance number
AP_Torqeedo_Backend *AP_Torqeedo::get_instance(uint8_t instance) const
{
    if (instance < AP_TORQEEDO_MAX_INSTANCES) {
        return _backends[instance];
    }
    return nullptr;
}

// get the AP_Torqeedo singleton
AP_Torqeedo *AP_Torqeedo::get_singleton()
{
    return _singleton;
}

AP_Torqeedo *AP_Torqeedo::_singleton = nullptr;

namespace AP {

AP_Torqeedo *torqeedo()
{
    return AP_Torqeedo::get_singleton();
}
}

#endif // HAL_TORQEEDO_ENABLED
