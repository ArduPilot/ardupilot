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

#include "AP_RPM.h"

#if AP_RPM_ENABLED

#include "RPM_Backend.h"
#include "RPM_Pin.h"
#include "RPM_SITL.h"
#include "RPM_EFI.h"
#include "RPM_Generator.h"
#include "RPM_HarmonicNotch.h"
#include "RPM_ESC_Telem.h"
#include "RPM_DroneCAN.h"

#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_RPM::var_info[] = {
    // 0-13 used by old param indexes before being moved to AP_RPM_Params

    // @Group: 1_
    // @Path: AP_RPM_Params.cpp
    AP_SUBGROUPINFO(_params[0], "1_", 14, AP_RPM, AP_RPM_Params),

#if RPM_MAX_INSTANCES > 1
    // @Group: 2_
    // @Path: AP_RPM_Params.cpp
    AP_SUBGROUPINFO(_params[1], "2_", 15, AP_RPM, AP_RPM_Params),
#endif

#if RPM_MAX_INSTANCES > 2
    // @Group: 3_
    // @Path: AP_RPM_Params.cpp
    AP_SUBGROUPINFO(_params[2], "3_", 16, AP_RPM, AP_RPM_Params),
#endif

#if RPM_MAX_INSTANCES > 3
    // @Group: 4_
    // @Path: AP_RPM_Params.cpp
    AP_SUBGROUPINFO(_params[3], "4_", 17, AP_RPM, AP_RPM_Params),
#endif

    AP_GROUPEND
};

AP_RPM::AP_RPM(void)
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_RPM must be singleton");
    }
    _singleton = this;
}

/*
  initialise the AP_RPM class.
 */
void AP_RPM::init(void)
{
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }

    for (uint8_t i=0; i<RPM_MAX_INSTANCES; i++) {
        switch (_params[i].type) {
#if AP_RPM_PIN_ENABLED
        case RPM_TYPE_PWM:
        case RPM_TYPE_PIN:
            // PWM option same as PIN option, for upgrade
            drivers[i] = NEW_NOTHROW AP_RPM_Pin(*this, i, state[i]);
            break;
#endif  // AP_RPM_PIN_ENABLED
#if AP_RPM_ESC_TELEM_ENABLED
        case RPM_TYPE_ESC_TELEM:
            drivers[i] = NEW_NOTHROW AP_RPM_ESC_Telem(*this, i, state[i]);
            break;
#endif  // AP_RPM_ESC_TELEM_ENABLED
#if AP_RPM_EFI_ENABLED
        case RPM_TYPE_EFI:
            drivers[i] = NEW_NOTHROW AP_RPM_EFI(*this, i, state[i]);
            break;
#endif  // AP_RPM_EFI_ENABLED
#if AP_RPM_GENERATOR_ENABLED
        case RPM_TYPE_GENERATOR:
            drivers[i] = NEW_NOTHROW AP_RPM_Generator(*this, i, state[i]);
            break;
#endif  // AP_RPM_GENERATOR_ENABLED
#if AP_RPM_HARMONICNOTCH_ENABLED
        // include harmonic notch last
        // this makes whatever process is driving the dynamic notch appear as an RPM value
        case RPM_TYPE_HNTCH:
            drivers[i] = NEW_NOTHROW AP_RPM_HarmonicNotch(*this, i, state[i]);
            break;
#endif  // AP_RPM_HARMONICNOTCH_ENABLED
#if AP_RPM_DRONECAN_ENABLED
        case RPM_TYPE_DRONECAN:
            drivers[i] = NEW_NOTHROW AP_RPM_DroneCAN(*this, i, state[i]);
            break;
#endif // AP_RPM_DRONECAN_ENABLED
#if AP_RPM_SIM_ENABLED
        case RPM_TYPE_SITL:
            drivers[i] = NEW_NOTHROW AP_RPM_SITL(*this, i, state[i]);
            break;
#endif  // AP_RPM_SIM_ENABLED
        }
        if (drivers[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1; // num_instances is a high-water-mark
        }
    }
}

/*
  update RPM state for all instances. This should be called by main loop
 */
void AP_RPM::update(void)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != nullptr) {
            if (_params[i].type == RPM_TYPE_NONE) {
                // allow user to disable an RPM sensor at runtime and force it to re-learn the quality if re-enabled.
                state[i].signal_quality = 0;
                continue;
            }

            drivers[i]->update();

#if AP_RPM_ESC_TELEM_OUTBOUND_ENABLED
            drivers[i]->update_esc_telem_outbound();
#endif
        }
    }

#if HAL_LOGGING_ENABLED
    Log_RPM();
#endif
}

/*
  check if an instance is healthy
 */
bool AP_RPM::healthy(uint8_t instance) const
{
    if (instance >= num_instances || _params[instance].type == RPM_TYPE_NONE) {
        return false;
    }

    // check that data quality is above minimum required
    if (state[instance].signal_quality < _params[instance].quality_min) {
        return false;
    }

    return true;
}

/*
  check if an instance is activated
 */
bool AP_RPM::enabled(uint8_t instance) const
{
    if (instance >= num_instances) {
        return false;
    }
    // if no sensor type is selected, the sensor is not activated.
    if (_params[instance].type == RPM_TYPE_NONE) {
        return false;
    }
    return true;
}

/*
  get RPM value, return true on success
 */
bool AP_RPM::get_rpm(uint8_t instance, float &rpm_value) const
{
    if (!healthy(instance)) {
        return false;
    }
    rpm_value = state[instance].rate_rpm;
    return true;
}

// check settings are valid
bool AP_RPM::arming_checks(size_t buflen, char *buffer) const
{
    for (uint8_t i=0; i<RPM_MAX_INSTANCES; i++) {
        switch (_params[i].type) {
#if AP_RPM_PIN_ENABLED
        case RPM_TYPE_PWM:
        case RPM_TYPE_PIN:
            if (_params[i].pin == -1) {
                hal.util->snprintf(buffer, buflen, "RPM%u_PIN not set", unsigned(i + 1));
                return false;
            }
            if (!hal.gpio->valid_pin(_params[i].pin)) {
                uint8_t servo_ch;
                if (hal.gpio->pin_to_servo_channel(_params[i].pin, servo_ch)) {
                    hal.util->snprintf(buffer, buflen, "RPM%u_PIN=%d, set SERVO%u_FUNCTION=-1", unsigned(i + 1), int(_params[i].pin.get()), unsigned(servo_ch+1));
                } else {
                    hal.util->snprintf(buffer, buflen, "RPM%u_PIN=%d invalid", unsigned(i + 1), int(_params[i].pin.get()));
                }
                return false;
            }
            break;
#endif
        }
    }
    return true;
}

#if HAL_LOGGING_ENABLED
void AP_RPM::Log_RPM() const
{
    // update logging for each instance
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] == nullptr || !enabled(i)) {
            // don't log unused instances
            continue;
        }

        const struct log_RPM pkt{
            LOG_PACKET_HEADER_INIT(LOG_RPM_MSG),
            time_us     : AP_HAL::micros64(),
            inst        : i,
            rpm         : state[i].rate_rpm,
            quality     : get_signal_quality(i),
            health      : uint8_t(healthy(i))
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }
}
#endif

#if AP_RPM_STREAM_ENABLED
// Return the sensor id to use for streaming over DroneCAN, negative number disables
int8_t AP_RPM::get_dronecan_sensor_id(uint8_t instance) const
{
    if (!enabled(instance)) {
        return -1;
    }
    return _params[instance].dronecan_sensor_id;
}
#endif


// singleton instance
AP_RPM *AP_RPM::_singleton;

namespace AP {

AP_RPM *rpm()
{
    return AP_RPM::get_singleton();
}

}

#endif  // AP_RPM_ENABLED
