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

#include "RPM_Pin.h"
#include "RPM_SITL.h"
#include "RPM_EFI.h"
#include "RPM_Generator.h"
#include "RPM_HarmonicNotch.h"
#include "RPM_ESC_Telem.h"

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

    convert_params();

    for (uint8_t i=0; i<RPM_MAX_INSTANCES; i++) {
        switch (_params[i].type) {
#if AP_RPM_PIN_ENABLED
        case RPM_TYPE_PWM:
        case RPM_TYPE_PIN:
            // PWM option same as PIN option, for upgrade
            drivers[i] = new AP_RPM_Pin(*this, i, state[i]);
            break;
#endif  // AP_RPM_PIN_ENABLED
#if AP_RPM_ESC_TELEM_ENABLED
        case RPM_TYPE_ESC_TELEM:
            drivers[i] = new AP_RPM_ESC_Telem(*this, i, state[i]);
            break;
#endif  // AP_RPM_ESC_TELEM_ENABLED
#if AP_RPM_EFI_ENABLED
        case RPM_TYPE_EFI:
            drivers[i] = new AP_RPM_EFI(*this, i, state[i]);
            break;
#endif  // AP_RPM_EFI_ENABLED
#if AP_RPM_GENERATOR_ENABLED
        case RPM_TYPE_GENERATOR:
            drivers[i] = new AP_RPM_Generator(*this, i, state[i]);
            break;
#endif  // AP_RPM_GENERATOR_ENABLED
#if AP_RPM_HARMONICNOTCH_ENABLED
        // include harmonic notch last
        // this makes whatever process is driving the dynamic notch appear as an RPM value
        case RPM_TYPE_HNTCH:
            drivers[i] = new AP_RPM_HarmonicNotch(*this, i, state[i]);
            break;
#endif  // AP_RPM_HARMONICNOTCH_ENABLED
#if AP_RPM_SIM_ENABLED
        case RPM_TYPE_SITL:
            drivers[i] = new AP_RPM_SITL(*this, i, state[i]);
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
PARAMETER_CONVERSION - Added: Aug-2021
*/
void AP_RPM::convert_params(void)
{
    if (_params[0].type.configured()) {
        // _params[0].type will always be configured after conversion is done the first time
        return;
    }

    // don't do conversion if neither RPM types were set
    bool type_set;
    uint8_t rpm_type = 0;
    uint8_t rpm2_type = 0;
    type_set = AP_Param::get_param_by_index(this, 0, AP_PARAM_INT8, &rpm_type);
    type_set |= AP_Param::get_param_by_index(this, 10, AP_PARAM_INT8, &rpm2_type);

    if (!type_set || (rpm_type == 0 && rpm2_type == 0)) {
        return;
    }

    struct ConversionTable {
        uint8_t old_element;
        uint8_t new_index;
        uint8_t instance;
    };

    const struct ConversionTable conversionTable[] = {
            // RPM 1
            {0, 0, 0}, // TYPE
            {1, 1, 0}, // SCALING
            {2, 2, 0}, // MAX
            {3, 3, 0}, // MIN
            {4, 4, 0}, // MIN_QUAL
            {5, 5, 0}, // PIN
            {6, 6, 0}, // ESC_MASK

            // RPM 2
            {10, 0, 1}, // TYPE
            {11, 1, 1}, // SCALING
            // MAX (Previous bug meant RPM2_MAX param was never accesible to users. No conversion required.)
            // MIN (Previous bug meant RPM2_MIN param was never accesible to users. No conversion required.)
            {4, 4, 1}, // MIN_QUAL (Previously the min quality of the 1st RPM instance was used for all RPM instances.)
            {12, 5, 1}, // PIN
            {13, 6, 1}, // ESC_MASK
    };

    char param_name[17] = {0};
    AP_Param::ConversionInfo info;
    info.new_name = param_name;

    if (!AP_Param::find_top_level_key_by_pointer(this, info.old_key)) {
        _params[0].type.save(true);
        return; // no conversion is supported on this platform
    }

    for (uint8_t i = 0; i < ARRAY_SIZE(conversionTable); i++) {
        uint8_t param_instance = conversionTable[i].instance + 1;
        uint8_t destination_index = conversionTable[i].new_index;
        info.old_group_element = conversionTable[i].old_element;

        // The var type of the params has not changed in the conversion so this is ok:
        info.type = (ap_var_type)AP_RPM_Params::var_info[destination_index].type;
        hal.util->snprintf(param_name, sizeof(param_name), "RPM%X_%s", param_instance, AP_RPM_Params::var_info[destination_index].name);
        param_name[sizeof(param_name)-1] = '\0';

        AP_Param::convert_old_parameter(&info, 1.0f, 0);
    }

    // force _params[0].type into storage to flag that conversion has been done
    _params[0].type.save(true);
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
        }
    }

#if HAL_LOGGING_ENABLED
    if (enabled(0) || enabled(1)) {
        Log_RPM();
    }
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
void AP_RPM::Log_RPM()
{
    float rpm1 = -1, rpm2 = -1;

    get_rpm(0, rpm1);
    get_rpm(1, rpm2);

    const struct log_RPM pkt{
        LOG_PACKET_HEADER_INIT(LOG_RPM_MSG),
        time_us     : AP_HAL::micros64(),
        rpm1        : rpm1,
        rpm2        : rpm2
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));
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
