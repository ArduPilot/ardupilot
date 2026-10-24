// SPDX-License-Identifier: GPL-3.0-or-later
#include "BindingAirspeed.h"

#include "AP_Module_config.h"

#if AP_MODULE_ENABLED
#include "BindingSupport.h"
#include <AP_Module/HostApi.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Airspeed/AP_Airspeed_Backend.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

namespace AP_Module
{

int32_t binding_airspeed_get_backend(const uint8_t *input, uint32_t input_size,
                                     uint8_t *output, uint32_t output_size)
{
    if (input == nullptr || input_size != 1 || output == nullptr || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
#if AP_AIRSPEED_SCRIPTING_ENABLED && APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    auto *frontend = AP_Airspeed::get_singleton();
    output[0] = frontend != nullptr && frontend->get_backend(input[0]) != nullptr;
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_airspeed_script_value(const uint8_t *input, uint32_t input_size,
                                      uint8_t *output, uint32_t output_size,
                                      uint8_t operation)
{
    if (input == nullptr || input_size != 5 || output == nullptr || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
#if AP_AIRSPEED_SCRIPTING_ENABLED && APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    auto *frontend = AP_Airspeed::get_singleton();
    auto *backend = frontend == nullptr ? nullptr : frontend->get_backend(input[0]);
    if (backend == nullptr) {
        return AP_MODULE_STATUS_UNAVAILABLE;
    }
    const float value = BindingWire::read_float(input + 1);
    switch (operation) {
    case 0:
        output[0] = backend->handle_script_airspeed(value);
        break;
    case 1:
        output[0] = backend->handle_script_differential_pressure(value);
        break;
    case 2:
        output[0] = backend->handle_script_temperature(value);
        break;
    default:
        return AP_MODULE_STATUS_INVALID;
    }
    return AP_MODULE_STATUS_OK;
#else
    (void)operation;
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

}
#endif
