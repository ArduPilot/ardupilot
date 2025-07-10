#include "AP_GenericVehicle.h"

const AP_Param::Info AP_GenericVehicle::var_info[] = {
    // @Param: FORMAT_VERSION
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    GSCALAR(format_version,         "FORMAT_VERSION", 0),

    // @Group:
    // @Path: ../libraries/AP_Vehicle/AP_Vehicle.cpp
    PARAM_VEHICLE_INFO,

#if HAL_GCS_ENABLED
    // @Param: SYSID_THISMAV
    // @DisplayName: MAVLink system ID of this vehicle
    // @Description: Allows setting an individual MAVLink system id for this vehicle to distinguish it from others on the same network
    // @Range: 1 255
    // @User: Advanced
    GSCALAR(sysid_this_mav,         "SYSID_THISMAV",  MAV_SYSTEM_ID),
#endif

#if AP_BOARDCONFIG_SINGLETON_ENABLED
    // @Group: BRD_
    // @Path: ../libraries/AP_BoardConfig/AP_BoardConfig.cpp
    GOBJECT(BoardConfig,            "BRD_",       AP_BoardConfig),
#endif

#if HAL_LOGGING_ENABLED
    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: yes
    // @User: Standard
    GSCALAR(log_bitmask,    "LOG_BITMASK",          65535),
#endif

    AP_VAREND
};

void AP_GenericVehicle::load_parameters(void)
{
    AP_Vehicle::load_parameters(g.format_version, Parameters::k_format_version);
}
