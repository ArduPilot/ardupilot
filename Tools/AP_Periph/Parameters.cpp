#include "AP_Periph.h"

extern const AP_HAL::HAL &hal;

/*
 *  AP_Periph parameter definitions
 *
 */

#define GSCALAR(v, name, def) { periph.g.v.vtype, name, Parameters::k_param_ ## v, &periph.g.v, {def_value : def} }
#define ASCALAR(v, name, def) { periph.aparm.v.vtype, name, Parameters::k_param_ ## v, (const void *)&periph.aparm.v, {def_value : def} }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &periph.g.v, {group_info : class::var_info} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, (const void *)&periph.v, {group_info : class::var_info} }
#define GOBJECTN(v, pname, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## pname, (const void *)&periph.v, {group_info : class::var_info} }

const AP_Param::Info AP_Periph_FW::var_info[] = {
    // @Param: FORMAT_VERSION
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    GSCALAR(format_version,         "FORMAT_VERSION", 0),

    // can node number, 0 for dynamic node allocation
    GSCALAR(can_node,         "CAN_NODE", HAL_CAN_DEFAULT_NODE_ID),

    // can node baudrate
    GSCALAR(can_baudrate,     "CAN_BAUDRATE", 1000000),
    
#ifdef HAL_PERIPH_ENABLE_GPS
    // GPS driver
    // @Group: GPS_
    // @Path: ../libraries/AP_GPS/AP_GPS.cpp
    GOBJECT(gps, "GPS_", AP_GPS),
#endif

#ifdef HAL_PERIPH_ENABLE_MAG
    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/AP_Compass.cpp
    GOBJECT(compass,         "COMPASS_",     Compass),
#endif

#ifdef HAL_PERIPH_ENABLE_BARO
    // Baro driver
    // @Group: BARO_
    // @Path: ../libraries/AP_Baro/AP_Baro.cpp
    GOBJECT(baro, "BARO_", AP_Baro),
#endif
    
    AP_VAREND
};


void AP_Periph_FW::load_parameters(void)
{
    AP_Param::setup_sketch_defaults();

    if (!AP_Param::check_var_info()) {
        hal.console->printf("Bad parameter table\n");
        AP_HAL::panic("Bad parameter table");
    }
    if (!g.format_version.load() ||
        g.format_version != Parameters::k_format_version) {
        // erase all parameters
        StorageManager::erase();
        AP_Param::erase_all();

        // save the current format version
        g.format_version.set_and_save(Parameters::k_format_version);
    }

    // Load all auto-loaded EEPROM variables
    AP_Param::load_all();
}
