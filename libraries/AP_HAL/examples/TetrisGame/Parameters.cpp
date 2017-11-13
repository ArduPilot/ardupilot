#include "TetrisGame.h"

/*
 *  Tetris APM game parameter definitions
 *
 */

#define GSCALAR(v, name, def) { gametetris.g.v.vtype, name, Parameters::k_param_ ## v, &gametetris.g.v, {def_value : def} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, (const void *)&gametetris.v, {group_info : class::var_info} }

const AP_Param::Info GameTetris::var_info[] = {
    // @Param: FORMAT_VERSION
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    GSCALAR(format_version,         "FORMAT_VERSION", 0),

    // @Param: SYSID_SW_TYPE
    // @DisplayName: Software Type
    // @Description: This is used by the ground station to recognise the software type (eg ArduPlane vs ArduCopter)
    // @Values: 0:ArduPlane,4:AntennaTracker,10:Copter,20:Rover,40:ArduSub
    // @User: Advanced
    // @ReadOnly: True
    GSCALAR(software_type,          "SYSID_SW_TYPE",  Parameters::k_software_type),

    // @Group: INS_
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins,                    "INS_", AP_InertialSensor),

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // @Group: SIM_
    // @Path: ../libraries/SITL/SITL.cpp
    GOBJECT(sitl, "SIM_", SITL::SITL),
#endif

    AP_VAREND
};

void GameTetris::load_parameters(void)
{
    if (!g.format_version.load() ||
        g.format_version != Parameters::k_format_version) {

        // erase all parameters
        hal.console->printf("Firmware change: erasing EEPROM...\n");
        AP_Param::erase_all();

        // save the current format version
        g.format_version.set_and_save(Parameters::k_format_version);
        hal.console->printf("done.\n");
    }

    uint32_t before = AP_HAL::micros();
    // Load all auto-loaded EEPROM variables
    AP_Param::load_all();
    hal.console->printf("load_all took %luus\n", (unsigned long)(AP_HAL::micros() - before));
}
