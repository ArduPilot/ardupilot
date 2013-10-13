/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *  AntennaTracker parameter definitions
 *
 */

#define GSCALAR(v, name, def) { g.v.vtype, name, Parameters::k_param_ ## v, &g.v, {def_value : def} }
#define ASCALAR(v, name, def) { aparm.v.vtype, name, Parameters::k_param_ ## v, &aparm.v, {def_value : def} }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &g.v, {group_info : class::var_info} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &v, {group_info : class::var_info} }

const AP_Param::Info var_info[] PROGMEM = {
    GSCALAR(format_version,         "FORMAT_VERSION", 0),
    GSCALAR(software_type,          "SYSID_SW_TYPE",  Parameters::k_software_type),

    // @Param: SYSID_THISMAV
    // @DisplayName: MAVLink system ID
    // @Description: The identifier of this device in the MAVLink protocol
    // @Range: 1 255
    // @User: Advanced
    GSCALAR(sysid_this_mav,         "SYSID_THISMAV",  MAV_SYSTEM_ID),

    // @Param: SYSID_MYGCS
    // @DisplayName: Ground station MAVLink system ID
    // @Description: The identifier of the ground station in the MAVLink protocol. Don't change this unless you also modify the ground station to match.
    // @Range: 1 255
    // @User: Advanced
    GSCALAR(sysid_my_gcs,           "SYSID_MYGCS",    255),

    // @Param: SERIAL0_BAUD
    // @DisplayName: USB Console Baud Rate
    // @Description: The baud rate used on the main uart
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200
    // @User: Standard
    GSCALAR(serial0_baud,           "SERIAL0_BAUD",   SERIAL0_BAUD/1000),

    // @Param: SERIAL3_BAUD
    // @DisplayName: Telemetry Baud Rate
    // @Description: The baud rate used on the telemetry port
    // @Values: 1:1200,2:2400,4:4800,9:9600,19:19200,38:38400,57:57600,111:111100,115:115200
    // @User: Standard
    GSCALAR(serial3_baud,           "SERIAL3_BAUD",   SERIAL3_BAUD/1000),



    // @Param: MAG_ENABLE
    // @DisplayName: Enable Compass
    // @Description: Setting this to Enabled(1) will enable the compass. Setting this to Disabled(0) will disable the compass. Note that this is separate from COMPASS_USE. This will enable the low level senor, and will enable logging of magnetometer data. To use the compass for navigation you must also set COMPASS_USE to 1.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(compass_enabled,        "MAG_ENABLE",     1),

    // barometer ground calibration. The GND_ prefix is chosen for
    // compatibility with previous releases of ArduPlane
    // @Group: GND_
    // @Path: ../libraries/AP_Baro/AP_Baro.cpp
    GOBJECT(barometer, "GND_", AP_Baro),

    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/Compass.cpp
    GOBJECT(compass,                "COMPASS_",     Compass),

    // @Group: SCHED_
    // @Path: ../libraries/AP_Scheduler/AP_Scheduler.cpp
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),

    // @Group: SR0_
    // @Path: GCS_Mavlink.pde
    GOBJECT(gcs0,                                   "SR0_",     GCS_MAVLINK),

    // @Group: SR3_
    // @Path: GCS_Mavlink.pde
    GOBJECT(gcs3,                                   "SR3_",     GCS_MAVLINK),

    // @Group: INS_
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins,                    "INS_", AP_InertialSensor),

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),

#if CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    // @Group: SIM_
    // @Path: ../libraries/SITL/SITL.cpp
    GOBJECT(sitl, "SIM_", SITL),
#endif

    // RC channel
    //-----------
    // @Group: RC1_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GOBJECT(channel_yaw,       "RC1_", RC_Channel),

    // @Group: RC2_
    // @Path: ../libraries/RC_Channel/RC_Channel.cpp
    GOBJECT(channel_pitch,     "RC2_", RC_Channel),

	GGROUP(pidPitch2Srv,       "PITCH2SRV_", PID),
	GGROUP(pidYaw2Srv,         "YAW2SRV_", PID),

    AP_VAREND
};


static void load_parameters(void)
{
    if (!g.format_version.load() ||
        g.format_version != Parameters::k_format_version) {

        // erase all parameters
        cliSerial->printf_P(PSTR("Firmware change: erasing EEPROM...\n"));
        AP_Param::erase_all();

        // save the current format version
        g.format_version.set_and_save(Parameters::k_format_version);
        cliSerial->println_P(PSTR("done."));
    } else {
        uint32_t before = hal.scheduler->micros();
        // Load all auto-loaded EEPROM variables
        AP_Param::load_all();
        cliSerial->printf_P(PSTR("load_all took %luus\n"), hal.scheduler->micros() - before);
    }
}
