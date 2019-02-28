#include "Tracker.h"

/*
 *  AntennaTracker parameter definitions
 *
 */

#define GSCALAR(v, name, def) { tracker.g.v.vtype, name, Parameters::k_param_ ## v, &tracker.g.v, {def_value : def} }
#define ASCALAR(v, name, def) { tracker.aparm.v.vtype, name, Parameters::k_param_ ## v, (const void *)&tracker.aparm.v, {def_value : def} }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &tracker.g.v, {group_info : class::var_info} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, (const void *)&tracker.v, {group_info : class::var_info} }
#define GOBJECTN(v, pname, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## pname, (const void *)&tracker.v, {group_info : class::var_info} }

const AP_Param::Info Tracker::var_info[] = {
    // @Param: FORMAT_VERSION
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    GSCALAR(format_version,         "FORMAT_VERSION", 0),

    // @Param: SYSID_THISMAV
    // @DisplayName: MAVLink system ID of this vehicle
    // @Description: Allows setting an individual system id for this vehicle to distinguish it from others on the same network
    // @Range: 1 255
    // @User: Advanced
    GSCALAR(sysid_this_mav,         "SYSID_THISMAV",  MAV_SYSTEM_ID),

    // @Param: SYSID_MYGCS
    // @DisplayName: Ground station MAVLink system ID
    // @Description: The identifier of the ground station in the MAVLink protocol. Don't change this unless you also modify the ground station to match.
    // @Range: 1 255
    // @User: Advanced
    GSCALAR(sysid_my_gcs,           "SYSID_MYGCS",    255),

    // @Param: SYSID_TARGET
    // @DisplayName: Target vehicle's MAVLink system ID
    // @Description: The identifier of the vehicle being tracked. This should be zero (to auto detect) or be the same as the SYSID_THISMAV parameter of the vehicle being tracked.
    // @Range: 1 255
    // @User: Advanced
    GSCALAR(sysid_target,           "SYSID_TARGET",    0),

    // @Param: MAG_ENABLE
    // @DisplayName: Enable Compass
    // @Description: Setting this to Enabled(1) will enable the compass. Setting this to Disabled(0) will disable the compass. Note that this is separate from COMPASS_USE. This will enable the low level senor, and will enable logging of magnetometer data. To use the compass for navigation you must also set COMPASS_USE to 1.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(compass_enabled,        "MAG_ENABLE",     1),

    // @Param: YAW_SLEW_TIME
    // @DisplayName: Time for yaw to slew through its full range
    // @Description: This controls how rapidly the tracker will change the servo output for yaw. It is set as the number of seconds to do a full rotation. You can use this parameter to slow the trackers movements, which may help with some types of trackers. A value of zero will allow for unlimited servo movement per update.
    // @Units: s
    // @Increment: 0.1
    // @Range: 0 20
    // @User: Standard
    GSCALAR(yaw_slew_time,          "YAW_SLEW_TIME",    2),

    // @Param: PITCH_SLEW_TIME
    // @DisplayName: Time for pitch to slew through its full range
    // @Description: This controls how rapidly the tracker will change the servo output for pitch. It is set as the number of seconds to do a full range of pitch movement. You can use this parameter to slow the trackers movements, which may help with some types of trackers. A value of zero will allow for unlimited servo movement per update.
    // @Units: s
    // @Increment: 0.1
    // @Range: 0 20
    // @User: Standard
    GSCALAR(pitch_slew_time,        "PITCH_SLEW_TIME",  2),

    // @Param: SCAN_SPEED
    // @DisplayName: Speed at which to rotate in scan mode
    // @Description: This controls how rapidly the tracker will move the servos in SCAN mode
    // @Units: deg/s
    // @Increment: 1
    // @Range: 0 100
    // @User: Standard
    GSCALAR(scan_speed,             "SCAN_SPEED",      5),

    // @Param: MIN_REVERSE_TIME
    // @DisplayName: Minimum time to apply a yaw reversal
    // @Description: When the tracker detects it has reached the limit of servo movement in yaw it will reverse and try moving to the other extreme of yaw. This parameter controls the minimum time it should reverse for. It is used to cope with trackers that have a significant lag in movement to ensure they do move all the way around.
    // @Units: s
    // @Increment: 1
    // @Range: 0 20
    // @User: Standard
    GSCALAR(min_reverse_time,       "MIN_REVERSE_TIME",  1),

    // @Param: START_LATITUDE
    // @DisplayName: Initial Latitude before GPS lock
    // @Description: Combined with START_LONGITUDE this parameter allows for an initial position of the tracker to be set. This position will be used until the GPS gets lock. It can also be used to run a stationary tracker with no GPS attached.
    // @Units: deg
    // @Increment: 0.000001
    // @Range: -90 90
    // @User: Standard
    GSCALAR(start_latitude,         "START_LATITUDE",   0),

    // @Param: START_LONGITUDE
    // @DisplayName: Initial Longitude before GPS lock
    // @Description: Combined with START_LATITUDE this parameter allows for an initial position of the tracker to be set. This position will be used until the GPS gets lock. It can also be used to run a stationary tracker with no GPS attached.
    // @Units: deg
    // @Increment: 0.000001
    // @Range: -180 180
    // @User: Standard
    GSCALAR(start_longitude,        "START_LONGITUDE",  0),

    // @Param: STARTUP_DELAY
    // @DisplayName: Delay before first servo movement from trim
    // @Description: This parameter can be used to force the servos to their trim value for a time on startup. This can help with some servo types
    // @Units: s
    // @Increment: 0.1
    // @Range: 0 10
    // @User: Standard
    GSCALAR(startup_delay,          "STARTUP_DELAY",   0),

    // @Param: SERVO_PITCH_TYPE
    // @DisplayName: Type of servo system being used for pitch
    // @Description: This allows selection of position servos or on/off servos for pitch
    // @Values: 0:Position,1:OnOff,2:ContinuousRotation
    // @User: Standard
    GSCALAR(servo_pitch_type,          "SERVO_PITCH_TYPE",   SERVO_TYPE_POSITION),

    // @Param: SERVO_YAW_TYPE
    // @DisplayName: Type of servo system being used for yaw
    // @Description: This allows selection of position servos or on/off servos for yaw
    // @Values: 0:Position,1:OnOff,2:ContinuousRotation
    // @User: Standard
    GSCALAR(servo_yaw_type,          "SERVO_YAW_TYPE",   SERVO_TYPE_POSITION),

    // @Param: ONOFF_YAW_RATE
    // @DisplayName: Yaw rate for on/off servos
    // @Description: Rate of change of yaw in degrees/second for on/off servos
    // @Units: deg/s
    // @Increment: 0.1
    // @Range: 0 50
    // @User: Standard
    GSCALAR(onoff_yaw_rate,      "ONOFF_YAW_RATE", 9.0f),

    // @Param: ONOFF_PITCH_RATE
    // @DisplayName: Pitch rate for on/off servos
    // @Description: Rate of change of pitch in degrees/second for on/off servos
    // @Units: deg/s
    // @Increment: 0.1
    // @Range: 0 50
    // @User: Standard
    GSCALAR(onoff_pitch_rate,      "ONOFF_PITCH_RATE", 1.0f),

    // @Param: ONOFF_YAW_MINT
    // @DisplayName: Yaw minimum movement time
    // @Description: Minimum amount of time in seconds to move in yaw
    // @Units: s
    // @Increment: 0.01
    // @Range: 0 2
    // @User: Standard
    GSCALAR(onoff_yaw_mintime,     "ONOFF_YAW_MINT", 0.1f),

    // @Param: ONOFF_PITCH_MINT
    // @DisplayName: Pitch minimum movement time
    // @Description: Minimim amount of time in seconds to move in pitch
    // @Units: s
    // @Increment: 0.01
    // @Range: 0 2
    // @User: Standard
    GSCALAR(onoff_pitch_mintime,   "ONOFF_PITCH_MINT", 0.1f),

    // @Param: YAW_TRIM
    // @DisplayName: Yaw trim
    // @Description: Amount of extra yaw to add when tracking. This allows for small adjustments for an out of trim compass.
    // @Units: deg
    // @Increment: 0.1
    // @Range: -10 10
    // @User: Standard
    GSCALAR(yaw_trim,              "YAW_TRIM", 0),

    // @Param: PITCH_TRIM
    // @DisplayName: Pitch trim
    // @Description: Amount of extra pitch to add when tracking. This allows for small adjustments for a badly calibrated barometer.
    // @Units: deg
    // @Increment: 0.1
    // @Range: -10 10
    // @User: Standard
    GSCALAR(pitch_trim,              "PITCH_TRIM", 0),

    // @Param: YAW_RANGE
    // @DisplayName: Yaw Angle Range
    // @Description: Yaw axis total range of motion in degrees
    // @Units: deg
    // @Increment: 0.1
    // @Range: 0 360
    // @User: Standard
    GSCALAR(yaw_range,              "YAW_RANGE", YAW_RANGE_DEFAULT),

    // @Param: DISTANCE_MIN
    // @DisplayName: Distance minimum to target
    // @Description: Tracker will track targets at least this distance away
    // @Units: m
    // @Increment: 1
    // @Range: 0 100
    // @User: Standard
    GSCALAR(distance_min,           "DISTANCE_MIN", DISTANCE_MIN_DEFAULT),

    // @Param: ALT_SOURCE
    // @DisplayName: Altitude Source
    // @Description: What provides altitude information for vehicle. Vehicle only assumes tracker has same altitude as vehicle's home
    // @Values: 0:Barometer,1:GPS,2:GPS vehicle only
    // @User: Standard
    GSCALAR(alt_source,				"ALT_SOURCE",	0),

    // @Param: MAV_UPDATE_RATE
    // @DisplayName: Mavlink Update Rate
    // @Description: The rate at which Mavlink updates position and baro data
    // @Units: Hz
    // @Increment: 1
    // @Range: 1 10
    // @User: Standard
    GSCALAR(mavlink_update_rate,	"MAV_UPDATE_RATE",	1),

    // @Param: PITCH_MIN
    // @DisplayName: Minimum Pitch Angle
    // @Description: The lowest angle the pitch can reach
    // @Units: deg
    // @Increment: 1
    // @Range: -90 0
    // @User: Standard
    GSCALAR(pitch_min,               "PITCH_MIN",	PITCH_MIN_DEFAULT),

    // @Param: PITCH_MAX
    // @DisplayName: Maximum Pitch Angle
    // @Description: The highest angle the pitch can reach
    // @Units: deg
    // @Increment: 1
    // @Range: 0 90
    // @User: Standard
    GSCALAR(pitch_max,               "PITCH_MAX",	PITCH_MAX_DEFAULT),

    // barometer ground calibration. The GND_ prefix is chosen for
    // compatibility with previous releases of ArduPlane
    // @Group: GND_
    // @Path: ../libraries/AP_Baro/AP_Baro.cpp
    GOBJECT(barometer, "GND_", AP_Baro),

    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/AP_Compass.cpp
    GOBJECT(compass,                "COMPASS_",     Compass),

    // @Group: SCHED_
    // @Path: ../libraries/AP_Scheduler/AP_Scheduler.cpp
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),

    // @Group: SR0_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(gcs().chan(0), gcs0,        "SR0_",     GCS_MAVLINK),

    // @Group: SR1_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(gcs().chan(1),  gcs1,       "SR1_",     GCS_MAVLINK),

    // @Group: SR2_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(gcs().chan(2),  gcs2,       "SR2_",     GCS_MAVLINK),

    // @Group: SR3_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(gcs().chan(3),  gcs3,       "SR3_",     GCS_MAVLINK),

    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: 4 byte bitmap of log types to enable
    // @Values: 127:Default,0:Disabled
    // @Bitmask: 0:ATTITUDE,1:GPS,2:RCIN,3:IMU,4:RCOUT,5:COMPASS,6:Battery
    // @User: Standard
    GSCALAR(log_bitmask, "LOG_BITMASK", DEFAULT_LOG_BITMASK),

    // @Group: INS_
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins,                    "INS_", AP_InertialSensor),

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // @Group: SIM_
    // @Path: ../libraries/SITL/SITL.cpp
    GOBJECT(sitl, "SIM_", SITL::SITL),
#endif

    // @Group: BRD_
    // @Path: ../libraries/AP_BoardConfig/AP_BoardConfig.cpp
    GOBJECT(BoardConfig,            "BRD_",       AP_BoardConfig),

#if HAL_WITH_UAVCAN
    // @Group: CAN_
    // @Path: ../libraries/AP_BoardConfig/AP_BoardConfig_CAN.cpp
    GOBJECT(BoardConfig_CAN,        "CAN_",       AP_BoardConfig_CAN),
#endif

    // GPS driver
    // @Group: GPS_
    // @Path: ../libraries/AP_GPS/AP_GPS.cpp
    GOBJECT(gps, "GPS_", AP_GPS),

    // @Group: NTF_
    // @Path: ../libraries/AP_Notify/AP_Notify.cpp
    GOBJECT(notify, "NTF_",  AP_Notify),

    // @Path: RC_Channels.cpp
    GOBJECT(rc_channels,     "RC", RC_Channels_Tracker),

    // @Path: ../libraries/SRV_Channel/SRV_Channels.cpp
    GOBJECT(servo_channels,     "SERVO", SRV_Channels),
    
    // @Group: SERIAL
    // @Path: ../libraries/AP_SerialManager/AP_SerialManager.cpp
    GOBJECT(serial_manager,    "SERIAL",   AP_SerialManager),

    // @Param: PITCH2SRV_P
    // @DisplayName: Pitch axis controller P gain
    // @Description: Pitch axis controller P gain.  Converts the difference between desired pitch angle and actual pitch angle into a pitch servo pwm change
    // @Range: 0.0 3.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: PITCH2SRV_I
    // @DisplayName: Pitch axis controller I gain
    // @Description: Pitch axis controller I gain.  Corrects long-term difference in desired pitch angle vs actual pitch angle
    // @Range: 0.0 3.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: PITCH2SRV_IMAX
    // @DisplayName: Pitch axis controller I gain maximum
    // @Description: Pitch axis controller I gain maximum.  Constrains the maximum pwm change that the I gain will output
    // @Range: 0 4000
    // @Increment: 10
    // @Units: d%
    // @User: Standard

    // @Param: PITCH2SRV_D
    // @DisplayName: Pitch axis controller D gain
    // @Description: Pitch axis controller D gain.  Compensates for short-term change in desired pitch angle vs actual pitch angle
    // @Range: 0.001 0.1
    // @Increment: 0.001
    // @User: Standard
	GGROUP(pidPitch2Srv,       "PITCH2SRV_", AC_PID),

    // @Param: YAW2SRV_P
    // @DisplayName: Yaw axis controller P gain
    // @Description: Yaw axis controller P gain.  Converts the difference between desired yaw angle (heading) and actual yaw angle into a yaw servo pwm change
    // @Range: 0.0 3.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: YAW2SRV_I
    // @DisplayName: Yaw axis controller I gain
    // @Description: Yaw axis controller I gain.  Corrects long-term difference in desired yaw angle (heading) vs actual yaw angle
    // @Range: 0.0 3.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: YAW2SRV_IMAX
    // @DisplayName: Yaw axis controller I gain maximum
    // @Description: Yaw axis controller I gain maximum.  Constrains the maximum pwm change that the I gain will output
    // @Range: 0 4000
    // @Increment: 10
    // @Units: d%
    // @User: Standard

    // @Param: YAW2SRV_D
    // @DisplayName: Yaw axis controller D gain
    // @Description: Yaw axis controller D gain.  Compensates for short-term change in desired yaw angle (heading) vs actual yaw angle
    // @Range: 0.001 0.1
    // @Increment: 0.001
    // @User: Standard
	GGROUP(pidYaw2Srv,         "YAW2SRV_", AC_PID),

    // @Param: CMD_TOTAL
    // @DisplayName: Number of loaded mission items
    // @Description: Set to 1 if HOME location has been loaded by the ground station. Do not change this manually.
    // @Range: 1 255
    // @User: Advanced
    GSCALAR(command_total,          "CMD_TOTAL",      0),

    // @Group: BATT
    // @Path: ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    GOBJECT(battery,                "BATT", AP_BattMonitor),

    // @Param: GCS_PID_MASK
    // @DisplayName: GCS PID tuning mask
    // @Description: bitmask of PIDs to send MAVLink PID_TUNING messages for
    // @User: Advanced
    // @Values: 0:None,1:Pitch,2:Yaw
    // @Bitmask: 0:Pitch,1:Yaw
    GSCALAR(gcs_pid_mask,           "GCS_PID_MASK",     0),

    AP_VAREND
};


void Tracker::load_parameters(void)
{
    if (!g.format_version.load() ||
        g.format_version != Parameters::k_format_version) {

        // erase all parameters
        hal.console->printf("Firmware change: erasing EEPROM...\n");
        StorageManager::erase();
        AP_Param::erase_all();

        // save the current format version
        g.format_version.set_and_save(Parameters::k_format_version);
        hal.console->printf("done.\n");
    }

    uint32_t before = AP_HAL::micros();
    // Load all auto-loaded EEPROM variables
    AP_Param::load_all();
    hal.console->printf("load_all took %luus\n", (unsigned long)(AP_HAL::micros() - before));

#if HAL_HAVE_SAFETY_SWITCH
    // configure safety switch to allow stopping the motors while armed
    AP_Param::set_default_by_name("BRD_SAFETYOPTION", AP_BoardConfig::BOARD_SAFETY_OPTION_BUTTON_ACTIVE_SAFETY_OFF|
                                                      AP_BoardConfig::BOARD_SAFETY_OPTION_BUTTON_ACTIVE_SAFETY_ON|
                                                      AP_BoardConfig::BOARD_SAFETY_OPTION_BUTTON_ACTIVE_ARMED);
#endif
}
