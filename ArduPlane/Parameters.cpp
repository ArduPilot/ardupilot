#include "Plane.h"

/*
 *  ArduPlane parameter definitions
 *
 */

#define GSCALAR(v, name, def) { plane.g.v.vtype, name, Parameters::k_param_ ## v, &plane.g.v, {def_value : def} }
#define ASCALAR(v, name, def) { plane.aparm.v.vtype, name, Parameters::k_param_ ## v, (const void *)&plane.aparm.v, {def_value : def} }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &plane.g.v, {group_info : class::var_info} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, (const void *)&plane.v, {group_info : class::var_info} }
#define GOBJECTN(v, pname, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## pname, (const void *)&plane.v, {group_info : class::var_info} }

const AP_Param::Info Plane::var_info[] = {
    // @Param: FORMAT_VERSION
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    GSCALAR(format_version,         "FORMAT_VERSION", 0),

    // @Param: SYSID_THISMAV
    // @DisplayName: MAVLink system ID of this vehicle
    // @Description: Allows setting an individual MAVLink system id for this vehicle to distinguish it from others on the same network
    // @Range: 1 255
    // @User: Advanced
    GSCALAR(sysid_this_mav,         "SYSID_THISMAV",  MAV_SYSTEM_ID),

    // @Param: SYSID_MYGCS
    // @DisplayName: Ground station MAVLink system ID
    // @Description: The identifier of the ground station in the MAVLink protocol. Don't change this unless you also modify the ground station to match.
    // @Range: 1 255
    // @User: Advanced
    GSCALAR(sysid_my_gcs,           "SYSID_MYGCS",    255),

    // @Group: SERIAL
    // @Path: ../libraries/AP_SerialManager/AP_SerialManager.cpp
    GOBJECT(serial_manager, "SERIAL",   AP_SerialManager),

    // @Param: AUTOTUNE_LEVEL
    // @DisplayName: Autotune level
    // @Description: Level of aggressiveness of pitch and roll PID gains. Lower values result in a 'softer' tune. Level 6 recommended for most planes.
    // @Range: 1 10
    // @Increment: 1
    // @User: Standard
    ASCALAR(autotune_level, "AUTOTUNE_LEVEL",  6),

    // @Param: TELEM_DELAY
    // @DisplayName: Telemetry startup delay 
    // @Description: The amount of time (in seconds) to delay radio telemetry to prevent an Xbee bricking on power up
    // @User: Standard
    // @Units: s
    // @Range: 0 30
    // @Increment: 1
    GSCALAR(telem_delay,            "TELEM_DELAY",     0),

    // @Param: GCS_PID_MASK
    // @DisplayName: GCS PID tuning mask
    // @Description: bitmask of PIDs to send MAVLink PID_TUNING messages for
    // @User: Advanced
    // @Bitmask: 0:Roll,1:Pitch,2:Yaw,3:Steering,4:Landing
    GSCALAR(gcs_pid_mask,           "GCS_PID_MASK",     0),

    // @Param: KFF_RDDRMIX
    // @DisplayName: Rudder Mix
    // @Description: Amount of rudder to add during aileron movement. Increase if nose initially yaws away from roll. Reduces adverse yaw.
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard
    GSCALAR(kff_rudder_mix,         "KFF_RDDRMIX",    RUDDER_MIX),

    // @Param: KFF_THR2PTCH
    // @DisplayName: Throttle to Pitch Mix
    // @Description: Degrees of elevator added for full throttle application. Increase to compensate for throttle causing down pitch.
    // @Range: 0 5
    // @Increment: 0.01
    // @User: Advanced
    GSCALAR(kff_throttle_to_pitch,  "KFF_THR2PTCH",   0),

    // @Param: STAB_PITCH_DOWN
    // @DisplayName: Low throttle pitch down trim 
    // @Description: Degrees of down pitch added when throttle is below TRIM_THROTTLE in FBWA and AUTOTUNE modes. Scales linearly so full value is added when THR_MIN is reached. Helps to keep airspeed higher in glides or landing approaches and prevents accidental stalls. 2 degrees recommended for most planes.
    // @Range: 0 15
    // @Increment: 0.1
    // @Units: deg
    // @User: Advanced
    GSCALAR(stab_pitch_down, "STAB_PITCH_DOWN",   2.0f),

    // @Param: GLIDE_SLOPE_MIN
    // @DisplayName: Glide slope minimum
    // @Description: This controls the minimum altitude change for a waypoint before a glide slope will be used instead of an immediate altitude change. The default value is 15 meters, which helps to smooth out waypoint missions where small altitude changes happen near waypoints. If you don't want glide slopes to be used in missions then you can set this to zero, which will disable glide slope calculations. Otherwise you can set it to a minimum number of meters of altitude error to the destination waypoint before a glide slope will be used to change altitude.
    // @Range: 0 1000
    // @Increment: 1
    // @Units: m
    // @User: Advanced
    GSCALAR(glide_slope_min, "GLIDE_SLOPE_MIN", 15),

    // @Param: GLIDE_SLOPE_THR
    // @DisplayName: Glide slope threshold
    // @Description: This controls the height above the glide slope the plane may be before rebuilding a glide slope. This is useful for smoothing out an autotakeoff
    // @Range: 0 100
    // @Increment: 1
    // @Units: m
    // @User: Advanced
    GSCALAR(glide_slope_threshold, "GLIDE_SLOPE_THR", 5.0),

    // @Param: STICK_MIXING
    // @DisplayName: Stick Mixing
    // @Description: When enabled, this adds user stick input to the control surfaces in auto modes, allowing the user to have some degree of flight control without changing modes.  There are two types of stick mixing available. If you set STICK_MIXING to 1 then it will use "fly by wire" mixing, which controls the roll and pitch in the same way that the FBWA mode does. This is the safest option if you usually fly ArduPlane in FBWA or FBWB mode. If you set STICK_MIXING to 2 then it will enable direct mixing mode, which is what the STABILIZE mode uses. That will allow for much more extreme maneuvers while in AUTO mode.
    // @Values: 0:Disabled,1:FBWMixing,2:DirectMixing
    // @User: Advanced
    GSCALAR(stick_mixing,           "STICK_MIXING",   STICK_MIXING_FBW),

    // @Param: AUTO_FBW_STEER
    // @DisplayName: Use FBWA steering in AUTO
    // @Description: When enabled this option gives FBWA navigation and steering in AUTO mode. This can be used to allow manual stabilised piloting with waypoint logic for triggering payloads. With this enabled the pilot has the same control over the plane as in FBWA mode, and the normal AUTO navigation is completely disabled. THIS OPTION IS NOT RECOMMENDED FOR NORMAL USE.
    // @Values: 0:Disabled,42:Enabled
    // @User: Advanced
    GSCALAR(auto_fbw_steer,          "AUTO_FBW_STEER",   0),

    // @Param: TKOFF_THR_MINSPD
    // @DisplayName: Takeoff throttle min speed
    // @Description: Minimum GPS ground speed in m/s used by the speed check that un-suppresses throttle in auto-takeoff. This can be be used for catapult launches where you want the motor to engage only after the plane leaves the catapult, but it is preferable to use the TKOFF_THR_MINACC and TKOFF_THR_DELAY parameters for catapult launches due to the errors associated with GPS measurements. For hand launches with a pusher prop it is strongly advised that this parameter be set to a value no less than 4 m/s to provide additional protection against premature motor start. Note that the GPS velocity will lag the real velocity by about 0.5 seconds. The ground speed check is delayed by the TKOFF_THR_DELAY parameter.
    // @Units: m/s
    // @Range: 0 30
    // @Increment: 0.1
    // @User: User
    GSCALAR(takeoff_throttle_min_speed,     "TKOFF_THR_MINSPD",  0),

    // @Param: TKOFF_THR_MINACC
    // @DisplayName: Takeoff throttle min acceleration
    // @Description: Minimum forward acceleration in m/s/s before arming the ground speed check in auto-takeoff. This is meant to be used for hand launches. Setting this value to 0 disables the acceleration test which means the ground speed check will always be armed which could allow GPS velocity jumps to start the engine. For hand launches and bungee launches this should be set to around 15. Also see TKOFF_ACCEL_CNT paramter for control of full "shake to arm".
    // @Units: m/s/s
    // @Range: 0 30
    // @Increment: 0.1
    // @User: User
    GSCALAR(takeoff_throttle_min_accel,     "TKOFF_THR_MINACC",  0),

    // @Param: TKOFF_THR_DELAY
    // @DisplayName: Takeoff throttle delay
    // @Description: This parameter sets the time delay (in 1/10ths of a second) that the ground speed check is delayed after the forward acceleration check controlled by TKOFF_THR_MINACC has passed. For hand launches with pusher propellers it is essential that this is set to a value of no less than 2 (0.2 seconds) to ensure that the aircraft is safely clear of the throwers arm before the motor can start. For bungee launches a larger value can be used (such as 30) to give time for the bungee to release from the aircraft before the motor is started.
    // @Units: ds
    // @Range: 0 127
    // @Increment: 1
    // @User: User
    GSCALAR(takeoff_throttle_delay,     "TKOFF_THR_DELAY",  2),

    // @Param: TKOFF_TDRAG_ELEV
    // @DisplayName: Takeoff tail dragger elevator
    // @Description: This parameter sets the amount of elevator to apply during the initial stage of a takeoff. It is used to hold the tail wheel of a taildragger on the ground during the initial takeoff stage to give maximum steering. This option should be combined with the TKOFF_TDRAG_SPD1 option and the GROUND_STEER_ALT option along with tuning of the ground steering controller. A value of zero means to bypass the initial "tail hold" stage of takeoff. Set to zero for hand and catapult launch. For tail-draggers you should normally set this to 100, meaning full up elevator during the initial stage of takeoff. For most tricycle undercarriage aircraft a value of zero will work well, but for some tricycle aircraft a small negative value (say around -20 to -30) will apply down elevator which will hold the nose wheel firmly on the ground during initial acceleration. Only use a negative value if you find that the nosewheel doesn't grip well during takeoff. Too much down elevator on a tricycle undercarriage may cause instability in steering as the plane pivots around the nosewheel. Add down elevator 10 percent at a time.
    // @Units: %
    // @Range: -100 100
    // @Increment: 1
    // @User: User
    GSCALAR(takeoff_tdrag_elevator,     "TKOFF_TDRAG_ELEV",  0),

    // @Param: TKOFF_TDRAG_SPD1
    // @DisplayName: Takeoff tail dragger speed1
    // @Description: This parameter sets the airspeed at which to stop holding the tail down and transition to rudder control of steering on the ground. When TKOFF_TDRAG_SPD1 is reached the pitch of the aircraft will be held level until TKOFF_ROTATE_SPD is reached, at which point the takeoff pitch specified in the mission will be used to "rotate" the pitch for takeoff climb. Set TKOFF_TDRAG_SPD1 to zero to go straight to rotation. This should be set to zero for hand launch and catapult launch. It should also be set to zero for tricycle undercarriages unless you are using the method above to genetly hold the nose wheel down. For tail dragger aircraft it should be set just below the stall speed.
    // @Units: m/s
    // @Range: 0 30
    // @Increment: 0.1
    // @User: User
    GSCALAR(takeoff_tdrag_speed1,     "TKOFF_TDRAG_SPD1",  0),

    // @Param: TKOFF_ROTATE_SPD
    // @DisplayName: Takeoff rotate speed
    // @Description: This parameter sets the airspeed at which the aircraft will "rotate", setting climb pitch specified in the mission. If TKOFF_ROTATE_SPD is zero then the climb pitch will be used as soon as takeoff is started. For hand launch and catapult launches a TKOFF_ROTATE_SPD of zero should be set. For all ground launches TKOFF_ROTATE_SPD should be set above the stall speed, usually by about 10 to 30 percent
    // @Units: m/s
    // @Range: 0 30
    // @Increment: 0.1
    // @User: User
    GSCALAR(takeoff_rotate_speed,     "TKOFF_ROTATE_SPD",  0),

    // @Param: TKOFF_THR_SLEW
    // @DisplayName: Takeoff throttle slew rate
    // @Description: This parameter sets the slew rate for the throttle during auto takeoff. When this is zero the THR_SLEWRATE parameter is used during takeoff. For rolling takeoffs it can be a good idea to set a lower slewrate for takeoff to give a slower acceleration which can improve ground steering control. The value is a percentage throttle change per second, so a value of 20 means to advance the throttle over 5 seconds on takeoff. Values below 20 are not recommended as they may cause the plane to try to climb out with too little throttle. A value of -1 means no limit on slew rate in takeoff.
    // @Units: %/s
    // @Range: -1 127
    // @Increment: 1
    // @User: User
    GSCALAR(takeoff_throttle_slewrate, "TKOFF_THR_SLEW",  0),

    // @Param: TKOFF_PLIM_SEC
    // @DisplayName: Takeoff pitch limit reduction
    // @Description: This parameter reduces the pitch minimum limit of an auto-takeoff just a few seconds before it reaches the target altitude. This reduces overshoot by allowing the flight controller to start leveling off a few seconds before reaching the target height. When set to zero, the mission pitch min is enforced all the way to and through the target altitude, otherwise the pitch min slowly reduces to zero in the final segment. This is the pitch_min, not the demand. The flight controller should still be commanding to gain altitude to finish the takeoff but with this param it is not forcing it higher than it wants to be.
    // @Units: s
    // @Range: 0 10
    // @Increment: 0.5
    // @User: Advanced
    GSCALAR(takeoff_pitch_limit_reduction_sec, "TKOFF_PLIM_SEC",  2),

    // @Param: TKOFF_FLAP_PCNT
    // @DisplayName: Takeoff flap percentage
    // @Description: The amount of flaps (as a percentage) to apply in automatic takeoff
    // @Range: 0 100
    // @Units: %
    // @User: Advanced
    GSCALAR(takeoff_flap_percent,     "TKOFF_FLAP_PCNT", 0),

    // @Param: FBWA_TDRAG_CHAN
    // @DisplayName: FBWA taildragger channel
    // @Description: This is a RC input channel which when it goes above 1700 enables FBWA taildragger takeoff mode. It should be assigned to a momentary switch. Once this feature is enabled it will stay enabled until the aircraft goes above TKOFF_TDRAG_SPD1 airspeed, changes mode, or the pitch goes above the initial pitch when this is engaged or goes below 0 pitch. When enabled the elevator will be forced to TKOFF_TDRAG_ELEV. This option allows for easier takeoffs on taildraggers in FBWA mode, and also makes it easier to test auto-takeoff steering handling in FBWA. Setting it to 0 disables this option.
    // @User: Standard
    GSCALAR(fbwa_tdrag_chan,          "FBWA_TDRAG_CHAN",  0),

    // @Param: LEVEL_ROLL_LIMIT
    // @DisplayName: Level flight roll limit
    // @Description: This controls the maximum bank angle in degrees during flight modes where level flight is desired, such as in the final stages of landing, and during auto takeoff. This should be a small angle (such as 5 degrees) to prevent a wing hitting the runway during takeoff or landing. Setting this to zero will completely disable heading hold on auto takeoff and final landing approach.
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: User
    GSCALAR(level_roll_limit,              "LEVEL_ROLL_LIMIT",   5),

    // @Param: USE_REV_THRUST
    // @DisplayName: Bitmask for when to allow negative reverse thrust
    // @Description: This controls when to use reverse thrust. If set to zero then reverse thrust is never used. If set to a non-zero value then the bits correspond to flight stages where reverse thrust may be used. Note that reverse thrust is only ever auto-enabled in auto-throttle modes. In modes where throttle control is pilot controlled the ability to do reverse thrust is controlled by throttle stick input. The most commonly used value for USE_REV_THRUST is 2, which means AUTO_LAND only. That enables reverse thrust in the landing stage of AUTO mode. Another common choice is 1, which means to use reverse thrust in all auto flight stages.
    // @Values: 0:Never,1:AutoAlways,2:AutoLanding
    // @Bitmask: 0:AUTO_ALWAYS,1:AUTO_LAND,2:AUTO_LOITER_TO_ALT,3:AUTO_LOITER_ALL,4:AUTO_WAYPOINTS,5:LOITER,6:RTL,7:CIRCLE,8:CRUISE,9:FBWB,10:GUIDED
    // @User: Advanced
    GSCALAR(use_reverse_thrust,     "USE_REV_THRUST",  USE_REVERSE_THRUST_AUTO_LAND_APPROACH),

    // @Param: NAV_CONTROLLER
    // @DisplayName: Navigation controller selection
    // @Description: Which navigation controller to enable. Currently the only navigation controller available is L1. From time to time other experimental controllers will be added which are selected using this parameter.
    // @Values: 0:Default,1:L1Controller
    // @User: Standard
    GSCALAR(nav_controller,          "NAV_CONTROLLER",   AP_Navigation::CONTROLLER_L1),

    // @Param: ALT_CTRL_ALG
    // @DisplayName: Altitude control algorithm
    // @Description: This sets what algorithm will be used for altitude control. The default is zero, which selects the most appropriate algorithm for your airframe. Currently the default is to use TECS (total energy control system). From time to time we will add other experimental altitude control algorithms which will be selected using this parameter.
    // @Values: 0:Automatic
    // @User: Advanced
    GSCALAR(alt_control_algorithm, "ALT_CTRL_ALG",    ALT_CONTROL_DEFAULT),

    // @Param: ALT_OFFSET
    // @DisplayName: Altitude offset
    // @Description: This is added to the target altitude in automatic flight. It can be used to add a global altitude offset to a mission
    // @Units: m
    // @Range: -32767 32767
    // @Increment: 1
    // @User: Advanced
    GSCALAR(alt_offset, "ALT_OFFSET",                 0),

    // @Param: WP_RADIUS
    // @DisplayName: Waypoint Radius
    // @Description: Defines the maximum distance from a waypoint that when crossed indicates the waypoint may be complete. To avoid the aircraft looping around the waypoint in case it misses by more than the WP_RADIUS an additional check is made to see if the aircraft has crossed a "finish line" passing through the waypoint and perpendicular to the flight path from the previous waypoint. If that finish line is crossed then the waypoint is considered complete. Note that the navigation controller may decide to turn later than WP_RADIUS before a waypoint, based on how sharp the turn is and the speed of the aircraft. It is safe to set WP_RADIUS much larger than the usual turn radius of your aircraft and the navigation controller will work out when to turn. If you set WP_RADIUS too small then you will tend to overshoot the turns.
    // @Units: m
    // @Range: 1 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(waypoint_radius,        "WP_RADIUS",      WP_RADIUS_DEFAULT),

    // @Param: WP_MAX_RADIUS
    // @DisplayName: Waypoint Maximum Radius
    // @Description: Sets the maximum distance to a waypoint for the waypoint to be considered complete. This overrides the "cross the finish line" logic that is normally used to consider a waypoint complete. For normal AUTO behaviour this parameter should be set to zero. Using a non-zero value is only recommended when it is critical that the aircraft does approach within the given radius, and should loop around until it has done so. This can cause the aircraft to loop forever if its turn radius is greater than the maximum radius set.
    // @Units: m
    // @Range: 0 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(waypoint_max_radius,        "WP_MAX_RADIUS",      0),

    // @Param: WP_LOITER_RAD
    // @DisplayName: Waypoint Loiter Radius
    // @Description: Defines the distance from the waypoint center, the plane will maintain during a loiter. If you set this value to a negative number then the default loiter direction will be counter-clockwise instead of clockwise.
    // @Units: m
    // @Range: -32767 32767
    // @Increment: 1
    // @User: Standard
    ASCALAR(loiter_radius,          "WP_LOITER_RAD",  LOITER_RADIUS_DEFAULT),

    // @Param: RTL_RADIUS
    // @DisplayName: RTL loiter radius
    // @Description: Defines the radius of the loiter circle when in RTL mode. If this is zero then WP_LOITER_RAD is used. If the radius is negative then a counter-clockwise is used. If positive then a clockwise loiter is used.
    // @Units: m
    // @Range: -32767 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(rtl_radius,             "RTL_RADIUS",  0),
    
#if GEOFENCE_ENABLED == ENABLED
    // @Param: FENCE_ACTION
    // @DisplayName: Action on geofence breach
    // @Description: What to do on fence breach. If this is set to 0 then no action is taken, and geofencing is disabled. If this is set to 1 then the plane will enter GUIDED mode, with the target waypoint as the fence return point. If this is set to 2 then the fence breach is reported to the ground station, but no other action is taken. If set to 3 then the plane enters guided mode but the pilot retains manual throttle control. If set to 4 the plane enters RTL mode, with the target waypoint as the closest rally point (or home point if there are no rally points).
    // @Values: 0:None,1:GuidedMode,2:ReportOnly,3:GuidedModeThrPass,4:RTL_Mode
    // @User: Standard
    GSCALAR(fence_action,           "FENCE_ACTION",   0),

    // @Param: FENCE_TOTAL
    // @DisplayName: Fence Total
    // @Description: Number of geofence points currently loaded
    // @User: Advanced
    GSCALAR(fence_total,            "FENCE_TOTAL",    0),

    // @Param: FENCE_CHANNEL
    // @DisplayName: Fence Channel
    // @Description: RC Channel to use to enable geofence. PWM input above 1750 enables the geofence
    // @User: Standard
    GSCALAR(fence_channel,          "FENCE_CHANNEL",  0),

    // @Param: FENCE_MINALT
    // @DisplayName: Fence Minimum Altitude
    // @Description: Minimum altitude allowed before geofence triggers
    // @Units: m
    // @Range: 0 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(fence_minalt,           "FENCE_MINALT",   0),

    // @Param: FENCE_MAXALT
    // @DisplayName: Fence Maximum Altitude
    // @Description: Maximum altitude allowed before geofence triggers
    // @Units: m
    // @Range: 0 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(fence_maxalt,           "FENCE_MAXALT",   0),

    // @Param: FENCE_RETALT
    // @DisplayName: Fence Return Altitude
    // @Description: Altitude the aircraft will transit to when a fence breach occurs.  If FENCE_RETALT is <= 0 then the midpoint between FENCE_MAXALT and FENCE_MINALT is used, unless FENCE_MAXALT < FENCE_MINALT.  If FENCE_MAXALT < FENCE_MINALT AND FENCE_RETALT is <= 0 then ALT_HOLD_RTL is the altitude used on a fence breach.
    // @Units: m
    // @Range: 0 32767
    // @Increment: 1
    // @User: Standard
    GSCALAR(fence_retalt,           "FENCE_RETALT",   0),

    // @Param: FENCE_AUTOENABLE
    // @DisplayName: Fence automatic enable
    // @Description: When set to 1, geofence automatically enables after an auto takeoff and automatically disables at the beginning of an auto landing.  When on the ground before takeoff the fence is disabled.  When set to 2, the fence autoenables after an auto takeoff, but only disables the fence floor during landing. It is highly recommended to not use this option for line of sight flying and use a fence enable channel instead.
    // @Values: 0:NoAutoEnable,1:AutoEnable,2:AutoEnableDisableFloorOnly
    // @User: Standard
    GSCALAR(fence_autoenable,       "FENCE_AUTOENABLE", 0),

    // @Param: FENCE_RET_RALLY
    // @DisplayName: Fence Return to Rally
    // @Description: When set to 1: on fence breach the plane will return to the nearest rally point rather than the fence return point.  If no rally points have been defined the plane will return to the home point.  
    // @Values: 0:FenceReturnPoint,1:NearestRallyPoint
    // @User: Standard
    GSCALAR(fence_ret_rally,        "FENCE_RET_RALLY",  0),     
#endif

    // @Param: STALL_PREVENTION
    // @DisplayName: Enable stall prevention
    // @Description: Enables roll limits at low airspeed in roll limiting flight modes. Roll limits based on aerodynamic load factor in turns and scale on ARSPD_FBW_MIN that must be set correctly. Without airspeed sensor, uses synthetic airspeed from wind speed estimate that may both be inaccurate.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    ASCALAR(stall_prevention, "STALL_PREVENTION",  1),

    // @Param: ARSPD_FBW_MIN
    // @DisplayName: Minimum Airspeed
    // @Description: Minimum airspeed demanded in automatic throttle modes. Should be set to 20% higher than level flight stall speed.
    // @Units: m/s
    // @Range: 5 100
    // @Increment: 1
    // @User: Standard
    ASCALAR(airspeed_min, "ARSPD_FBW_MIN",  AIRSPEED_FBW_MIN),

    // @Param: ARSPD_FBW_MAX
    // @DisplayName: Maximum Airspeed
    // @Description: Maximum airspeed demanded in automatic throttle modes. Should be set slightly less than level flight speed at THR_MAX and also at least 50% above ARSPD_FBW_MAX to allow for accurate TECS altitude control.
    // @Units: m/s
    // @Range: 5 100
    // @Increment: 1
    // @User: Standard
    ASCALAR(airspeed_max, "ARSPD_FBW_MAX",  AIRSPEED_FBW_MAX),

    // @Param: FBWB_ELEV_REV
    // @DisplayName: Fly By Wire elevator reverse
    // @Description: Reverse sense of elevator in FBWB and CRUISE modes. When set to 0 up elevator (pulling back on the stick) means to lower altitude. When set to 1, up elevator means to raise altitude.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(flybywire_elev_reverse, "FBWB_ELEV_REV",  0),

#if AP_TERRAIN_AVAILABLE
    // @Param: TERRAIN_FOLLOW
    // @DisplayName: Use terrain following
    // @Description: This enables terrain following for CRUISE mode, FBWB mode, RTL and for rally points. To use this option you also need to set TERRAIN_ENABLE to 1, which enables terrain data fetching from the GCS, and you need to have a GCS that supports sending terrain data to the aircraft. When terrain following is enabled then CRUISE and FBWB mode will hold height above terrain rather than height above home. In RTL the return to launch altitude will be considered to be a height above the terrain. Rally point altitudes will be taken as height above the terrain. This option does not affect mission items, which have a per-waypoint flag for whether they are height above home or height above the terrain. To use terrain following missions you need a ground station which can set the waypoint type to be a terrain height waypoint when creating the mission.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(terrain_follow, "TERRAIN_FOLLOW",  0),

    // @Param: TERRAIN_LOOKAHD
    // @DisplayName: Terrain lookahead
    // @Description: This controls how far ahead the terrain following code looks to ensure it stays above upcoming terrain. A value of zero means no lookahead, so the controller will track only the terrain directly below the aircraft. The lookahead will never extend beyond the next waypoint when in AUTO mode.
    // @Range: 0 10000
    // @Units: m
    // @User: Standard
    GSCALAR(terrain_lookahead, "TERRAIN_LOOKAHD",  2000),
#endif

    // @Param: FBWB_CLIMB_RATE
    // @DisplayName: Fly By Wire B altitude change rate
    // @Description: This sets the rate in m/s at which FBWB and CRUISE modes will change its target altitude for full elevator deflection. Note that the actual climb rate of the aircraft can be lower than this, depending on your airspeed and throttle control settings. If you have this parameter set to the default value of 2.0, then holding the elevator at maximum deflection for 10 seconds would change the target altitude by 20 meters.
    // @Range: 1 10
    // @Units: m/s
	// @Increment: 0.1
    // @User: Standard
    GSCALAR(flybywire_climb_rate, "FBWB_CLIMB_RATE",  2.0f),

    // @Param: THR_MIN
    // @DisplayName: Minimum Throttle
    // @Description: Minimum throttle percentage used in all modes except manual, provided THR_PASS_STAB is not set. Negative values allow reverse thrust if hardware supports it.
    // @Units: %
    // @Range: -100 100
    // @Increment: 1
    // @User: Standard
    ASCALAR(throttle_min,           "THR_MIN",        THROTTLE_MIN),

    // @Param: THR_MAX
    // @DisplayName: Maximum Throttle
    // @Description: Maximum throttle percentage used in automatic throttle modes.
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    ASCALAR(throttle_max,           "THR_MAX",        THROTTLE_MAX),

    // @Param: TKOFF_THR_MAX
    // @DisplayName: Maximum Throttle for takeoff
    // @Description: The maximum throttle setting during automatic takeoff. If this is zero then THR_MAX is used for takeoff as well.
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    // @User: Advanced
    ASCALAR(takeoff_throttle_max,   "TKOFF_THR_MAX",        0),

    // @Param: THR_SLEWRATE
    // @DisplayName: Throttle slew rate
    // @Description: Maximum change in throttle percentage per second. Lower limit  based on 1 microsend of servo increase per loop. Divide SCHED_LOOP_RATE by approximately 10 to determine minimum achievable value.
    // @Units: %/s
    // @Range: 0 127
    // @Increment: 1
    // @User: Standard
    ASCALAR(throttle_slewrate,      "THR_SLEWRATE",   100),

    // @Param: FLAP_SLEWRATE
    // @DisplayName: Flap slew rate
    // @Description: maximum percentage change in flap output per second. A setting of 25 means to not change the flap by more than 25% of the full flap range in one second. A value of 0 means no rate limiting.
    // @Units: %/s
    // @Range: 0 100
    // @Increment: 1
    // @User: Advanced
    GSCALAR(flap_slewrate,          "FLAP_SLEWRATE",   75),

    // @Param: THR_SUPP_MAN
    // @DisplayName: Throttle suppress manual passthru
    // @Description: When throttle is suppressed in auto mode it is normally forced to zero. If you enable this option, then while suppressed it will be manual throttle. This is useful on petrol engines to hold the idle throttle manually while waiting for takeoff
	// @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    GSCALAR(throttle_suppress_manual,"THR_SUPP_MAN",   0),

    // @Param: THR_PASS_STAB
    // @DisplayName: Throttle passthru in stabilize
    // @Description: If this is set then when in STABILIZE, FBWA or ACRO modes the throttle is a direct passthru from the transmitter. This means the THR_MIN and THR_MAX settings are not used in these modes. This is useful for petrol engines where you setup a throttle cut switch that suppresses the throttle below the normal minimum.
	// @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    GSCALAR(throttle_passthru_stabilize,"THR_PASS_STAB",   0),

    // @Param: THR_FAILSAFE
    // @DisplayName: Throttle and RC Failsafe Enable
    // @Description: This enables failsafe on loss of RC input. How this is detected depends on the type of RC receiver being used. For older radios an input below the THR_FS_VALUE is used to trigger failsafe. For newer radios the failsafe trigger is part of the protocol between the autopilot and receiver.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(throttle_fs_enabled,    "THR_FAILSAFE",   1),


    // @Param: THR_FS_VALUE
    // @DisplayName: Throttle Failsafe Value
    // @Description: The PWM level on the throttle input channel below which throttle failsafe triggers. Note that this should be well below the normal minimum for your throttle channel.
    // @Range: 925 2200
    // @Increment: 1
    // @User: Standard
    GSCALAR(throttle_fs_value,      "THR_FS_VALUE",   950),

    // @Param: TRIM_THROTTLE
    // @DisplayName: Throttle cruise percentage
    // @Description: Target percentage of throttle to apply for flight in automatic throttle modes and throttle percentage that maintains TRIM_ARSPD_CM. Caution: low battery voltages at the end of flights may require higher throttle to maintain airspeed.
    // @Units: %
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    ASCALAR(throttle_cruise,        "TRIM_THROTTLE",  THROTTLE_CRUISE),

    // @Param: THROTTLE_NUDGE
    // @DisplayName: Throttle nudge enable
    // @Description: When enabled, this uses the throttle input in auto-throttle modes to 'nudge' the throttle or airspeed to higher or lower values. When you have an airspeed sensor the nudge affects the target airspeed, so that throttle inputs above 50% will increase the target airspeed from TRIM_ARSPD_CM up to a maximum of ARSPD_FBW_MAX. When no airspeed sensor is enabled the throttle nudge will push up the target throttle for throttle inputs above 50%.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(throttle_nudge,         "THROTTLE_NUDGE",  1),

    // @Param: FS_SHORT_ACTN
    // @DisplayName: Short failsafe action
    // @Description: The action to take on a short (FS_SHORT_TIMEOUT) failsafe event. A short failsafe even can be triggered either by loss of RC control (see THR_FS_VALUE) or by loss of GCS control (see FS_GCS_ENABL). If in CIRCLE or RTL mode this parameter is ignored. A short failsafe event in stabilization and manual modes will cause an change to CIRCLE mode if FS_SHORT_ACTN is 0 or 1, and a change to FBWA mode if FS_SHORT_ACTN is 2. In all other modes (AUTO, GUIDED and LOITER) a short failsafe event will cause no mode change if FS_SHORT_ACTN is set to 0, will cause a change to CIRCLE mode if set to 1 and will change to FBWA mode if set to 2. Please see the documentation for FS_LONG_ACTN for the behaviour after FS_LONG_TIMEOUT seconds of failsafe.
    // @Values: 0:CIRCLE/no change(if already in AUTO|GUIDED|LOITER),1:CIRCLE,2:FBWA,3:Disable
    // @User: Standard
    GSCALAR(fs_action_short,        "FS_SHORT_ACTN",  FS_ACTION_SHORT_BESTGUESS),

    // @Param: FS_SHORT_TIMEOUT
    // @DisplayName: Short failsafe timeout
    // @Description: The time in seconds that a failsafe condition has to persist before a short failsafe event will occur. This defaults to 1.5 seconds
    // @Units: s
    // @Range: 1 100
    // @Increment: 0.5
    // @User: Standard
    GSCALAR(fs_timeout_short,        "FS_SHORT_TIMEOUT", 1.5f),

    // @Param: FS_LONG_ACTN
    // @DisplayName: Long failsafe action
    // @Description: The action to take on a long (FS_LONG_TIMEOUT seconds) failsafe event. If the aircraft was in a stabilization or manual mode when failsafe started and a long failsafe occurs then it will change to RTL mode if FS_LONG_ACTN is 0 or 1, and will change to FBWA if FS_LONG_ACTN is set to 2. If the aircraft was in an auto mode (such as AUTO or GUIDED) when the failsafe started then it will continue in the auto mode if FS_LONG_ACTN is set to 0, will change to RTL mode if FS_LONG_ACTN is set to 1 and will change to FBWA mode if FS_LONG_ACTN is set to 2. If FS_LONG_ACTION is set to 3, the parachute will be deployed (make sure the chute is configured and enabled). 
    // @Values: 0:Continue,1:ReturnToLaunch,2:Glide,3:Deploy Parachute
    // @User: Standard
    GSCALAR(fs_action_long,         "FS_LONG_ACTN",   FS_ACTION_LONG_CONTINUE),

    // @Param: FS_LONG_TIMEOUT
    // @DisplayName: Long failsafe timeout
    // @Description: The time in seconds that a failsafe condition has to persist before a long failsafe event will occur. This defaults to 5 seconds.
    // @Units: s
    // @Range: 1 300
    // @Increment: 0.5
    // @User: Standard
    GSCALAR(fs_timeout_long,        "FS_LONG_TIMEOUT", 5),

    // @Param: FS_GCS_ENABL
    // @DisplayName: GCS failsafe enable
    // @Description: Enable ground control station telemetry failsafe. Failsafe will trigger after FS_LONG_TIMEOUT seconds of no MAVLink heartbeat messages. There are three possible enabled settings. Seeing FS_GCS_ENABL to 1 means that GCS failsafe will be triggered when the aircraft has not received a MAVLink HEARTBEAT message. Setting FS_GCS_ENABL to 2 means that GCS failsafe will be triggered on either a loss of HEARTBEAT messages, or a RADIO_STATUS message from a MAVLink enabled 3DR radio indicating that the ground station is not receiving status updates from the aircraft, which is indicated by the RADIO_STATUS.remrssi field being zero (this may happen if you have a one way link due to asymmetric noise on the ground station and aircraft radios).Setting FS_GCS_ENABL to 3 means that GCS failsafe will be triggered by Heartbeat(like option one), but only in AUTO mode. WARNING: Enabling this option opens up the possibility of your plane going into failsafe mode and running the motor on the ground it it loses contact with your ground station. If this option is enabled on an electric plane then you should enable ARMING_REQUIRED.
    // @Values: 0:Disabled,1:Heartbeat,2:HeartbeatAndREMRSSI,3:HeartbeatAndAUTO
    // @User: Standard
    GSCALAR(gcs_heartbeat_fs_enabled, "FS_GCS_ENABL", GCS_FAILSAFE_OFF),

    // @Param: FLTMODE_CH
    // @DisplayName: Flightmode channel
    // @Description: RC Channel to use for flight mode control
    // @User: Advanced
    GSCALAR(flight_mode_channel,    "FLTMODE_CH",     FLIGHT_MODE_CHANNEL),

    // @Param: FLTMODE1
    // @DisplayName: FlightMode1
    // @Description: Flight mode for switch position 1 (910 to 1230 and above 2049)
    // @Values: 0:Manual,1:CIRCLE,2:STABILIZE,3:TRAINING,4:ACRO,5:FBWA,6:FBWB,7:CRUISE,8:AUTOTUNE,10:Auto,11:RTL,12:Loiter,14:AVOID_ADSB,15:Guided,17:QSTABILIZE,18:QHOVER,19:QLOITER,20:QLAND,21:QRTL,22:QAUTOTUNE
    // @User: Standard
    GSCALAR(flight_mode1,           "FLTMODE1",       FLIGHT_MODE_1),

    // @Param: FLTMODE2
    // @DisplayName: FlightMode2
    // @Description: Flight mode for switch position 2 (1231 to 1360)
    // @Values: 0:Manual,1:CIRCLE,2:STABILIZE,3:TRAINING,4:ACRO,5:FBWA,6:FBWB,7:CRUISE,8:AUTOTUNE,10:Auto,11:RTL,12:Loiter,14:AVOID_ADSB,15:Guided,17:QSTABILIZE,18:QHOVER,19:QLOITER,20:QLAND,21:QRTL,22:QAUTOTUNE
    // @User: Standard
    GSCALAR(flight_mode2,           "FLTMODE2",       FLIGHT_MODE_2),

    // @Param: FLTMODE3
    // @DisplayName: FlightMode3
    // @Description: Flight mode for switch position 3 (1361 to 1490)
    // @Values: 0:Manual,1:CIRCLE,2:STABILIZE,3:TRAINING,4:ACRO,5:FBWA,6:FBWB,7:CRUISE,8:AUTOTUNE,10:Auto,11:RTL,12:Loiter,14:AVOID_ADSB,15:Guided,17:QSTABILIZE,18:QHOVER,19:QLOITER,20:QLAND,21:QRTL,22:QAUTOTUNE
    // @User: Standard
    GSCALAR(flight_mode3,           "FLTMODE3",       FLIGHT_MODE_3),

    // @Param: FLTMODE4
    // @DisplayName: FlightMode4
    // @Description: Flight mode for switch position 4 (1491 to 1620)
    // @Values: 0:Manual,1:CIRCLE,2:STABILIZE,3:TRAINING,4:ACRO,5:FBWA,6:FBWB,7:CRUISE,8:AUTOTUNE,10:Auto,11:RTL,12:Loiter,14:AVOID_ADSB,15:Guided,17:QSTABILIZE,18:QHOVER,19:QLOITER,20:QLAND,21:QRTL,22:QAUTOTUNE
    // @User: Standard
    GSCALAR(flight_mode4,           "FLTMODE4",       FLIGHT_MODE_4),

    // @Param: FLTMODE5
    // @DisplayName: FlightMode5
    // @Description: Flight mode for switch position 5 (1621 to 1749)
    // @Values: 0:Manual,1:CIRCLE,2:STABILIZE,3:TRAINING,4:ACRO,5:FBWA,6:FBWB,7:CRUISE,8:AUTOTUNE,10:Auto,11:RTL,12:Loiter,14:AVOID_ADSB,15:Guided,17:QSTABILIZE,18:QHOVER,19:QLOITER,20:QLAND,21:QRTL,22:QAUTOTUNE
    // @User: Standard
    GSCALAR(flight_mode5,           "FLTMODE5",       FLIGHT_MODE_5),

    // @Param: FLTMODE6
    // @DisplayName: FlightMode6
    // @Description: Flight mode for switch position 6 (1750 to 2049)
    // @Values: 0:Manual,1:CIRCLE,2:STABILIZE,3:TRAINING,4:ACRO,5:FBWA,6:FBWB,7:CRUISE,8:AUTOTUNE,10:Auto,11:RTL,12:Loiter,14:AVOID_ADSB,15:Guided,17:QSTABILIZE,18:QHOVER,19:QLOITER,20:QLAND,21:QRTL,22:QAUTOTUNE
    // @User: Standard
    GSCALAR(flight_mode6,           "FLTMODE6",       FLIGHT_MODE_6),

    // @Param: INITIAL_MODE
    // @DisplayName: Initial flight mode
    // @Description: This selects the mode to start in on boot. This is useful for when you want to start in AUTO mode on boot without a receiver.
    // @Values: 0:Manual,1:CIRCLE,2:STABILIZE,3:TRAINING,4:ACRO,5:FBWA,6:FBWB,7:CRUISE,8:AUTOTUNE,10:Auto,11:RTL,12:Loiter,14:AVOID_ADSB,15:Guided,17:QSTABILIZE,18:QHOVER,19:QLOITER,20:QLAND,21:QRTL,22:QAUTOTUNE
    // @User: Advanced
    GSCALAR(initial_mode,        "INITIAL_MODE",     Mode::Number::MANUAL),

    // @Param: LIM_ROLL_CD
    // @DisplayName: Maximum Bank Angle
    // @Description: Maximum bank angle commanded in modes with stabilized limits. Increase this value for sharper turns, but decrease to prevent accelerated stalls.
    // @Units: cdeg
    // @Range: 0 9000
    // @Increment: 1
    // @User: Standard
    ASCALAR(roll_limit_cd,          "LIM_ROLL_CD",    HEAD_MAX_CENTIDEGREE),

    // @Param: LIM_PITCH_MAX
    // @DisplayName: Maximum Pitch Angle
    // @Description: Maximum pitch up angle commanded in modes with stabilized limits.
    // @Units: cdeg
    // @Range: 0 9000
    // @Increment: 1
    // @User: Standard
    ASCALAR(pitch_limit_max_cd,     "LIM_PITCH_MAX",  PITCH_MAX_CENTIDEGREE),

    // @Param: LIM_PITCH_MIN
    // @DisplayName: Minimum Pitch Angle
    // @Description: Maximum pitch down angle commanded in modes with stabilized limits
    // @Units: cdeg
    // @Range: -9000 0
    // @Increment: 1
    // @User: Standard
    ASCALAR(pitch_limit_min_cd,     "LIM_PITCH_MIN",  PITCH_MIN_CENTIDEGREE),

    // @Param: ACRO_ROLL_RATE
    // @DisplayName: ACRO mode roll rate
    // @Description: The maximum roll rate at full stick deflection in ACRO mode
    // @Units: deg/s
    // @Range: 10 500
    // @Increment: 1
    // @User: Standard
    GSCALAR(acro_roll_rate,          "ACRO_ROLL_RATE",    180),

    // @Param: ACRO_PITCH_RATE
    // @DisplayName: ACRO mode pitch rate
    // @Description: The maximum pitch rate at full stick deflection in ACRO mode
    // @Units: deg/s
    // @Range: 10 500
    // @Increment: 1
    // @User: Standard
    GSCALAR(acro_pitch_rate,          "ACRO_PITCH_RATE",  180),

    // @Param: ACRO_LOCKING
    // @DisplayName: ACRO mode attitude locking
    // @Description: Enable attitude locking when sticks are released
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(acro_locking,             "ACRO_LOCKING",     0),

    // @Param: GROUND_STEER_ALT
    // @DisplayName: Ground steer altitude
    // @Description: Altitude at which to use the ground steering controller on the rudder. If non-zero then the STEER2SRV controller will be used to control the rudder for altitudes within this limit of the home altitude.
    // @Units: m
    // @Range: -100 100
    // @Increment: 0.1
    // @User: Standard
    GSCALAR(ground_steer_alt,         "GROUND_STEER_ALT",   0),

    // @Param: GROUND_STEER_DPS
    // @DisplayName: Ground steer rate
    // @Description: Ground steering rate in degrees per second for full rudder stick deflection
    // @Units: deg/s
    // @Range: 10 360
    // @Increment: 1
    // @User: Advanced
    GSCALAR(ground_steer_dps,         "GROUND_STEER_DPS",  90),

    // @Param: TRIM_AUTO
    // @DisplayName: Automatic trim adjustment
    // @Description: Enables the setting SERVOn_TRIM values to current levels when switching out of MANUAL mode. Should not be left on as mode switches while the plane is rolling or pitching can cause invalid trim values and subsequent unstable behavior. See newer and safer SERVO_AUTO_TRIM parameter for automated results.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(auto_trim,              "TRIM_AUTO",      AUTO_TRIM),

    // @Param: MIXING_GAIN
    // @DisplayName: Mixing Gain
    // @Description: The gain for the Vtail and elevon output mixers. The default is 0.5, which ensures that the mixer doesn't saturate, allowing both input channels to go to extremes while retaining control over the output. Hardware mixers often have a 1.0 gain, which gives more servo throw, but can saturate. If you don't have enough throw on your servos with VTAIL_OUTPUT or ELEVON_OUTPUT enabled then you can raise the gain using MIXING_GAIN. The mixer allows outputs in the range 900 to 2100 microseconds.
    // @Range: 0.5 1.2
    // @User: User
    GSCALAR(mixing_gain,            "MIXING_GAIN",    0.5f),

    // @Param: RUDDER_ONLY
    // @DisplayName: Rudder only aircraft
    // @Description: Enable rudder only mode. The rudder will control attitude in attitude controlled modes (such as FBWA). You should setup your transmitter to send roll stick inputs to the RCMAP_YAW channel (normally channel 4). The rudder servo should be attached to the RCMAP_YAW channel as well. Note that automatic ground steering will be disabled for rudder only aircraft. You should also set KFF_RDDRMIX to 1.0. You will also need to setup the YAW2SRV_DAMP yaw damping appropriately for your aircraft. A value of 0.5 for YAW2SRV_DAMP is a good starting point.
    // @Values: 0:Disabled,1:Enabled
    // @User: User
    GSCALAR(rudder_only,             "RUDDER_ONLY",  0),

    // @Param: MIXING_OFFSET
    // @DisplayName: Mixing Offset
    // @Description: The offset for the Vtail and elevon output mixers, as a percentage. This can be used in combination with MIXING_GAIN to configure how the control surfaces respond to input. The response to aileron or elevator input can be increased by setting this parameter to a positive or negative value. A common usage is to enter a positive value to increase the aileron response of the elevons of a flying wing. The default value of zero will leave the aileron-input response equal to the elevator-input response.
    // @Units: d%
    // @Range: -1000 1000
    // @User: User
    GSCALAR(mixing_offset,          "MIXING_OFFSET",  0),

    // @Param: DSPOILR_RUD_RATE
    // @DisplayName: Differential spoilers rudder rate
    // @Description: Sets the amount of deflection that the rudder output will apply to the differential spoilers, as a percentage. The default value of 100 results in full rudder applying full deflection. A value of 0 will result in the differential spoilers exactly following the elevons (no rudder effect).
    // @Units: %
    // @Range: -100 100
    // @User: User
    GSCALAR(dspoiler_rud_rate,      "DSPOILR_RUD_RATE",  DSPOILR_RUD_RATE_DEFAULT),

    // @Param: SYS_NUM_RESETS
    // @DisplayName: Num Resets
    // @Description: Number of APM board resets
    // @ReadOnly: True
    // @User: Advanced
    GSCALAR(num_resets,             "SYS_NUM_RESETS", 0),

    // @Param: LOG_BITMASK
    // @DisplayName: Log bitmask
    // @Description: Bitmap of what on-board log types to enable. This value is made up of the sum of each of the log types you want to be saved. It is usually best just to enable all log types by setting this to 65535. The individual bits are ATTITUDE_FAST=1, ATTITUDE_MEDIUM=2, GPS=4, PerformanceMonitoring=8, ControlTuning=16, NavigationTuning=32, Mode=64, IMU=128, Commands=256, Battery=512, Compass=1024, TECS=2048, Camera=4096, RCandServo=8192, Sonar=16384, Arming=32768, FullLogs=65535
    // @Values: 0:Disabled,65535:All-Default
    // @Bitmask: 0:ATTITUDE_FAST,1:ATTITUDE_MED,2:GPS,3:PM,4:CTUN,5:NTUN,6:MODE,7:IMU,8:CMD,9:CURRENT,10:COMPASS,11:TECS,12:CAMERA,13:RC,14:SONAR,15:ARM/DISARM,19:IMU_RAW
    // @User: Advanced
    GSCALAR(log_bitmask,            "LOG_BITMASK",    DEFAULT_LOG_BITMASK),

    // @Param: RST_SWITCH_CH
    // @DisplayName: Reset Switch Channel
    // @Description: RC channel to use to reset to last flight mode	after geofence takeover.
    // @User: Advanced
    GSCALAR(reset_switch_chan,      "RST_SWITCH_CH",  0),

    // @Param: RST_MISSION_CH
    // @DisplayName: Reset Mission Channel
    // @Description: Enables a channel to reset the mission to the first waypoint. Mission restart is triggered by channel rising above 1750 PWM. 0 disables.
    // @User: Advanced
    GSCALAR(reset_mission_chan,      "RST_MISSION_CH",  0),

    // @Param: TRIM_ARSPD_CM
    // @DisplayName: Target airspeed
    // @Description: Target airspeed in cm/s in automatic throttle modes. Value is as an indicated (calibrated/apparent) airspeed.
    // @Units: cm/s
    // @User: User
    ASCALAR(airspeed_cruise_cm,     "TRIM_ARSPD_CM",  AIRSPEED_CRUISE_CM),

    // @Param: SCALING_SPEED
    // @DisplayName: speed used for speed scaling calculations
    // @Description: Airspeed in m/s to use when calculating surface speed scaling. Note that changing this value will affect all PID values
    // @Units: m/s
    // @User: Advanced
    GSCALAR(scaling_speed,        "SCALING_SPEED",    SCALING_SPEED),

    // @Param: MIN_GNDSPD_CM
    // @DisplayName: Minimum ground speed
    // @Description: Minimum ground speed in cm/s when under airspeed control
    // @Units: cm/s
    // @User: Advanced
    ASCALAR(min_gndspeed_cm,      "MIN_GNDSPD_CM",  MIN_GNDSPEED_CM),

    // @Param: TRIM_PITCH_CD
    // @DisplayName: Pitch angle offset
    // @Description: Offset applied to AHRS pitch used for in-flight pitch trimming. Correct ground leveling is better than changing this parameter.
    // @Units: cdeg
    // @User: Advanced
    GSCALAR(pitch_trim_cd,        "TRIM_PITCH_CD",  0),

    // @Param: ALT_HOLD_RTL
    // @DisplayName: RTL altitude
    // @Description: Target altitude above home for RTL mode. Maintains current altitude if set to -1. Rally point altitudes are used if plane does not return to home.
    // @Units: cm
    // @User: User
    GSCALAR(RTL_altitude_cm,        "ALT_HOLD_RTL",   ALT_HOLD_HOME_CM),

    // @Param: ALT_HOLD_FBWCM
    // @DisplayName: Minimum altitude for FBWB mode
    // @Description: This is the minimum altitude in centimeters that FBWB and CRUISE modes will allow. If you attempt to descend below this altitude then the plane will level off. A value of zero means no limit.
    // @Units: cm
    // @User: User
    GSCALAR(FBWB_min_altitude_cm,   "ALT_HOLD_FBWCM", ALT_HOLD_FBW_CM),

    // @Param: FLAP_IN_CHANNEL
    // @DisplayName: Flap input channel
    // @Description: An RC input channel to use for flaps control. If this is set to a RC channel number then that channel will be used for manual flaps control. When enabled, the percentage of flaps is taken as the percentage travel from the TRIM value of the channel to the MIN value of the channel. A value above the TRIM values will give inverse flaps (spoilers). This option needs to be enabled in conjunction with a FUNCTION setting on an output channel to one of the flap functions. When a FLAP_IN_CHANNEL is combined with auto-flaps the higher of the two flap percentages is taken.
    // @User: User
    GSCALAR(flapin_channel,         "FLAP_IN_CHANNEL",  0),

    // @Param: FLAP_1_PERCNT
    // @DisplayName: Flap 1 percentage
    // @Description: The percentage change in flap position when FLAP_1_SPEED is reached. Use zero to disable flaps
    // @Range: 0 100
    // @Units: %
    // @User: Advanced
    GSCALAR(flap_1_percent,         "FLAP_1_PERCNT",  FLAP_1_PERCENT),

    // @Param: FLAP_1_SPEED
    // @DisplayName: Flap 1 speed
    // @Description: The speed in meters per second at which to engage FLAP_1_PERCENT of flaps. Note that FLAP_1_SPEED should be greater than or equal to FLAP_2_SPEED
    // @Range: 0 100
	// @Increment: 1
    // @Units: m/s
    // @User: Advanced
    GSCALAR(flap_1_speed,           "FLAP_1_SPEED",   FLAP_1_SPEED),

    // @Param: FLAP_2_PERCNT
    // @DisplayName: Flap 2 percentage
    // @Description: The percentage change in flap position when FLAP_2_SPEED is reached. Use zero to disable flaps
    // @Range: 0 100
	// @Units: %
    // @User: Advanced
    GSCALAR(flap_2_percent,         "FLAP_2_PERCNT",  FLAP_2_PERCENT),

    // @Param: FLAP_2_SPEED
    // @DisplayName: Flap 2 speed
    // @Description: The speed in meters per second at which to engage FLAP_2_PERCENT of flaps. Note that FLAP_1_SPEED should be greater than or equal to FLAP_2_SPEED
    // @Range: 0 100
	// @Units: m/s
	// @Increment: 1
    // @User: Advanced
    GSCALAR(flap_2_speed,           "FLAP_2_SPEED",   FLAP_2_SPEED),

#if HAL_WITH_IO_MCU
    // @Param: OVERRIDE_CHAN
    // @DisplayName: IO override channel
    // @Description: If set to a non-zero value then this is an RC input channel number to use for giving IO manual control in case the main FMU microcontroller on a board with a IO co-processor fails. When this RC input channel goes above 1750 the FMU microcontroller will no longer be involved in controlling the servos and instead the IO microcontroller will directly control the servos. Note that IO manual control will be automatically activated if the FMU crashes for any reason. This parameter allows you to test for correct manual behaviour without actually crashing the FMU. This parameter is can be set to a non-zero value either for ground testing purposes or for giving the effect of an external override control board. Please also see the docs on OVERRIDE_SAFETY. Note that you may set OVERRIDE_CHAN to the same channel as FLTMODE_CH to get IO based override when in flight mode 6. Note that when override is triggered due to a FMU crash the 6 auxiliary output channels on Pixhawk will no longer be updated, so all the flight controls you need must be assigned to the first 8 channels.
    // @User: Advanced
    GSCALAR(override_channel,      "OVERRIDE_CHAN",  0),

    // @Param: OVERRIDE_SAFETY
    // @DisplayName: IO override safety switch
    // @Description: This controls whether the safety switch is turned off when you activate override with OVERRIDE_CHAN. When set to 1 the safety switch is de-activated (activating the servos) then a IO override is triggered. In that case the safety remains de-activated after override is disabled. If OVERRIDE_SAFETTY is set to 0 then the safety switch state does not change. Note that regardless of the value of this parameter the servos will be active while override is active.
    // @User: Advanced
    GSCALAR(override_safety,      "OVERRIDE_SAFETY",  1),
#endif

#if HIL_SUPPORT
    // @Param: HIL_MODE
    // @DisplayName: HIL mode enable
    // @Description: This enables and disables hardware in the loop mode. If HIL_MODE is 1 then on the next reboot all sensors are replaced with HIL sensors which come from the GCS.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    // @RebootRequired: True
    GSCALAR(hil_mode,               "HIL_MODE",      0),
#endif

    // @Param: HIL_SERVOS
    // @DisplayName: HIL Servos enable
    // @Description: This controls whether real servo controls are used in HIL mode. If you enable this then the APM will control the real servos in HIL mode. If disabled it will report servo values, but will not output to the real servos. Be careful that your motor and propeller are not connected if you enable this option.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    GSCALAR(hil_servos,            "HIL_SERVOS",      0),

    // @Param: HIL_ERR_LIMIT
    // @DisplayName: Limit of error in HIL attitude before reset
    // @Description: This controls the maximum error in degrees on any axis before HIL will reset the DCM attitude to match the HIL_STATE attitude. This limit will prevent poor timing on HIL from causing a major attitude error. If the value is zero then no limit applies.
    // @Units: deg
    // @Range: 0 90
    // @Increment: 0.1
    // @User: Advanced
    GSCALAR(hil_err_limit,         "HIL_ERR_LIMIT",   5),

    // @Param: RTL_AUTOLAND
    // @DisplayName: RTL auto land
    // @Description: Automatically begin landing sequence after arriving at RTL location. This requires the addition of a DO_LAND_START mission item, which acts as a marker for the start of a landing sequence. The closest landing sequence will be chosen to the current location. 
    // @Values: 0:Disable,1:Enable - go HOME then land,2:Enable - go directly to landing sequence
    // @User: Standard
    GSCALAR(rtl_autoland,         "RTL_AUTOLAND",   0),

    // @Param: CRASH_ACC_THRESH
    // @DisplayName: Crash Deceleration Threshold
    // @Description: X-Axis deceleration threshold to notify the crash detector that there was a possible impact which helps disarm the motor quickly after a crash. This value should be much higher than normal negative x-axis forces during normal flight, check flight log files to determine the average IMU.x values for your aircraft and motor type. Higher value means less sensative (triggers on higher impact). For electric planes that don't vibrate much during fight a value of 25 is good (that's about 2.5G). For petrol/nitro planes you'll want a higher value. Set to 0 to disable the collision detector.
    // @Units: m/s/s
    // @Range: 10 127
    // @User: Advanced
    GSCALAR(crash_accel_threshold,          "CRASH_ACC_THRESH",   0),

    // @Param: CRASH_DETECT
    // @DisplayName: Crash Detection
    // @Description: Automatically detect a crash during AUTO flight and perform the bitmask selected action(s). Disarm will turn off motor for safety and to help against burning out ESC and motor. Setting the mode to manual will help save the servos from burning out by overexerting if the aircraft crashed in an odd orientation such as upsidedown. Set to 0 to disable crash detection.
    // @Bitmask: 0:Disarm
    // @User: Advanced
    ASCALAR(crash_detection_enable,         "CRASH_DETECT",   0),

    // barometer ground calibration. The GND_ prefix is chosen for
    // compatibility with previous releases of ArduPlane
    // @Group: GND_
    // @Path: ../libraries/AP_Baro/AP_Baro.cpp
    GOBJECT(barometer, "GND_", AP_Baro),

    // GPS driver
    // @Group: GPS_
    // @Path: ../libraries/AP_GPS/AP_GPS.cpp
    GOBJECT(gps, "GPS_", AP_GPS),

#if CAMERA == ENABLED
    // @Group: CAM_
    // @Path: ../libraries/AP_Camera/AP_Camera.cpp
    GOBJECT(camera,                  "CAM_", AP_Camera),
#endif

    // @Group: ARMING_
    // @Path: AP_Arming.cpp,../libraries/AP_Arming/AP_Arming.cpp
    GOBJECT(arming,                 "ARMING_", AP_Arming_Plane),

    // @Group: RELAY_
    // @Path: ../libraries/AP_Relay/AP_Relay.cpp
    GOBJECT(relay,                  "RELAY_", AP_Relay),

#if PARACHUTE == ENABLED
	// @Group: CHUTE_
    // @Path: ../libraries/AP_Parachute/AP_Parachute.cpp
    GOBJECT(parachute,		"CHUTE_", AP_Parachute),

    // @Param: CHUTE_CHAN
    // @DisplayName: Parachute release channel
    // @Description: If set to a non-zero value then this is an RC input channel number to use for manually releasing the parachute. When this channel goes above 1700 the parachute will be released
    // @User: Advanced
    GSCALAR(parachute_channel,      "CHUTE_CHAN",  0),
#endif

    // @Group: RNGFND
    // @Path: ../libraries/AP_RangeFinder/RangeFinder.cpp
    GOBJECT(rangefinder,            "RNGFND", RangeFinder),

    // @Param: RNGFND_LANDING
    // @DisplayName: Enable rangefinder for landing
    // @Description: This enables the use of a rangefinder for automatic landing. The rangefinder will be used both on the landing approach and for final flare
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(rangefinder_landing,    "RNGFND_LANDING",   0),

#if AP_TERRAIN_AVAILABLE
    // @Group: TERRAIN_
    // @Path: ../libraries/AP_Terrain/AP_Terrain.cpp
    GOBJECT(terrain,                "TERRAIN_", AP_Terrain),
#endif

    // @Group: ADSB_
    // @Path: ../libraries/AP_ADSB/AP_ADSB.cpp
    GOBJECT(adsb,                "ADSB_", AP_ADSB),

    // @Group: AVD_
    // @Path: ../libraries/AP_Avoidance/AP_Avoidance.cpp
    GOBJECT(avoidance_adsb, "AVD_", AP_Avoidance_Plane),

    // @Group: Q_
    // @Path: quadplane.cpp
    GOBJECT(quadplane,           "Q_", QuadPlane),

    // @Group: TUNE_
    // @Path: tuning.cpp,../libraries/AP_Tuning/AP_Tuning.cpp
    GOBJECT(tuning,           "TUNE_", AP_Tuning_Plane),
    
    // @Group: Q_A_
    // @Path: ../libraries/AC_AttitudeControl/AC_AttitudeControl.cpp,../libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp
    { AP_PARAM_GROUP, "Q_A_", Parameters::k_param_q_attitude_control,
      (const void *)&plane.quadplane.attitude_control,
      {group_info : AC_AttitudeControl_Multi::var_info}, AP_PARAM_FLAG_POINTER },
    
    // @Group: RLL2SRV_
    // @Path: ../libraries/APM_Control/AP_RollController.cpp
	GOBJECT(rollController,         "RLL2SRV_",   AP_RollController),

    // @Group: PTCH2SRV_
    // @Path: ../libraries/APM_Control/AP_PitchController.cpp
	GOBJECT(pitchController,        "PTCH2SRV_",  AP_PitchController),

    // @Group: YAW2SRV_
    // @Path: ../libraries/APM_Control/AP_YawController.cpp
	GOBJECT(yawController,          "YAW2SRV_",   AP_YawController),

    // @Group: STEER2SRV_
    // @Path: ../libraries/APM_Control/AP_SteerController.cpp
	GOBJECT(steerController,        "STEER2SRV_",   AP_SteerController),

	// variables not in the g class which contain EEPROM saved variables

    // @Group: COMPASS_
    // @Path: ../libraries/AP_Compass/AP_Compass.cpp
    GOBJECT(compass,                "COMPASS_",     Compass),

    // @Group: SCHED_
    // @Path: ../libraries/AP_Scheduler/AP_Scheduler.cpp
    GOBJECT(scheduler, "SCHED_", AP_Scheduler),

    // @Group: RCMAP_
    // @Path: ../libraries/AP_RCMapper/AP_RCMapper.cpp
    GOBJECT(rcmap,                "RCMAP_",         RCMapper),

    // @Group: SR0_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs._chan[0], gcs0,        "SR0_",     GCS_MAVLINK),

    // @Group: SR1_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs._chan[1],  gcs1,       "SR1_",     GCS_MAVLINK),

    // @Group: SR2_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs._chan[2],  gcs2,       "SR2_",     GCS_MAVLINK),

    // @Group: SR3_
    // @Path: GCS_Mavlink.cpp
    GOBJECTN(_gcs._chan[3],  gcs3,       "SR3_",     GCS_MAVLINK),

    // @Group: INS_
    // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
    GOBJECT(ins,                    "INS_", AP_InertialSensor),

    // @Group: AHRS_
    // @Path: ../libraries/AP_AHRS/AP_AHRS.cpp
    GOBJECT(ahrs,                   "AHRS_",    AP_AHRS),

    // @Group: ARSPD
    // @Path: ../libraries/AP_Airspeed/AP_Airspeed.cpp
    GOBJECT(airspeed,                               "ARSPD",   AP_Airspeed),

    // @Group: NAVL1_
    // @Path: ../libraries/AP_L1_Control/AP_L1_Control.cpp
    GOBJECT(L1_controller,         "NAVL1_",   AP_L1_Control),

    // @Group: TECS_
    // @Path: ../libraries/AP_TECS/AP_TECS.cpp
    GOBJECT(TECS_controller,         "TECS_",   AP_TECS),

#if MOUNT == ENABLED
    // @Group: MNT
    // @Path: ../libraries/AP_Mount/AP_Mount.cpp
    GOBJECT(camera_mount,           "MNT",  AP_Mount),
#endif

    // @Group: LOG
    // @Path: ../libraries/AP_Logger/AP_Logger.cpp
    GOBJECT(logger,           "LOG",  AP_Logger),

    // @Group: BATT
    // @Path: ../libraries/AP_BattMonitor/AP_BattMonitor.cpp
    GOBJECT(battery,                "BATT", AP_BattMonitor),

    // @Group: BRD_
    // @Path: ../libraries/AP_BoardConfig/AP_BoardConfig.cpp
    GOBJECT(BoardConfig,            "BRD_",       AP_BoardConfig),

#if HAL_WITH_UAVCAN
    // @Group: CAN_
    // @Path: ../libraries/AP_BoardConfig/AP_BoardConfig_CAN.cpp
    GOBJECT(BoardConfig_CAN,        "CAN_",       AP_BoardConfig_CAN),
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // @Group: SIM_
    // @Path: ../libraries/SITL/SITL.cpp
    GOBJECT(sitl, "SIM_", SITL::SITL),
#endif

#if ADVANCED_FAILSAFE == ENABLED
    // @Group: AFS_
    // @Path: ../libraries/AP_AdvancedFailsafe/AP_AdvancedFailsafe.cpp
    GOBJECT(afs,  "AFS_", AP_AdvancedFailsafe),
#endif

#if OPTFLOW == ENABLED
    // @Group: FLOW
    // @Path: ../libraries/AP_OpticalFlow/OpticalFlow.cpp
    GOBJECT(optflow,   "FLOW", OpticalFlow),
#endif

    // @Group: MIS_
    // @Path: ../libraries/AP_Mission/AP_Mission.cpp
    GOBJECT(mission, "MIS_",       AP_Mission),

    // @Group: RALLY_
    // @Path: ../libraries/AP_Rally/AP_Rally.cpp
    GOBJECT(rally,  "RALLY_",       AP_Rally),

#if AP_AHRS_NAVEKF_AVAILABLE
    // @Group: EK2_
    // @Path: ../libraries/AP_NavEKF2/AP_NavEKF2.cpp
    GOBJECTN(EKF2, NavEKF2, "EK2_", NavEKF2),

    // @Group: EK3_
    // @Path: ../libraries/AP_NavEKF3/AP_NavEKF3.cpp
    GOBJECTN(EKF3, NavEKF3, "EK3_", NavEKF3),
#endif

    // @Group: RPM
    // @Path: ../libraries/AP_RPM/AP_RPM.cpp
    GOBJECT(rpm_sensor, "RPM", AP_RPM),
    
    // @Group: RSSI_
    // @Path: ../libraries/AP_RSSI/AP_RSSI.cpp
    GOBJECT(rssi, "RSSI_",  AP_RSSI),

    // @Group: NTF_
    // @Path: ../libraries/AP_Notify/AP_Notify.cpp
    GOBJECT(notify, "NTF_",  AP_Notify),

    // @Group: 
    // @Path: Parameters.cpp
    GOBJECT(g2, "",  ParametersG2),
    
    // @Group: LAND_
    // @Path: ../libraries/AP_Landing/AP_Landing.cpp
    GOBJECT(landing, "LAND_", AP_Landing),

#if OSD_ENABLED
    // @Group: OSD
    // @Path: ../libraries/AP_OSD/AP_OSD.cpp
    GOBJECT(osd, "OSD", AP_OSD),
#endif
    
    AP_VAREND
};

/*
  2nd group of parameters
 */
const AP_Param::GroupInfo ParametersG2::var_info[] = {

    // @Group: BTN_
    // @Path: ../libraries/AP_Button/AP_Button.cpp
    AP_SUBGROUPINFO(button, "BTN_", 1, ParametersG2, AP_Button),

    // @Group: ICE_
    // @Path: ../libraries/AP_ICEngine/AP_ICEngine.cpp
    AP_SUBGROUPINFO(ice_control, "ICE_", 2, ParametersG2, AP_ICEngine),

    // 3 was used by prototype for servo_channels
    
    // @Param: SYSID_ENFORCE
    // @DisplayName: GCS sysid enforcement
    // @Description: This controls whether packets from other than the expected GCS system ID will be accepted
    // @Values: 0:NotEnforced,1:Enforced
    // @User: Advanced
    AP_GROUPINFO("SYSID_ENFORCE", 4, ParametersG2, sysid_enforce, 0),
#if STATS_ENABLED == ENABLED
    // @Group: STAT
    // @Path: ../libraries/AP_Stats/AP_Stats.cpp
    AP_SUBGROUPINFO(stats, "STAT", 5, ParametersG2, AP_Stats),
#endif

    // @Group: SERVO
    // @Path: ../libraries/SRV_Channel/SRV_Channels.cpp
    AP_SUBGROUPINFO(servo_channels, "SERVO", 6, ParametersG2, SRV_Channels),

    // @Group: RC
    // @Path: ../libraries/RC_Channel/RC_Channels_VarInfo.h
    AP_SUBGROUPINFO(rc_channels, "RC", 7, ParametersG2, RC_Channels_Plane),
    
#if SOARING_ENABLED == ENABLED
    // @Group: SOAR_
    // @Path: ../libraries/AP_Soaring/AP_Soaring.cpp
    AP_SUBGROUPINFO(soaring_controller, "SOAR_", 8, ParametersG2, SoaringController),
#endif
  
    // @Param: RUDD_DT_GAIN
    // @DisplayName: rudder differential thrust gain
    // @Description: gain control from rudder to differential thrust
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RUDD_DT_GAIN", 9, ParametersG2, rudd_dt_gain, 10),

    // @Param: MANUAL_RCMASK
    // @DisplayName: Manual R/C pass-through mask
    // @Description: Mask of R/C channels to pass directly to corresponding output channel when in MANUAL mode. When in any mode except MANUAL the channels selected with this option behave normally. This parameter is designed to allow for complex mixing strategies to be used for MANUAL flight using transmitter based mixing. Note that when this option is used you need to be very careful with pre-flight checks to ensure that the output is correct both in MANUAL and non-MANUAL modes.
    // @Bitmask: 0:Chan1,1:Chan2,2:Chan3,3:Chan4,4:Chan5,5:Chan6,6:Chan7,7:Chan8,8:Chan9,9:Chan10,10:Chan11,11:Chan12,12:Chan13,13:Chan14,14:Chan15,15:Chan16
    // @User: Advanced
    AP_GROUPINFO("MANUAL_RCMASK", 10, ParametersG2, manual_rc_mask, 0),
    
    // @Param: HOME_RESET_ALT
    // @DisplayName: Home reset altitude threshold
    // @Description: When the aircraft is within this altitude of the home waypoint, while disarmed it will automatically update the home position. Set to 0 to continously reset it.
    // @Values: -1:Never reset,0:Always reset
    // @Range: -1 127
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("HOME_RESET_ALT", 11, ParametersG2, home_reset_threshold, 0),

#if GRIPPER_ENABLED == ENABLED
    // @Group: GRIP_
    // @Path: ../libraries/AP_Gripper/AP_Gripper.cpp
    AP_SUBGROUPINFO(gripper, "GRIP_", 12, ParametersG2, AP_Gripper),
#endif

    // @Param: FLIGHT_OPTIONS
    // @DisplayName: Flight mode options
    // @Description: Flight mode specific options
    // @Bitmask: 0:Rudder mixing in direct flight modes only (Manual / Stabilize / Acro),1:Use centered throttle in Cruise or FBWB to indicate trim airspeed, 2:Disable attitude check for takeoff arming, 3:Force target airspeed to trim airspeed in Cruise or FBWB
    // @User: Advanced
    AP_GROUPINFO("FLIGHT_OPTIONS", 13, ParametersG2, flight_options, 0),

#ifdef ENABLE_SCRIPTING
    // Scripting is intentionally not showing up in the parameter docs until it is a more standard feature
    AP_SUBGROUPINFO(scripting, "SCR_", 14, ParametersG2, AP_Scripting),
#endif

    // @Param: TKOFF_ACCEL_CNT
    // @DisplayName: Takeoff throttle acceleration count
    // @Description: This is the number of acceleration events to require for arming with TKOFF_THR_MINACC. The default is 1, which means a single forward acceleration above TKOFF_THR_MINACC will arm. By setting this higher than 1 you can require more forward/backward movements to arm.
    // @Range: 1 10
    // @User: User
    AP_GROUPINFO("TKOFF_ACCEL_CNT", 15, ParametersG2, takeoff_throttle_accel_count, 1),

#if LANDING_GEAR_ENABLED == ENABLED
    // @Group: LGR_
    // @Path: ../libraries/AP_LandingGear/AP_LandingGear.cpp
    AP_SUBGROUPINFO(landing_gear, "LGR_", 16, ParametersG2, AP_LandingGear),
#endif

    // @Param: DSPOILER_CROW_W1
    // @DisplayName: Differential spoiler crow flaps outer weight
    // @Description: This is amount of deflection applied to the two outer surfaces for differential spoilers for flaps to give crow flaps. It is a number from 0 to 100. At zero no crow flaps are applied. A recommended starting value is 25.
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("DSPOILER_CROW_W1", 17, ParametersG2, crow_flap_weight_outer, 0),

    // @Param: DSPOILER_CROW_W2
    // @DisplayName: Differential spoiler crow flaps inner weight
    // @Description: This is amount of deflection applied to the two inner surfaces for differential spoilers for flaps to give crow flaps. It is a number from 0 to 100. At zero no crow flaps are applied. A recommended starting value is 45.
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("DSPOILER_CROW_W2", 18, ParametersG2, crow_flap_weight_inner, 0),

    // @Param: TKOFF_TIMEOUT
    // @DisplayName: Takeoff timeout
    // @Description: This is the timeout for an automatic takeoff. If this is non-zero and the aircraft does not reach a ground speed of at least 4 m/s within this number of seconds then the takeoff is aborted and the vehicle disarmed. If the value is zero then no timeout applies.
    // @Range: 0 120
    // @Increment: 1
    // @Units: s
    // @User: User
    AP_GROUPINFO("TKOFF_TIMEOUT", 19, ParametersG2, takeoff_timeout, 0),

    // @Param: DSPOILER_OPTS
    // @DisplayName: Differential spoiler and crow flaps options
    // @Description: Differential spoiler and crow flaps options
    // @Values: 0: none, 1: D spoilers have pitch input, 2: use both control surfaces on each wing for roll, 4: Progressive crow, flaps only first (0-50% flap in) then crow flaps (50 - 100% flap in)
    // @Bitmask: 0:pitch control, 1:full span, 2:Progressive crow
    // @User: Advanced
    AP_GROUPINFO("DSPOILER_OPTS", 20, ParametersG2, crow_flap_options, 3),

    // @Param: DSPOILER_AILMTCH
    // @DisplayName: Differential spoiler aileron matching
    // @Description: This scales down the inner flaps so less than full downwards range can be used for differential spoiler and full span ailerons, 100 is use full range, upwards travel is unaffected
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("DSPOILER_AILMTCH", 21, ParametersG2, crow_flap_aileron_matching, 100),

    AP_GROUPEND
};

ParametersG2::ParametersG2(void) :
    ice_control(plane.rpm_sensor)
#if SOARING_ENABLED == ENABLED
    ,soaring_controller(plane.ahrs, plane.TECS_controller, plane.aparm)
#endif
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  This is a conversion table from old parameter values to new
  parameter names. The startup code looks for saved values of the old
  parameters and will copy them across to the new parameters if the
  new parameter does not yet have a saved value. It then saves the new
  value.
  
  Note that this works even if the old parameter has been removed. It
  relies on the old k_param index not being removed
  
  The second column below is the index in the var_info[] table for the
  old object. This should be zero for top level parameters.
 */
const AP_Param::ConversionInfo conversion_table[] = {
    { Parameters::k_param_log_bitmask_old,    0,      AP_PARAM_INT16, "LOG_BITMASK" },
    { Parameters::k_param_rally_limit_km_old, 0,      AP_PARAM_FLOAT, "RALLY_LIMIT_KM" },
    { Parameters::k_param_rally_total_old,    0,      AP_PARAM_INT8, "RALLY_TOTAL" },
    { Parameters::k_param_serial0_baud,       0,      AP_PARAM_INT16, "SERIAL0_BAUD" },
    { Parameters::k_param_serial1_baud,       0,      AP_PARAM_INT16, "SERIAL1_BAUD" },
    { Parameters::k_param_serial2_baud,       0,      AP_PARAM_INT16, "SERIAL2_BAUD" },

    // these are needed to cope with the change to treat nested index 0 as index 63
    { Parameters::k_param_quadplane,          3,      AP_PARAM_FLOAT, "Q_RT_RLL_P" },
    { Parameters::k_param_quadplane,          4,      AP_PARAM_FLOAT, "Q_RT_PIT_P" },
    { Parameters::k_param_quadplane,          5,      AP_PARAM_FLOAT, "Q_RT_YAW_P" },

    { Parameters::k_param_quadplane,          6,      AP_PARAM_FLOAT, "Q_STB_R_P" },
    { Parameters::k_param_quadplane,          7,      AP_PARAM_FLOAT, "Q_STB_P_P" },
    { Parameters::k_param_quadplane,          8,      AP_PARAM_FLOAT, "Q_STB_Y_P" },

    { Parameters::k_param_quadplane,         12,      AP_PARAM_FLOAT, "Q_PZ_P" },
    { Parameters::k_param_quadplane,         13,      AP_PARAM_FLOAT, "Q_PXY_P" },
    { Parameters::k_param_quadplane,         14,      AP_PARAM_FLOAT, "Q_VXY_P" },
    { Parameters::k_param_quadplane,         15,      AP_PARAM_FLOAT, "Q_VZ_P" },
    { Parameters::k_param_quadplane,         16,      AP_PARAM_FLOAT, "Q_AZ_P" },

    { Parameters::k_param_land_slope_recalc_shallow_threshold,0,AP_PARAM_FLOAT, "LAND_SLOPE_RCALC" },
    { Parameters::k_param_land_slope_recalc_steep_threshold_to_abort,0,AP_PARAM_FLOAT, "LAND_ABORT_DEG" },
    { Parameters::k_param_land_pitch_cd,      0,      AP_PARAM_INT16, "LAND_PITCH_CD" },
    { Parameters::k_param_land_flare_alt,     0,      AP_PARAM_FLOAT, "LAND_FLARE_ALT" },
    { Parameters::k_param_land_flare_sec,     0,      AP_PARAM_FLOAT, "LAND_FLARE_SEC" },
    { Parameters::k_param_land_pre_flare_sec, 0,      AP_PARAM_FLOAT, "LAND_PF_SEC" },
    { Parameters::k_param_land_pre_flare_alt, 0,      AP_PARAM_FLOAT, "LAND_PF_ALT" },
    { Parameters::k_param_land_pre_flare_airspeed, 0, AP_PARAM_FLOAT, "LAND_PF_ARSPD" },
    { Parameters::k_param_land_throttle_slewrate, 0,  AP_PARAM_INT8,  "LAND_THR_SLEW" },
    { Parameters::k_param_land_disarm_delay,  0,      AP_PARAM_INT8,  "LAND_DISARMDELAY" },
    { Parameters::k_param_land_then_servos_neutral,0, AP_PARAM_INT8,  "LAND_THEN_NEUTRAL" },
    { Parameters::k_param_land_abort_throttle_enable,0,AP_PARAM_INT8, "LAND_ABORT_THR" },
    { Parameters::k_param_land_flap_percent,  0,      AP_PARAM_INT8,  "LAND_FLAP_PERCENT" },

    // battery failsafes
    { Parameters::k_param_fs_batt_voltage,    0,      AP_PARAM_FLOAT, "BATT_LOW_VOLT" },
    { Parameters::k_param_fs_batt_mah,        0,      AP_PARAM_FLOAT, "BATT_LOW_MAH" },

    { Parameters::k_param_arming,             3,      AP_PARAM_INT8,  "ARMING_RUDDER" },

    { Parameters::k_param_compass_enabled_deprecated,       0,      AP_PARAM_INT8, "COMPASS_ENABLE" },
};

void Plane::load_parameters(void)
{
    if (!AP_Param::check_var_info()) {
        hal.console->printf("Bad parameter table\n");
        AP_HAL::panic("Bad parameter table");
    }
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

    uint32_t before = micros();
    // Load all auto-loaded EEPROM variables
    AP_Param::load_all();
    AP_Param::convert_old_parameters(&conversion_table[0], ARRAY_SIZE(conversion_table));

    // setup defaults in SRV_Channels
    g2.servo_channels.set_default_function(CH_1, SRV_Channel::k_aileron);
    g2.servo_channels.set_default_function(CH_2, SRV_Channel::k_elevator);
    g2.servo_channels.set_default_function(CH_3, SRV_Channel::k_throttle);
    g2.servo_channels.set_default_function(CH_4, SRV_Channel::k_rudder);
        
    const uint8_t old_rc_keys[14] = { Parameters::k_param_rc_1_old,  Parameters::k_param_rc_2_old,
                                      Parameters::k_param_rc_3_old,  Parameters::k_param_rc_4_old,
                                      Parameters::k_param_rc_5_old,  Parameters::k_param_rc_6_old,
                                      Parameters::k_param_rc_7_old,  Parameters::k_param_rc_8_old,
                                      Parameters::k_param_rc_9_old,  Parameters::k_param_rc_10_old,
                                      Parameters::k_param_rc_11_old, Parameters::k_param_rc_12_old,
                                      Parameters::k_param_rc_13_old, Parameters::k_param_rc_14_old };
    const uint16_t old_aux_chan_mask = 0x3FF0;
    SRV_Channels::upgrade_parameters(old_rc_keys, old_aux_chan_mask, &rcmap);

    // possibly convert elevon and vtail mixers
    convert_mixers();
    
    if (quadplane.enable) {
        // quadplanes needs a higher loop rate
        AP_Param::set_default_by_name("SCHED_LOOP_RATE", 300);
    }

    AP_Param::set_frame_type_flags(AP_PARAM_FRAME_PLANE);

    hal.console->printf("load_all took %uus\n", (unsigned)(micros() - before));
}

/*
  convert from old ELEVON_OUTPUT and VTAIL_OUTPUT mixers to function
  based mixing
 */
void Plane::convert_mixers(void)
{
    AP_Int8 elevon_output;
    AP_Int8 vtail_output;
    AP_Param::ConversionInfo elevon_info = {
        Parameters::k_param_elevon_output,
        0,
        AP_PARAM_INT8,
        nullptr
    };
    AP_Param::ConversionInfo vtail_info = {
        Parameters::k_param_vtail_output,
        0,
        AP_PARAM_INT8,
        nullptr
    };
    SRV_Channel *chan1 = SRV_Channels::srv_channel(CH_1);
    SRV_Channel *chan2 = SRV_Channels::srv_channel(CH_2);
    SRV_Channel *chan4 = SRV_Channels::srv_channel(CH_4);

    if (AP_Param::find_old_parameter(&vtail_info, &vtail_output) &&
        vtail_output.get() != 0 &&
        chan2->get_function() == SRV_Channel::k_elevator &&
        chan4->get_function() == SRV_Channel::k_rudder &&
        !chan2->function_configured() &&
        !chan4->function_configured()) {
        hal.console->printf("Converting vtail_output %u\n", vtail_output.get());
        switch (vtail_output) {
        case MIXING_UPUP:
        case MIXING_UPUP_SWP:
            chan2->reversed_set_and_save_ifchanged(false);
            chan4->reversed_set_and_save_ifchanged(false);
            break;
        case MIXING_UPDN:
        case MIXING_UPDN_SWP:
            chan2->reversed_set_and_save_ifchanged(false);
            chan4->reversed_set_and_save_ifchanged(true);
            break;
        case MIXING_DNUP:
        case MIXING_DNUP_SWP:
            chan2->reversed_set_and_save_ifchanged(true);
            chan4->reversed_set_and_save_ifchanged(false);
            break;
        case MIXING_DNDN:
        case MIXING_DNDN_SWP:
            chan2->reversed_set_and_save_ifchanged(true);
            chan4->reversed_set_and_save_ifchanged(true);
            break;
        }
        if (vtail_output < MIXING_UPUP_SWP) {
            chan2->function_set_and_save(SRV_Channel::k_vtail_right);
            chan4->function_set_and_save(SRV_Channel::k_vtail_left);
        } else {
            chan2->function_set_and_save(SRV_Channel::k_vtail_left);
            chan4->function_set_and_save(SRV_Channel::k_vtail_right);
        }
    } else if (AP_Param::find_old_parameter(&elevon_info, &elevon_output) &&
        elevon_output.get() != 0 &&
        chan1->get_function() == SRV_Channel::k_aileron &&
        chan2->get_function() == SRV_Channel::k_elevator &&
        !chan1->function_configured() &&
        !chan2->function_configured()) {
        hal.console->printf("convert elevon_output %u\n", elevon_output.get());
        switch (elevon_output) {
        case MIXING_UPUP:
        case MIXING_UPUP_SWP:
            chan2->reversed_set_and_save_ifchanged(false);
            chan1->reversed_set_and_save_ifchanged(false);
            break;
        case MIXING_UPDN:
        case MIXING_UPDN_SWP:
            chan2->reversed_set_and_save_ifchanged(false);
            chan1->reversed_set_and_save_ifchanged(true);
            break;
        case MIXING_DNUP:
        case MIXING_DNUP_SWP:
            chan2->reversed_set_and_save_ifchanged(true);
            chan1->reversed_set_and_save_ifchanged(false);
            break;
        case MIXING_DNDN:
        case MIXING_DNDN_SWP:
            chan2->reversed_set_and_save_ifchanged(true);
            chan1->reversed_set_and_save_ifchanged(true);
            break;
        }
        if (elevon_output < MIXING_UPUP_SWP) {
            chan1->function_set_and_save(SRV_Channel::k_elevon_right);
            chan2->function_set_and_save(SRV_Channel::k_elevon_left);
        } else {
            chan1->function_set_and_save(SRV_Channel::k_elevon_left);
            chan2->function_set_and_save(SRV_Channel::k_elevon_right);
        }
    }

    // convert any k_aileron_with_input to aileron and k_elevator_with_input to k_elevator
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        SRV_Channel *chan = SRV_Channels::srv_channel(i);
        if (chan->get_function() == SRV_Channel::k_aileron_with_input) {
            chan->function_set_and_save(SRV_Channel::k_aileron);
        } else if (chan->get_function() == SRV_Channel::k_elevator_with_input) {
            chan->function_set_and_save(SRV_Channel::k_elevator);
        }
    }
    
}
