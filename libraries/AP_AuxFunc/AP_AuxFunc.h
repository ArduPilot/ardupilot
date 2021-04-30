#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

class AP_AuxFunc {

public:

    // Aux Switch enumeration
    enum class Function {
        DO_NOTHING =           0, // aux switch disabled
        FLIP =                 2, // flip
        SIMPLE_MODE =          3, // change to simple mode
        RTL =                  4, // change to RTL flight mode
        SAVE_TRIM =            5, // save current position as level
        SAVE_WP =              7, // save mission waypoint or RTL if in auto mode
        CAMERA_TRIGGER =       9, // trigger camera servo or relay
        RANGEFINDER =         10, // allow enabling or disabling rangefinder in flight which helps avoid surface tracking when you are far above the ground
        FENCE =               11, // allow enabling or disabling fence in flight
        RESETTOARMEDYAW =     12, // UNUSED
        SUPERSIMPLE_MODE =    13, // change to simple mode in middle, super simple at top
        ACRO_TRAINER =        14, // low = disabled, middle = leveled, high = leveled and limited
        SPRAYER =             15, // enable/disable the crop sprayer
        AUTO =                16, // change to auto flight mode
        AUTOTUNE =            17, // auto tune
        LAND =                18, // change to LAND flight mode
        GRIPPER =             19, // Operate cargo grippers low=off, middle=neutral, high=on
        PARACHUTE_ENABLE  =   21, // Parachute enable/disable
        PARACHUTE_RELEASE =   22, // Parachute release
        PARACHUTE_3POS =      23, // Parachute disable, enable, release with 3 position switch
        MISSION_RESET =       24, // Reset auto mission to start from first command
        ATTCON_FEEDFWD =      25, // enable/disable the roll and pitch rate feed forward
        ATTCON_ACCEL_LIM =    26, // enable/disable the roll, pitch and yaw accel limiting
        RETRACT_MOUNT =       27, // Retract Mount
        RELAY =               28, // Relay pin on/off (only supports first relay)
        LANDING_GEAR =        29, // Landing gear controller
        LOST_VEHICLE_SOUND =  30, // Play lost vehicle sound
        MOTOR_ESTOP =         31, // Emergency Stop Switch
        MOTOR_INTERLOCK =     32, // Motor On/Off switch
        BRAKE =               33, // Brake flight mode
        RELAY2 =              34, // Relay2 pin on/off
        RELAY3 =              35, // Relay3 pin on/off
        RELAY4 =              36, // Relay4 pin on/off
        THROW =               37, // change to THROW flight mode
        AVOID_ADSB =          38, // enable AP_Avoidance library
        PRECISION_LOITER =    39, // enable precision loiter
        AVOID_PROXIMITY =     40, // enable object avoidance using proximity sensors (ie. horizontal lidar)
        ARMDISARM =           41, // arm or disarm vehicle
        SMART_RTL =           42, // change to SmartRTL flight mode
        INVERTED  =           43, // enable inverted flight
        WINCH_ENABLE =        44, // winch enable/disable
        WINCH_CONTROL =       45, // winch control
        RC_OVERRIDE_ENABLE =  46, // enable RC Override
        USER_FUNC1 =          47, // user function #1
        USER_FUNC2 =          48, // user function #2
        USER_FUNC3 =          49, // user function #3
        LEARN_CRUISE =        50, // learn cruise throttle (Rover)
        MANUAL       =        51, // manual mode
        ACRO         =        52, // acro mode
        STEERING     =        53, // steering mode
        HOLD         =        54, // hold mode
        GUIDED       =        55, // guided mode
        LOITER       =        56, // loiter mode
        FOLLOW       =        57, // follow mode
        CLEAR_WP     =        58, // clear waypoints
        SIMPLE       =        59, // simple mode
        ZIGZAG       =        60, // zigzag mode
        ZIGZAG_SaveWP =       61, // zigzag save waypoint
        COMPASS_LEARN =       62, // learn compass offsets
        SAILBOAT_TACK =       63, // rover sailboat tack
        REVERSE_THROTTLE =    64, // reverse throttle input
        GPS_DISABLE  =        65, // disable GPS for testing
        RELAY5 =              66, // Relay5 pin on/off
        RELAY6 =              67, // Relay6 pin on/off
        STABILIZE =           68, // stabilize mode
        POSHOLD   =           69, // poshold mode
        ALTHOLD   =           70, // althold mode
        FLOWHOLD  =           71, // flowhold mode
        CIRCLE    =           72, // circle mode
        DRIFT     =           73, // drift mode
        SAILBOAT_MOTOR_3POS = 74, // Sailboat motoring 3pos
        SURFACE_TRACKING =    75, // Surface tracking upwards or downwards
        STANDBY  =            76, // Standby mode
        TAKEOFF   =           77, // takeoff
        RUNCAM_CONTROL =      78, // control RunCam device
        RUNCAM_OSD_CONTROL =  79, // control RunCam OSD
        VISODOM_CALIBRATE  =  80, // calibrate visual odometry camera's attitude
        DISARM =              81, // disarm vehicle
        Q_ASSIST =            82, // disable, enable and force Q assist
        ZIGZAG_Auto =         83, // zigzag auto switch
        AIRMODE =             84, // enable / disable airmode for copter
        GENERATOR   =         85, // generator control
        TER_DISABLE =         86, // disable terrain following in CRUISE/FBWB modes
        CROW_SELECT =         87, // select CROW mode for diff spoilers;high disables,mid forces progressive
        SOARING =             88, // three-position switch to set soaring mode
        LANDING_FLARE =       89, // force flare, throttle forced idle, pitch to LAND_PITCH_CD, tilts up
        EKF_POS_SOURCE =      90, // change EKF position source between primary, secondary and tertiary sources
        ARSPD_CALIBRATE=      91, // calibrate airspeed ratio 
        FBWA =                92, // Fly-By-Wire-A
        RELOCATE_MISSION =    93, // used in separate branch MISSION_RELATIVE
        VTX_POWER =           94, // VTX power level
        FBWA_TAILDRAGGER =    95, // enables FBWA taildragger takeoff mode. Once this feature is enabled it will stay enabled until the aircraft goes above TKOFF_TDRAG_SPD1 airspeed, changes mode, or the pitch goes above the initial pitch when this is engaged or goes below 0 pitch. When enabled the elevator will be forced to TKOFF_TDRAG_ELEV. This option allows for easier takeoffs on taildraggers in FBWA mode, and also makes it easier to test auto-takeoff steering handling in FBWA.
        MODE_SWITCH_RESET =   96, // trigger re-reading of mode switch
        WIND_VANE_DIR_OFSSET= 97, // flag for windvane direction offset input, used with windvane type 2

        // entries from 100 onwards are expected to be developer
        // options used for testing
        KILL_IMU1 =          100, // disable first IMU (for IMU failure testing)
        KILL_IMU2 =          101, // disable second IMU (for IMU failure testing)
        CAM_MODE_TOGGLE =    102, // Momentary switch to cycle camera modes
        EKF_LANE_SWITCH =    103, // trigger lane switch attempt
        EKF_YAW_RESET =      104, // trigger yaw reset attempt
        GPS_DISABLE_YAW =    105, // disable GPS yaw for testing
        DISABLE_AIRSPEED_USE = 106, // equivalent to AIRSPEED_USE 0
        // if you add something here, make sure to update the documentation of the parameter in RC_Channel.cpp!
        // also, if you add an option >255, you will need to fix duplicate_options_exist

        // inputs from 200 will eventually used to replace RCMAP
        ROLL =               201, // roll input
        PITCH =              202, // pitch input
        THROTTLE =           203, // throttle pilot input
        YAW =                204, // yaw pilot input
        MAINSAIL =           207, // mainsail input
        FLAP =               208, // flap input
        FWD_THR =            209, // VTOL manual forward throttle
        AIRBRAKE =           210, // manual airbrake control
        WALKING_HEIGHT =     211, // walking robot height input

        // inputs for the use of onboard lua scripting
        SCRIPTING_1 =        300,
        SCRIPTING_2 =        301,
        SCRIPTING_3 =        302,
        SCRIPTING_4 =        303,
        SCRIPTING_5 =        304,
        SCRIPTING_6 =        305,
        SCRIPTING_7 =        306,
        SCRIPTING_8 =        307,
    };

    // auxillary switch handling (n.b.: we store this as 2-bits!):
    enum class SwitchPos : uint8_t {
        LOW,       // indicates auxiliary switch is in the low position (pwm <1200)
        MIDDLE,    // indicates auxiliary switch is in the middle position (pwm >1200, <1800)
        HIGH       // indicates auxiliary switch is in the high position (pwm >1800)
    };

    enum class TriggerSource : uint8_t {
        INIT,
        RC,
        BUTTON,
        MAVLINK,
        MISSION,
        SCRIPTING,
    };

    AP_AuxFunc() {
        _singleton = this;
    }

    static AP_AuxFunc *get_singleton();

    virtual bool init_function(Function function, SwitchPos);

    // wrapper function around do_aux_function which allows us to log
    bool run_function(Function function, SwitchPos pos, TriggerSource source);

    // // method for other parts of the system (e.g. Button and mavlink)
    // // to trigger auxillary functions
    // bool run_aux_function(AP_AuxFunc::Function function, AP_AuxFunc::SwitchPos pos, AP_AuxFunc::TriggerSource source) {
    //     return rc_channel(0)->run_aux_function(function, pos, source);
    // }



#if !HAL_MINIMIZE_FEATURES
    static const char *string_for_function(Function function);
#endif

    virtual bool do_function(Function function, SwitchPos);
    virtual void do_function_armdisarm(const SwitchPos pos);
    virtual void do_function_mission_reset(const SwitchPos pos);

private:

    static AP_AuxFunc *_singleton;

    void do_function_avoid_adsb(const SwitchPos pos);
    void do_function_avoid_proximity(const SwitchPos pos);
    void do_function_camera_trigger(const SwitchPos pos);
    void do_function_runcam_control(const SwitchPos pos);
    void do_function_runcam_osd_control(const SwitchPos pos);
    void do_function_fence(const SwitchPos pos);
    void do_function_clear_wp(const SwitchPos pos);
    void do_function_gripper(const SwitchPos pos);
    void do_function_lost_vehicle_sound(const SwitchPos pos);
    void do_function_rc_override_enable(const SwitchPos pos);
    void do_function_relay(uint8_t relay, bool val);
    void do_function_sprayer(const SwitchPos pos);
    void do_function_generator(const SwitchPos pos);

#if !HAL_MINIMIZE_FEATURES
    // Structure to lookup switch change announcements
    struct LookupTable{
       AP_AuxFunc::Function option;
       const char *announcement;
    };

    static const LookupTable lookuptable[];
#endif
};

namespace AP {
    AP_AuxFunc &auxfunc();
};
