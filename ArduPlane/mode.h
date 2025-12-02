#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Common/Location.h>
#include <stdint.h>
#include <AP_Soaring/AP_Soaring.h>
#include <AP_ADSB/AP_ADSB.h>
#include <AP_Vehicle/ModeReason.h>
#include "quadplane.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Mission/AP_Mission.h>
#include "config.h"
#include "pullup.h"
#include "systemid.h"

#ifndef AP_QUICKTUNE_ENABLED
#define AP_QUICKTUNE_ENABLED HAL_QUADPLANE_ENABLED
#endif

#ifndef MODE_AUTOLAND_ENABLED
#define MODE_AUTOLAND_ENABLED 1
#endif

#include <AP_Quicktune/AP_Quicktune.h>

class AC_PosControl;
class AC_AttitudeControl_Multi;
class AC_Loiter;
class Mode
{
public:

    /* Do not allow copies */
    CLASS_NO_COPY(Mode);

    // Auto Pilot modes
    // ----------------
    enum Number : uint8_t {
        MANUAL        = 0,
        CIRCLE        = 1,
        STABILIZE     = 2,
        TRAINING      = 3,
        ACRO          = 4,
        FLY_BY_WIRE_A = 5,
        FLY_BY_WIRE_B = 6,
        CRUISE        = 7,
        AUTOTUNE      = 8,
        AUTO          = 10,
        RTL           = 11,
        LOITER        = 12,
        TAKEOFF       = 13,
        AVOID_ADSB    = 14,
        GUIDED        = 15,
        INITIALISING  = 16,
#if HAL_QUADPLANE_ENABLED
        QSTABILIZE    = 17,
        QHOVER        = 18,
        QLOITER       = 19,
        QLAND         = 20,
        QRTL          = 21,
#if QAUTOTUNE_ENABLED
        QAUTOTUNE     = 22,
#endif
        QACRO         = 23,
#endif
        THERMAL       = 24,
#if HAL_QUADPLANE_ENABLED
        LOITER_ALT_QLAND = 25,
#endif
#if MODE_AUTOLAND_ENABLED
        AUTOLAND      = 26,
#endif

    // Mode number 30 reserved for "offboard" for external/lua control.
    };

    // Constructor
    Mode();

    // enter this mode, always returns true/success
    bool enter();

    // perform any cleanups required:
    void exit();

    // run controllers specific to this mode
    virtual void run();

    // returns a unique number specific to this mode
    virtual Number mode_number() const = 0;

    // returns full text name
    virtual const char *name() const = 0;

    // returns a string for this flightmode, exactly 4 bytes
    virtual const char *name4() const = 0;

    // returns true if the vehicle can be armed in this mode
    bool pre_arm_checks(size_t buflen, char *buffer) const;

    // Reset rate and steering and TECS controllers
    void reset_controllers();

    //
    // methods that sub classes should override to affect movement of the vehicle in this mode
    //

    // convert user input to targets, implement high level control for this mode
    virtual void update() = 0;

    // true for all q modes
    virtual bool is_vtol_mode() const { return false; }
    virtual bool is_vtol_man_throttle() const;
    virtual bool is_vtol_man_mode() const { return false; }

    // guided or adsb mode
    virtual bool is_guided_mode() const { return false; }

    // true if mode can have terrain following disabled by switch
    virtual bool allows_terrain_disable() const { return false; }

    // true if automatic switch to thermal mode is supported.
    virtual bool does_automatic_thermal_switch() const {return false; }

    // subclasses override this if they require navigation.
    virtual void navigate() { return; }

    // this allows certain flight modes to mix RC input with throttle
    // depending on airspeed_nudge_cm
    virtual bool allows_throttle_nudging() const { return false; }

    // true if the mode sets the vehicle destination, which controls
    // whether control input is ignored with STICK_MIXING=0
    virtual bool does_auto_navigation() const { return false; }

    // true if the mode sets the vehicle destination, which controls
    // whether control input is ignored with STICK_MIXING=0
    virtual bool does_auto_throttle() const { return false; }
    
    // true if the mode supports autotuning (via switch for modes other
    // that AUTOTUNE itself
    virtual bool mode_allows_autotuning() const { return false; }

    // method for mode specific target altitude profiles
    virtual void update_target_altitude();

    // handle a guided target request from GCS
    virtual bool handle_guided_request(Location target_loc) { return false; }

    // true if is landing 
    virtual bool is_landing() const { return false; }

    // true if is taking 
    virtual bool is_taking_off() const;

    // true if throttle min/max limits should be applied
    virtual bool use_throttle_limits() const;

    // true if voltage correction should be applied to throttle
    virtual bool use_battery_compensation() const;
 
#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    virtual bool allows_autoland_direction_capture() const { return false; }
#endif

#if AP_QUICKTUNE_ENABLED
    // does this mode support VTOL quicktune?
    virtual bool supports_quicktune() const { return false; }
#endif

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support quadplane vtol systemid?
    virtual bool supports_vtol_systemid() const { return false; }

    // does this mode support plane or quadplane fixed wing systemid?
    virtual bool supports_fw_systemid() const { return false; }

    // Return true if fixed wing system ID should be allowed
    bool allow_fw_systemid() const;
#endif

protected:

    // subclasses override this to perform checks before entering the mode
    virtual bool _enter() { return true; }

    // subclasses override this to perform any required cleanup when exiting the mode
    virtual void _exit() { return; }

    // mode specific pre-arm checks
    virtual bool _pre_arm_checks(size_t buflen, char *buffer) const;

    // Helper to output to both k_rudder and k_steering servo functions
    void output_rudder_and_steering(float val);

    // Output pilot throttle, this is used in stabilized modes without auto throttle control
    void output_pilot_throttle();

    // makes the initialiser list in the constructor manageable
    uint8_t unused_integer;

#if HAL_QUADPLANE_ENABLED
    // References for convenience, used by QModes
    AC_PosControl*& pos_control;
    AC_AttitudeControl_Multi*& attitude_control;
    AC_Loiter*& loiter_nav;
    QuadPlane& quadplane;
    QuadPlane::PosControlState &poscontrol;
#endif
    AP_AHRS& ahrs;
};


class ModeAcro : public Mode
{
friend class ModeQAcro;
public:

    Mode::Number mode_number() const override { return Mode::Number::ACRO; }
    const char *name() const override { return "ACRO"; }
    const char *name4() const override { return "ACRO"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

    void stabilize();

    void stabilize_quaternion();

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif

protected:

    // ACRO controller state
    struct {
        bool locked_roll;
        bool locked_pitch;
        float locked_roll_err;
        int32_t locked_pitch_cd;
        Quaternion q;
        bool roll_active_last;
        bool pitch_active_last;
        bool yaw_active_last;
    } acro_state;

    bool _enter() override;
};

class ModeAuto : public Mode
{
public:
    friend class Plane;

    Number mode_number() const override { return Number::AUTO; }
    const char *name() const override { return "AUTO"; }
    const char *name4() const override { return "AUTO"; }

    bool does_automatic_thermal_switch() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override;

    bool does_auto_throttle() const override;
    
    bool mode_allows_autotuning() const override { return true; }

    bool is_landing() const override;

    void do_nav_delay(const AP_Mission::Mission_Command& cmd);
    bool verify_nav_delay(const AP_Mission::Mission_Command& cmd);

    bool verify_altitude_wait(const AP_Mission::Mission_Command& cmd);

    void run() override;

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif

#if AP_PLANE_GLIDER_PULLUP_ENABLED
    bool in_pullup() const { return pullup.in_pullup(); }
#endif

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support fixed wing systemid?
    bool supports_fw_systemid() const override { return true; }
#endif

protected:

    bool _enter() override;
    void _exit() override;
    bool _pre_arm_checks(size_t buflen, char *buffer) const override;

private:

    // Delay the next navigation command
    struct {
        uint32_t time_max_ms;
        uint32_t time_start_ms;
    } nav_delay;

    // wiggle state and timer for NAV_ALTITUDE_WAIT
    void wiggle_servos();
    struct {
        uint8_t stage;
        uint32_t last_ms;
    } wiggle;

#if AP_PLANE_GLIDER_PULLUP_ENABLED
    GliderPullup pullup;
#endif // AP_PLANE_GLIDER_PULLUP_ENABLED
};


class ModeAutoTune : public Mode
{
public:

    Number mode_number() const override { return Number::AUTOTUNE; }
    const char *name() const override { return "AUTOTUNE"; }
    const char *name4() const override { return "ATUN"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
    
    bool mode_allows_autotuning() const override { return true; }

    void run() override;

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif
    
protected:

    bool _enter() override;
};

class ModeGuided : public Mode
{
public:

    Number mode_number() const override { return Number::GUIDED; }
    const char *name() const override { return "GUIDED"; }
    const char *name4() const override { return "GUID"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    virtual bool is_guided_mode() const override { return true; }

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

    // handle a guided target request from GCS
    bool handle_guided_request(Location target_loc) override;

#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    // handle a guided airspeed command, typically from companion computer
    bool handle_change_airspeed(const float airspeed, const float acceleration);
#endif // AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED

    void set_radius_and_direction(const float radius, const bool direction_is_ccw);

    void update_target_altitude() override;

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support fixed wing systemid?
    bool supports_fw_systemid() const override { return true; }
#endif

protected:

    bool _enter() override;
    bool _pre_arm_checks(size_t buflen, char *buffer) const override { return true; }
#if AP_QUICKTUNE_ENABLED
    bool supports_quicktune() const override { return true; }
#endif

private:
    float active_radius_m;
};

class ModeCircle: public Mode
{
public:

    Number mode_number() const override { return Number::CIRCLE; }
    const char *name() const override { return "CIRCLE"; }
    const char *name4() const override { return "CIRC"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support fixed wing systemid?
    bool supports_fw_systemid() const override { return true; }
#endif

protected:

    bool _enter() override;
};

class ModeLoiter : public Mode
{
public:

    Number mode_number() const override { return Number::LOITER; }
    const char *name() const override { return "LOITER"; }
    const char *name4() const override { return "LOIT"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool isHeadingLinedUp(const Location loiterCenterLoc, const Location targetLoc);
    bool isHeadingLinedUp_cd(const int32_t bearing_cd, const int32_t heading_cd);
    bool isHeadingLinedUp_cd(const int32_t bearing_cd);

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

    bool allows_terrain_disable() const override { return true; }

    void update_target_altitude() override;
    
    bool mode_allows_autotuning() const override { return true; }

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support fixed wing systemid?
    bool supports_fw_systemid() const override { return true; }
#endif

protected:

    bool _enter() override;
};

#if HAL_QUADPLANE_ENABLED
class ModeLoiterAltQLand : public ModeLoiter
{
public:

    Number mode_number() const override { return Number::LOITER_ALT_QLAND; }
    const char *name() const override { return "Loiter to QLAND"; }
    const char *name4() const override { return "L2QL"; }

    // handle a guided target request from GCS
    bool handle_guided_request(Location target_loc) override;

protected:
    bool _enter() override;

    void navigate() override;

private:
    void switch_qland();

};
#endif // HAL_QUADPLANE_ENABLED

class ModeManual : public Mode
{
public:

    Number mode_number() const override { return Number::MANUAL; }
    const char *name() const override { return "MANUAL"; }
    const char *name4() const override { return "MANU"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

    // true if throttle min/max limits should be applied
    bool use_throttle_limits() const override;

    // true if voltage correction should be applied to throttle
    bool use_battery_compensation() const override { return false; }

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif

};


class ModeRTL : public Mode
{
public:

    Number mode_number() const override { return Number::RTL; }
    const char *name() const override { return "RTL"; }
    const char *name4() const override { return "RTL "; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;
    bool _pre_arm_checks(size_t buflen, char *buffer) const override { return false; }

private:

    // Switch to QRTL if enabled and within radius
    bool switch_QRTL();
};

class ModeStabilize : public Mode
{
public:

    Number mode_number() const override { return Number::STABILIZE; }
    const char *name() const override { return "STABILIZE"; }
    const char *name4() const override { return "STAB"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif

private:
    void stabilize_stick_mixing_direct();

};

class ModeTraining : public Mode
{
public:

    Number mode_number() const override { return Number::TRAINING; }
    const char *name() const override { return "TRAINING"; }
    const char *name4() const override { return "TRAN"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif
};

class ModeInitializing : public Mode
{
public:

    Number mode_number() const override { return Number::INITIALISING; }
    const char *name() const override { return "INITIALISING"; }
    const char *name4() const override { return "INIT"; }

    bool _enter() override { return false; }

    // methods that affect movement of the vehicle in this mode
    void update() override { }

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_throttle() const override { return true; }

protected:
    bool _pre_arm_checks(size_t buflen, char *buffer) const override { return false; }

};

class ModeFBWA : public Mode
{
public:

    Number mode_number() const override { return Number::FLY_BY_WIRE_A; }
    const char *name() const override { return "FLY_BY_WIRE_A"; }
    const char *name4() const override { return "FBWA"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;
    
    bool mode_allows_autotuning() const override { return true; }

    void run() override;

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support fixed wing systemid?
    bool supports_fw_systemid() const override { return true; }
#endif

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif

};

class ModeFBWB : public Mode
{
public:

    Number mode_number() const override { return Number::FLY_BY_WIRE_B; }
    const char *name() const override { return "FLY_BY_WIRE_B"; }
    const char *name4() const override { return "FBWB"; }

    bool allows_terrain_disable() const override { return true; }

    bool does_automatic_thermal_switch() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    bool does_auto_throttle() const override { return true; }
    
    bool mode_allows_autotuning() const override { return true; }

    void update_target_altitude() override {};

protected:

    bool _enter() override;
};

class ModeCruise : public Mode
{
public:

    Number mode_number() const override { return Number::CRUISE; }
    const char *name() const override { return "CRUISE"; }
    const char *name4() const override { return "CRUS"; }

    bool allows_terrain_disable() const override { return true; }

    bool does_automatic_thermal_switch() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool get_target_heading_cd(int32_t &target_heading) const;

    bool does_auto_throttle() const override { return true; }

    void update_target_altitude() override {};

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support fixed wing systemid?
    bool supports_fw_systemid() const override { return true; }
#endif

protected:

    bool _enter() override;

    bool locked_heading;
    int32_t locked_heading_cd;
    uint32_t lock_timer_ms;
};

#if HAL_ADSB_ENABLED
class ModeAvoidADSB : public Mode
{
public:

    Number mode_number() const override { return Number::AVOID_ADSB; }
    const char *name() const override { return "AVOID_ADSB"; }
    const char *name4() const override { return "AVOI"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    virtual bool is_guided_mode() const override { return true; }

    bool does_auto_throttle() const override { return true; }

protected:

    bool _enter() override;
};
#endif

#if HAL_QUADPLANE_ENABLED
class ModeQStabilize : public Mode
{
public:

    Number mode_number() const override { return Number::QSTABILIZE; }
    const char *name() const override { return "QSTABILIZE"; }
    const char *name4() const override { return "QSTB"; }

    bool is_vtol_mode() const override { return true; }
    bool is_vtol_man_throttle() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }
    bool allows_throttle_nudging() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // used as a base class for all Q modes
    bool _enter() override;

    void run() override;

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support quadplane vtol systemid?
    bool supports_vtol_systemid() const override { return true; }
#endif
    
protected:
private:

    void set_tailsitter_roll_pitch(const float roll_input, const float pitch_input);
    void set_limited_roll_pitch(const float roll_input, const float pitch_input);

};

class ModeQHover : public Mode
{
public:

    Number mode_number() const override { return Number::QHOVER; }
    const char *name() const override { return "QHOVER"; }
    const char *name4() const override { return "QHOV"; }

    bool is_vtol_mode() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support quadplane vtol systemid?
    bool supports_vtol_systemid() const override { return true; }
#endif
    
protected:

    bool _enter() override;
#if AP_QUICKTUNE_ENABLED
    bool supports_quicktune() const override { return true; }
#endif
};

class ModeQLoiter : public Mode
{
friend class QuadPlane;
friend class ModeQLand;
friend class Plane;

public:

    Number mode_number() const override { return Number::QLOITER; }
    const char *name() const override { return "QLOITER"; }
    const char *name4() const override { return "QLOT"; }

    bool is_vtol_mode() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

#if AP_PLANE_SYSTEMID_ENABLED
    // does this mode support quadplane vtol systemid?
    bool supports_vtol_systemid() const override { return true; }
#endif
    
protected:

    bool _enter() override;
    uint32_t last_target_loc_set_ms;

#if AP_QUICKTUNE_ENABLED
    bool supports_quicktune() const override { return true; }
#endif
};

class ModeQLand : public Mode
{
public:
    Number mode_number() const override { return Number::QLAND; }
    const char *name() const override { return "QLAND"; }
    const char *name4() const override { return "QLND"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

protected:

    bool _enter() override;
    bool _pre_arm_checks(size_t buflen, char *buffer) const override { return false; }
};

class ModeQRTL : public Mode
{
public:

    Number mode_number() const override { return Number::QRTL; }
    const char *name() const override { return "QRTL"; }
    const char *name4() const override { return "QRTL"; }

    bool is_vtol_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

    bool does_auto_throttle() const override { return true; }

    void update_target_altitude() override;

    bool allows_throttle_nudging() const override;

    float get_VTOL_return_radius() const;

protected:

    bool _enter() override;
    bool _pre_arm_checks(size_t buflen, char *buffer) const override { return false; }

private:

    enum class SubMode {
        climb,
        RTL,
    } submode;
};

class ModeQAcro : public Mode
{
public:

    Number mode_number() const override { return Number::QACRO; }
    const char *name() const override { return "QACRO"; }
    const char *name4() const override { return "QACO"; }

    bool is_vtol_mode() const override { return true; }
    bool is_vtol_man_throttle() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void run() override;

protected:

    bool _enter() override;
};

#if QAUTOTUNE_ENABLED
class ModeQAutotune : public Mode
{
public:

    Number mode_number() const override { return Number::QAUTOTUNE; }
    const char *name() const override { return "QAUTOTUNE"; }
    const char *name4() const override { return "QATN"; }

    bool is_vtol_mode() const override { return true; }
    virtual bool is_vtol_man_mode() const override { return true; }

    void run() override;

    // methods that affect movement of the vehicle in this mode
    void update() override;

protected:

    bool _enter() override;
    void _exit() override;
};
#endif  // QAUTOTUNE_ENABLED

#endif  // HAL_QUADPLANE_ENABLED

class ModeTakeoff: public Mode
{
public:
    ModeTakeoff();

    Number mode_number() const override { return Number::TAKEOFF; }
    const char *name() const override { return "TAKEOFF"; }
    const char *name4() const override { return "TKOF"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }

#if MODE_AUTOLAND_ENABLED   
    // true if mode allows landing direction to be set on first takeoff after arm in this mode 
    bool allows_autoland_direction_capture() const override { return true; }
#endif

    // var_info for holding parameter information
    static const struct AP_Param::GroupInfo var_info[];

    AP_Int16 target_alt;
    AP_Int16 level_alt;
    AP_Float ground_pitch;

protected:
    AP_Int16 target_dist;
    AP_Int8 level_pitch;

    bool takeoff_mode_setup;
    Location start_loc;

    bool _enter() override;

private:

    // flag that we have already called autoenable fences once in MODE TAKEOFF
    bool have_autoenabled_fences;

};
#if MODE_AUTOLAND_ENABLED
class ModeAutoLand: public Mode
{
public:
    ModeAutoLand();

    Number mode_number() const override { return Number::AUTOLAND; }
    const char *name() const override { return "AUTOLAND"; }
    const char *name4() const override { return "ALND"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    bool does_auto_throttle() const override { return true; }
    
    bool is_landing() const override;
    
    void check_takeoff_direction(void);

    // return true when lined up correctly from the LOITER_TO_ALT
    bool landing_lined_up(void);

    // see if we should capture the direction
    void arm_check(void);

    // var_info for holding parameter information
    static const struct AP_Param::GroupInfo var_info[];

    AP_Int16 final_wp_alt;
    AP_Int16 final_wp_dist;
    AP_Int16 landing_dir_off;
    AP_Int8  options;
    AP_Int16 terrain_alt_min;

    // Bitfields of AUTOLAND_OPTIONS
    enum class AutoLandOption {
        AUTOLAND_DIR_ON_ARM     = (1U << 0), // set dir for autoland on arm if compass in use.
    };

    enum class AutoLandStage {
        CLIMB,
        LOITER,
        LANDING
    };

    bool autoland_option_is_set(AutoLandOption option) const {
        return (options & int8_t(option)) != 0;
    }

protected:
    bool _enter() override;
    AP_Mission::Mission_Command cmd_climb;
    AP_Mission::Mission_Command cmd_loiter;
    AP_Mission::Mission_Command cmd_land;
    Location land_start;
    AutoLandStage stage;
    void set_autoland_direction(const float heading);
};
#endif
#if HAL_SOARING_ENABLED

class ModeThermal: public Mode
{
public:

    Number mode_number() const override { return Number::THERMAL; }
    const char *name() const override { return "THERMAL"; }
    const char *name4() const override { return "THML"; }

    // methods that affect movement of the vehicle in this mode
    void update() override;

    // Update thermal tracking and exiting logic.
    void update_soaring();

    void navigate() override;

    bool allows_throttle_nudging() const override { return true; }

    bool does_auto_navigation() const override { return true; }

    // true if we are in an auto-throttle mode, which means
    // we need to run the speed/height controller
    bool does_auto_throttle() const override { return true; }

protected:

    bool exit_heading_aligned() const;
    void restore_mode(const char *reason, ModeReason modereason);

    bool _enter() override;
};

#endif
