/// @file   AP_LandingGear.h
/// @brief  Landing gear control library
#pragma once

#include "AP_LandingGear_config.h"

#if AP_LANDINGGEAR_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_Common/AP_Common.h>
#include <AP_Logger/AP_Logger_config.h>

/// @class  AP_LandingGear
/// @brief  Class managing the control of landing gear
class AP_LandingGear {
public:
    AP_LandingGear() {
        // setup parameter defaults
        AP_Param::setup_object_defaults(this, var_info);
        
        if (_singleton != nullptr) {
            AP_HAL::panic("AP_LandingGear must be singleton");
        }
        _singleton = this;
    }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_LandingGear);
    
    // get singleton instance
    static AP_LandingGear *get_singleton(void) {
        return _singleton;
    }

    // Gear command modes
    enum LandingGearCommand {
        LandingGear_Retract,
        LandingGear_Deploy,
    };

    // Gear command modes
    enum LandingGearStartupBehaviour {
        LandingGear_Startup_WaitForPilotInput = 0,
        LandingGear_Startup_Retract = 1,
        LandingGear_Startup_Deploy = 2,
    };

    /// initialise state of landing gear
    void init();

    /// returns true if the landing gear is deployed
    bool deployed();
    
    enum LG_LandingGear_State {
        LG_UNKNOWN = -1,
        LG_RETRACTED = 0,
        LG_DEPLOYED = 1,
        LG_RETRACTING = 2,
        LG_DEPLOYING = 3,
        };
    /// returns detailed state of gear
    LG_LandingGear_State get_state();
    
    enum LG_WOW_State {
        LG_WOW_UNKNOWN = -1,
        LG_NO_WOW = 0,
        LG_WOW = 1,
        };

    LG_WOW_State get_wow_state();

    /// set landing gear position to retract, deploy or deploy-and-keep-deployed
    void set_position(LandingGearCommand cmd);
    
    uint32_t get_gear_state_duration_ms() const;
    uint32_t get_wow_state_duration_ms() const;

    static const struct AP_Param::GroupInfo        var_info[];
    
    void update(float height_above_ground_m);
    
    bool check_before_land(void);

    // retract after takeoff or deploy for landing depending on the OPTIONS parameter
    void retract_after_takeoff();
    void deploy_for_landing();

private:
    // Parameters
    AP_Int8     _enable;
    AP_Int8     _startup_behaviour;     // start-up behaviour (see LandingGearStartupBehaviour)
    
    AP_Int8     _pin_deployed;
    AP_Int8     _pin_deployed_polarity;
    AP_Int8     _pin_weight_on_wheels;
    AP_Int8     _pin_weight_on_wheels_polarity;
    AP_Int16    _deploy_alt;
    AP_Int16    _retract_alt;
    AP_Int16    _options;

    // bitmask of options
    enum class Option : uint16_t {
        RETRACT_AFTER_TAKEOFF = (1U<<0),
        DEPLOY_DURING_LANDING = (1U<<1)
    };

    // internal variables
    bool        _deployed;              // true if the landing gear has been deployed, initialized false
    bool        _have_changed;          // have we changed the servo state?

    int16_t     _last_height_above_ground;
    
    // debounce
    LG_WOW_State wow_state_current = LG_WOW_UNKNOWN;
    uint32_t last_wow_event_ms;
    
    LG_LandingGear_State gear_state_current = LG_UNKNOWN;
    uint32_t last_gear_event_ms;

    /// retract - retract landing gear
    void retract();
    
    /// deploy - deploy the landing gear
    void deploy();

#if HAL_LOGGING_ENABLED
    // log weight on wheels state
    void log_wow_state(LG_WOW_State state);
#else
    void log_wow_state(LG_WOW_State state) {}
#endif

    static AP_LandingGear *_singleton;
};

#endif  // AP_LANDINGGEAR_ENABLED
