// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "AP_Land.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Land::var_info[] = {

    // @Param: PITCH_CD
    // @DisplayName: Landing Pitch
    // @Description: Used in autoland to give the minimum pitch in the final stage of landing (after the flare). This parameter can be used to ensure that the final landing attitude is appropriate for the type of undercarriage on the aircraft. Note that it is a minimum pitch only - the landing code will control pitch above this value to try to achieve the configured landing sink rate.
    // @Units: centi-Degrees
    // @User: Advanced
    AP_GROUPINFO("PITCH_CD",  0, AP_Land, pitch_cd, 0),

    // @Param: FLARE_ALT
    // @DisplayName: Landing flare altitude
    // @Description: Altitude in autoland at which to lock heading and flare to the LAND_PITCH_CD pitch. Note that this option is secondary to LAND_FLARE_SEC. For a good landing it preferable that the flare is triggered by LAND_FLARE_SEC.
    // @Units: meters
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("FLARE_ALT", 1, AP_Land, flare_alt, 3.0),

    // @Param: FLARE_SEC
    // @DisplayName: Landing flare time
    // @Description: Vertical time before landing point at which to lock heading and flare with the motor stopped. This is vertical time, and is calculated based solely on the current height above the ground and the current descent rate.  Set to 0 if you only wish to flare based on altitude (see LAND_FLARE_ALT).
    // @Units: seconds
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("FLARE_SEC", 2, AP_Land, flare_sec, 2.0),

    // @Param: DISARMDELAY
    // @DisplayName: Landing disarm delay
    // @Description: After a landing has completed using a LAND waypoint, automatically disarm after this many seconds have passed. Use 0 to not disarm.
    // @Units: seconds
    // @Increment: 1
    // @Range: 0 127
    // @User: Advanced
    AP_GROUPINFO("DISARMDELAY", 3, AP_Land, disarm_delay, 20),

    // @Param: ABORT_THR
    // @DisplayName: Landing abort using throttle
    // @Description: Allow a landing abort to trigger with a throttle > 95%
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ABORT_THR", 4, AP_Land, abort_throttle_enable, 0),

    // @Param: FLAP_PERCNT
    // @DisplayName: Landing flap percentage
    // @Description: The amount of flaps (as a percentage) to apply in the landing approach and flare of an automatic landing
    // @Range: 0 100
    // @Units: Percent
    // @User: Advanced
    AP_GROUPINFO("FLAP_PERCNT", 5, AP_Land, flap_percent, 0),
    
    // @Param: SLOPE
    // @DisplayName: Land Glide Slope
    // @Description: User defined landing approach glide slope (vertical distance/horizontal distance)
    // @User: Advanced
    AP_GROUPINFO("SLOPE", 6, AP_Land, slope, 0),
    
    // @Param: FLARE_THR
    // @DisplayName: Flare Throttle Percentage
    // @Description: The percentage of reverse throttle to use during the flare stage. A positive value will give you forward throttle and a negative value will give you reverse throttle. Set to 0 if your plane's prop cannot spin while on the ground.
    // @Range: -100 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("FLARE_THR", 7, AP_Land, flare_thr, 0),
    
    // @Param: REV_PT_DN
    // @DisplayName: Reverse throttle point DOWN
    // @Description: It is at this amount of altitude error from the land slope that we use the least amount of reverse throttle (this happens when we are below our land slope)
    // @Range: 0 10000
    // @Increments: 1
    // @User: Advanced
    AP_GROUPINFO("REV_PT_DN", 8, AP_Land, rev_pt_dn, 0),
    
    // @Param: REV_PT_UP
    // @DisplayName: Reverse throttle point UP
    // @Description: It is at this amount of altitude error from the land slope that we use 100% of reverse throttle (this happens when we are above our land slope)
    // @Range: 0 10000
    // @Increments: 1
    // @User: Advanced
    AP_GROUPINFO("REV_PT_UP", 9, AP_Land, rev_pt_up, 0),
    
    // @Param: PFLARE_THR
    // @DisplayName: Pre-flare throttle percentage
    // @Description: The percentage of reverse throttle to use during the pre-flare stage. A positive value will give you forward throttle and a negative value will give you reverse throttle.
    // @Range: -100 100
    // @Increments: 1
    // @User: Advanced
    AP_GROUPINFO("PFLARE_THR", 10, AP_Land, pre_flare_thr, 0),
    
    // @Param: PFLARE_ALT
    // @DisplayName: Pre-flare altitude
    // @Description: At what altitude will we prepare for the flare.
    // @Range: 0 10000
    // @Increments: 1
    // @User: Advanced
    AP_GROUPINFO("PFLARE_ALT", 11, AP_Land, pre_flare_alt, 0),
    
    // @Param: A_TAR_SPD
    // @DisplayName: Approach Target Speed
    // @Description: A rough target groundspeed the autopilot will try to reach during approach
    // @Range: 
    // @Increment: .1
    // @User: Advanced
    AP_GROUPINFO("A_TAR_SPD", 12, AP_Land, app_tar_spd, 8),
    
    // @Param: A_SPD_CONS
    // @DisplayName: Approach Speed Constant
    // @Description: A constant used for calculating the amount of PWM offset based on target groundspeed error. (PWM_Offset = constant * speed_error)
    // @Range: 
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("A_SPD_CONS", 13, AP_Land, app_spd_const, 50),
    
    // @Param: REV_ON_GND
    // @DisplayName: Reverse on ground
    // @Description: Enables the motor to spin while on the ground. Useful for slowing planes down during touchdown, but disable if your prop cannot spin while on the ground. This value dictates how much additional throttle braking you want after you pass the Land point during flare. 1 means give an extra 1us PWM of throttle per 1 meter. A value of 10 works well here.
    // @Range: 0 100
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("REV_ON_GND", 14, AP_Land, rev_on_gnd, 0),
    
    // @Param: THR_MIN_REV
    // @DisplayName: Minimum Reverse Throttle PWM
    // @Description: PWM value that gives 100% reverse throttle
    // @Range: 0 10000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("THR_MIN_REV", 15, AP_Land, thr_min_rev_pwm, 0),
	
	// @Param: BAT_OFFSET
    // @DisplayName: Land battery offset
    // @Description: Determines the maximum amount of battery offset to add to reverse throttle during landings to account for full batteries
    // @Range: 0 10000
	// @Units: PWM
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("bat_offset", 16, AP_Land, bat_offset, 100),

    AP_GROUPEND
};

bool AP_Land::flightstage_is_land(AP_SpdHgtControl::FlightStage fs)
{
    return (fs == AP_SpdHgtControl::FLIGHT_LAND_FINAL ||
            fs == AP_SpdHgtControl::FLIGHT_LAND_APPROACH ||
            fs == AP_SpdHgtControl::FLIGHT_LAND_FINAL_STEEP ||
            fs == AP_SpdHgtControl::FLIGHT_LAND_APPROACH_STEEP);
}
