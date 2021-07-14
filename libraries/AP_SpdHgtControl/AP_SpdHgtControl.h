#pragma once

/// @file    AP_SpdHgtControl.h
/// @brief   generic speed & height controller interface

/*
  This defines a generic interface for speed & height controllers. Each
  specific controller should be a subclass of this generic
  interface. All variables used by controllers should be in their
  own class.
 */

#include <AP_Vehicle/AP_Vehicle.h>

#include <stdint.h>

class AP_SpdHgtControl {
public:
	// Update the internal state of the height and height rate estimator
	// Update of the inertial speed rate estimate internal state
	// Should be called at 50Hz or faster
	virtual void update_50hz(void) = 0;


	// Update of the pitch and throttle demands
	// Should be called at 10Hz or faster
	virtual void update_pitch_throttle( int32_t hgt_dem_cm,
										int32_t EAS_dem_cm,
										enum AP_Vehicle::FixedWing::FlightStage flight_stage,
                                        float distance_beyond_land_wp,
										int32_t ptchMinCO_cd,
										int16_t throttle_nudge,
                                        float hgt_afe,
										float load_factor) = 0;

	// demanded throttle in percentage
	// should return 0 to 100
	virtual int32_t get_throttle_demand(void)=0;
	
	// demanded pitch angle in centi-degrees
	// should return -9000 to +9000
	virtual int32_t get_pitch_demand(void)=0;
	
	// Rate of change of velocity along X body axis in m/s^2
    virtual float get_VXdot(void)=0;
	
	// return current target airspeed
	virtual float get_target_airspeed(void) const = 0;

	// return maximum climb rate
	virtual float get_max_climbrate(void) const = 0;

    // return maximum sink rate (+ve number)
    virtual float get_max_sinkrate(void) const = 0;

    // added to let SoaringController reset pitch integrator to zero
    virtual void reset_pitch_I(void) = 0;
    
    // return landing sink rate
    virtual float get_land_sinkrate(void) const = 0;

    // return landing airspeed
    virtual float get_land_airspeed(void) const = 0;

	// set path_proportion accessor
    virtual void set_path_proportion(float path_proportion) = 0;

    // reset on next loop
    virtual void reset(void) = 0;

    // set gliding requested flag
    virtual void set_gliding_requested_flag(bool gliding_requested) = 0;

    // set propulsion failed flag
    virtual void set_propulsion_failed_flag(bool propulsion_failed) = 0;

	// add new controllers to this enum. Users can then
	// select which controller to use by setting the
	// SPDHGT_CONTROLLER parameter
	enum ControllerType {
		CONTROLLER_TECS     = 1
	};
	

};
