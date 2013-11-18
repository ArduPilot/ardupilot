// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file    AP_SpdHgtControl.h
/// @brief   generic speed & height controller interface

/*
  This defines a generic interface for speed & height controllers. Each
  specific controller should be a subclass of this generic
  interface. All variables used by controllers should be in their
  own class.
 */

#ifndef AP_SPDHGTCONTROL_H
#define AP_SPDHGTCONTROL_H

#include <AP_Common.h>
#include <AP_Param.h>
#include <DataFlash.h>

class AP_SpdHgtControl {
public:
	// Update the internal state of the height and height rate estimator
	// Update of the inertial speed rate estimate internal state
	// Should be called at 50Hz or faster
	virtual void update_50hz(float height_above_field) = 0;

	/**
	   stages of flight so the altitude controller can choose to
	   prioritise height or speed
	 */
	enum FlightStage {
		FLIGHT_NORMAL        = 1,
		FLIGHT_TAKEOFF       = 2,
		FLIGHT_LAND_APPROACH = 3,
		FLIGHT_LAND_FINAL    = 4
	};

	// Update of the pitch and throttle demands
	// Should be called at 10Hz or faster
	virtual void update_pitch_throttle( int32_t hgt_dem_cm,
										int32_t EAS_dem_cm,
										enum FlightStage flight_stage,
										int32_t ptchMinCO_cd,
										int16_t throttle_nudge,
                                        float hgt_afe) = 0;

	// demanded throttle in percentage
	// should return 0 to 100
	virtual int32_t get_throttle_demand(void)=0;
	
	// demanded pitch angle in centi-degrees
	// should return -9000 to +9000
	virtual int32_t get_pitch_demand(void)=0;
	
	// Rate of change of velocity along X body axis in m/s^2
    virtual float get_VXdot(void)=0;
	
	// log data on internal state of the controller. Called at 10Hz
	virtual void log_data(DataFlash_Class &dataflash, uint8_t msgid) = 0;

	// add new controllers to this enum. Users can then
	// select which controller to use by setting the
	// SPDHGT_CONTROLLER parameter
	enum ControllerType {
		CONTROLLER_TECS     = 1
	};
	

};


#endif // AP_SPDHGTCONTROL_H
