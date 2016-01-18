// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *  Copyright (c) BirdsEyeView Aerobotics, LLC, 2016.
 *
 *  This program is free software: you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 3 as published
 *  by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License version 3 for more details.
 *
 *  You should have received a copy of the GNU General Public License version
 *  3 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __AP_PITCH_CONTROLLER_H__
#define __AP_PITCH_CONTROLLER_H__

#include <AP_AHRS.h>
#include <AP_Common.h>
#include <AP_Vehicle.h>
#include <AP_AutoTune.h>
#include <DataFlash.h>
#include <AP_Math.h>

class AP_PitchController {
public:
	//BEV modified from FixedWing to VTOL
	AP_PitchController(AP_AHRS &ahrs, const AP_Vehicle::VTOL &parms, DataFlash_Class &_dataflash) :
		aparm(parms),
        autotune(gains, AP_AutoTune::AUTOTUNE_PITCH, parms, _dataflash),
        _ahrs(ahrs)
    { 
		AP_Param::setup_object_defaults(this, var_info);
	}

	int32_t get_rate_out(float desired_rate, float scaler);
	int32_t get_servo_out(int32_t angle_err, float scaler, bool disable_integrator);

	void reset_I();

    void autotune_start(void) { autotune.start(); }
    void autotune_restore(void) { autotune.stop(); }

	static const struct AP_Param::GroupInfo var_info[];

private:
	//BEV modified from FixedWing to VTOL
	const AP_Vehicle::VTOL &aparm;
    AP_AutoTune::ATGains gains;
    AP_AutoTune autotune;
	AP_Int16 _max_rate_neg;
	AP_Float _roll_ff;
	uint32_t _last_t;
	float _last_out;
	
	float _integrator;

	int32_t _get_rate_out(float desired_rate, float scaler, bool disable_integrator, float aspeed);
    float   _get_coordination_rate_offset(float &aspeed, bool &inverted) const;
	
	AP_AHRS &_ahrs;
	
};

#endif // __AP_PITCH_CONTROLLER_H__
