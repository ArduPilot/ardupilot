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

#ifndef __AP_YAW_CONTROLLER_H__
#define __AP_YAW_CONTROLLER_H__

#include <AP_AHRS.h>
#include <AP_Common.h>
#include <AP_Vehicle.h>
#include <math.h>

class AP_YawController {
public:                      
	//BEV modified from FixedWing to VTOL
	AP_YawController(AP_AHRS &ahrs, const AP_Vehicle::VTOL &parms) :
		aparm(parms),
        _ahrs(ahrs)
	{
		AP_Param::setup_object_defaults(this, var_info);
	}

	int32_t get_servo_out(float scaler, bool disable_integrator);

	void reset_I();

	static const struct AP_Param::GroupInfo var_info[];

private:
	//BEV modified from FixedWing to VTOL
	const AP_Vehicle::VTOL &aparm;
	AP_Float _K_A;
	AP_Float _K_I;
	AP_Float _K_D;
	AP_Float _K_FF;
    AP_Int16 _imax;
	uint32_t _last_t;
	float _last_error;
	float _last_out;
	float _last_rate_hp_out;
	float _last_rate_hp_in;
	float _K_D_last;

	float _integrator;

	AP_AHRS &_ahrs;
};

#endif // __AP_YAW_CONTROLLER_H__
