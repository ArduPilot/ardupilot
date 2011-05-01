/*
 * AP_Controller.h
 * Copyright (C) James Goppert 2010 <james.goppert@gmail.com>
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef AP_Controller_H
#define AP_Controller_H

#include "AP_Navigator.h"
#include "AP_Guide.h"
#include "AP_HardwareAbstractionLayer.h"
#include "../AP_Common/AP_Vector.h"
#include "../AP_Common/AP_Var.h"

namespace apo {

/// Controller class
class AP_Controller {
public:
	AP_Controller(AP_Navigator * nav, AP_Guide * guide,
			AP_HardwareAbstractionLayer * hal) :
		_nav(nav), _guide(guide), _hal(hal) {
	}

	bool commLost() {
		if (_hal->getTimeSinceLastHeartBeat() > heartbeatTimeout) return true;
		else return false;
	}

	virtual void update(const float & dt) = 0;

	MAV_MODE getMode() { return _mode; }

protected:
	MAV_MODE _mode;
	AP_Navigator * _nav;
	AP_Guide * _guide;
	AP_HardwareAbstractionLayer * _hal;
};

class Pid2 {
public:
	Pid2(AP_Var::Key key, const prog_char_t * name, float kP = 0.0,
			float kI = 0.0, float kD = 0.0, float iMax = 0.0, float yMax = 0.0,
			uint8_t dFcut = 20.0) :
		_group(key, name), _eP(0), _eI(0), _eD(0),
				_kP(&_group, 1, kP, PSTR("P")),
				_kI(&_group, 2, kI, PSTR("I")),
				_kD(&_group, 3, kD, PSTR("D")),
				_iMax(&_group, 4, iMax, PSTR("IMAX")),
				_yMax(&_group, 5, yMax, PSTR("YMAX")),
				_fCut(&_group, 6, dFcut, PSTR("FCUT")) {
	}

	float update(const float & input, const float & dt) {

		// derivative with low pass
		float RC = 1 / (2 * M_PI * _fCut); // low pass filter
		_eD = _eD + ((_eP - input) / dt - _eD) * (dt / (dt + RC));

		// proportional, note must come after derivative
		// because derivatve uses _eP as previous value
		_eP = input;

		// integral
		_eI += _eP * dt;

		// wind up guard
		if (_eI > _iMax)
			_eI = _iMax;
		if (_eI < -_iMax)
			_eI = -_iMax;

		// pid sum
		float y = _kP * _eP + _kI * _eI + _kD * _eD;

		// saturation
		if (y > _yMax)
			y = _yMax;
		if (y < -_yMax)
			y = -_yMax;

		return y;
	}
protected:
	AP_Var_group _group; /// helps with parameter management
	float _eP; /// input
	float _eI; /// integral of input
	float _eD; /// derivative of input
	AP_Float _kP; /// proportional gain
	AP_Float _kI; /// integral gain
	AP_Float _kD; /// derivative gain
	AP_Float _iMax; /// integrator saturation
	AP_Float _yMax; /// output saturation
	AP_Uint8 _fCut; /// derivative low-pass cut freq (Hz)
};


/// PID(DFB) block
class PidDFB2 {
public:
	PidDFB2(AP_Var::Key key, const prog_char_t * name,
			float kP = 0.0, float kI = 0.0, float kD = 0.0, float iMax = 0.0,
			float yMax = 0.0) :
		_group(key, name), _eP(0), _eI(0), _eD(0),
				_kP(&_group, 1, kP, PSTR("P")),
				_kI(&_group, 2, kI, PSTR("I")),
				_kD(&_group, 3, kD, PSTR("D")),
				_iMax(&_group, 4, iMax, PSTR("IMAX")),
				_yMax(&_group, 5, yMax, PSTR("YMAX")) {
	}

	float update(const float & input, const float & derivative, const float & dt) {

		// proportional, note must come after derivative
		// because derivative uses _eP as previous value
		_eP = input;

		// integral
		_eI += _eP * dt;

		// wind up guard
		if (_eI > _iMax)
			_eI = _iMax;
		if (_eI < -_iMax)
			_eI = -_iMax;

		// pid sum
		float y = _kP * _eP + _kI * _eI - _kD * derivative;

		// saturation
		if (y > _yMax)
			y = _yMax;
		if (y < -_yMax)
			y = -_yMax;

		return y;
	}
protected:
	AP_Var_group _group; /// helps with parameter management
	float _eP; /// input
	float _eI; /// integral of input
	float _eD; /// derivative of input
	AP_Float _kP; /// proportional gain
	AP_Float _kI; /// integral gain
	AP_Float _kD; /// derivative gain
	AP_Float _iMax; /// integrator saturation
	AP_Float _yMax; /// integrator saturation
};

} // apo

#endif // AP_Controller_H
// vim:ts=4:sw=4:expandtab
