/*
 * ControllerCar.h
 *
 *  Created on: Jun 30, 2011
 *      Author: jgoppert
 */

#ifndef CONTROLLERCAR_H_
#define CONTROLLERCAR_H_

#include "../APO/AP_Controller.h"

namespace apo {

class ControllerCar: public AP_Controller {
private:
	AP_Var_group _group;
	AP_Uint8 _mode;
	enum {
		k_chMode = k_radioChannelsStart, k_chStr, k_chThr
	};
	enum {
		k_pidStr = k_controllersStart, k_pidThr
	};
	enum {
		CH_MODE = 0, CH_STR, CH_THR
	};
	BlockPIDDfb pidStr;
	BlockPID pidThr;
public:
	ControllerCar(AP_Navigator * nav, AP_Guide * guide,
			AP_HardwareAbstractionLayer * hal) :
				AP_Controller(nav, guide, hal),
				_group(k_cntrl, PSTR("CNTRL_")),
				_mode(&_group, 1, MAV_MODE_UNINIT, PSTR("MODE")),
				pidStr(new AP_Var_group(k_pidStr, PSTR("STR_")), 1, steeringP,
						steeringI, steeringD, steeringIMax, steeringYMax),
				pidThr(new AP_Var_group(k_pidThr, PSTR("THR_")), 1, throttleP,
						throttleI, throttleD, throttleIMax, throttleYMax,
						throttleDFCut) {
		_hal->debug->println_P(PSTR("initializing car controller"));

		_hal->rc.push_back(
				new AP_RcChannel(k_chMode, PSTR("MODE_"), APM_RC, 7, 1100,
						1500, 1900));
		_hal->rc.push_back(
				new AP_RcChannel(k_chStr, PSTR("STR_"), APM_RC, 0, 1100, 1540,
						1900));
		_hal->rc.push_back(
				new AP_RcChannel(k_chThr, PSTR("THR_"), APM_RC, 1, 1100, 1500,
						1900));
	}
	virtual MAV_MODE getMode() {
		return (MAV_MODE) _mode.get();
	}
	virtual void update(const float & dt) {

		// check for heartbeat
		if (_hal->heartBeatLost()) {
			_mode = MAV_MODE_FAILSAFE;
			setAllRadioChannelsToNeutral();
			_hal->setState(MAV_STATE_EMERGENCY);
			_hal->debug->printf_P(PSTR("comm lost, send heartbeat from gcs\n"));
			return;
		// if throttle less than 5% cut motor power
		} else if (_hal->rc[CH_THR]->getRadioPosition() < 0.05) {
			_mode = MAV_MODE_LOCKED;
			setAllRadioChannelsToNeutral();
			_hal->setState(MAV_STATE_STANDBY);
			return;
		// if in live mode then set state to active
		} else if (_hal->getMode() == MODE_LIVE) {
			_hal->setState(MAV_STATE_ACTIVE);
		// if in hardware in the loop (control) mode, set to hilsim
		} else if (_hal->getMode() == MODE_HIL_CNTL) {
			_hal->setState(MAV_STATE_HILSIM);
		}
		
		// read switch to set mode
		if (_hal->rc[CH_MODE]->getRadioPosition() > 0) {
			_mode = MAV_MODE_MANUAL;
		} else {
			_mode = MAV_MODE_AUTO;
		}

		// manual mode
		switch (_mode) {

		case MAV_MODE_MANUAL: {
			setAllRadioChannelsManually();
			//_hal->debug->println("manual");
			break;
		}
		case MAV_MODE_AUTO: {
			float headingError = _guide->getHeadingCommand()
					- _nav->getYaw();
			if (headingError > 180 * deg2Rad)
				headingError -= 360 * deg2Rad;
			if (headingError < -180 * deg2Rad)
				headingError += 360 * deg2Rad;
			_hal->rc[CH_STR]->setPosition(
					pidStr.update(headingError, _nav->getYawRate(), dt));
			_hal->rc[CH_THR]->setPosition(
					pidThr.update(
							_guide->getGroundSpeedCommand()
									- _nav->getGroundSpeed(), dt));
			//_hal->debug->println("automode");
			break;
		}

		default: {
			setAllRadioChannelsToNeutral();
			_mode = MAV_MODE_FAILSAFE;
			_hal->setState(MAV_STATE_EMERGENCY);
			_hal->debug->printf_P(PSTR("unknown controller mode\n"));
			break;
		}

		}
	}
};

} // namespace apo

#endif /* CONTROLLERCAR_H_ */
