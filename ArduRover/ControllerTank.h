/*
 * ControllerTank.h
 *
 *  Created on: Jun 30, 2011
 *      Author: jgoppert
 */

#ifndef CONTROLLERTANK_H_
#define CONTROLLERTANK_H_

#include "../APO/AP_Controller.h"

namespace apo {

class ControllerTank: public AP_Controller {
private:
	AP_Var_group _group;
	AP_Uint8 _mode;
	enum {
		k_chMode = k_radioChannelsStart, k_chLeft, k_chRight, k_chStr, k_chThr
	};
	enum {
		k_pidStr = k_controllersStart, k_pidThr
	};
	enum {
		CH_MODE = 0, CH_LEFT, CH_RIGHT, CH_STR, CH_THR
	};
	BlockPIDDfb pidStr;
	BlockPID pidThr;
public:
	ControllerTank(AP_Navigator * nav, AP_Guide * guide,
			AP_HardwareAbstractionLayer * hal) :
				AP_Controller(nav, guide, hal),
				_group(k_cntrl, PSTR("CNTRL_")),
				_mode(&_group, 1, MAV_MODE_UNINIT, PSTR("MODE")),
				pidStr(new AP_Var_group(k_pidStr, PSTR("STR_")), 1, steeringP,
						steeringI, steeringD, steeringIMax, steeringYMax),
				pidThr(new AP_Var_group(k_pidThr, PSTR("THR_")), 1, throttleP,
						throttleI, throttleD, throttleIMax, throttleYMax,
						throttleDFCut) {
		_hal->debug->println_P(PSTR("initializing tank controller"));

		_hal->rc.push_back(
				new AP_RcChannel(k_chMode, PSTR("MODE_"), APM_RC, 5, 1100,
						1500, 1900, RC_MODE_IN));
		_hal->rc.push_back(
				new AP_RcChannel(k_chLeft, PSTR("LEFT_"), APM_RC, 0, 1100, 1500,
						1900, RC_MODE_OUT));
		_hal->rc.push_back(
				new AP_RcChannel(k_chRight, PSTR("RIGHT_"), APM_RC, 1, 1100, 1500,
						1900, RC_MODE_OUT));
		_hal->rc.push_back(
				new AP_RcChannel(k_chStr, PSTR("STR_"), APM_RC, 0, 1100, 1500,
						1900, RC_MODE_IN));
		_hal->rc.push_back(
				new AP_RcChannel(k_chThr, PSTR("THR_"), APM_RC, 1, 1100, 1500,
						1900, RC_MODE_IN));
	}
	virtual MAV_MODE getMode() {
		return (MAV_MODE) _mode.get();
	}
	void mix(float headingOutput, float throttleOutput) {
		_hal->rc[CH_LEFT]->setPosition(throttleOutput + headingOutput);
		_hal->rc[CH_RIGHT]->setPosition(throttleOutput - headingOutput);
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
			mix(_hal->rc[CH_STR]->getPosition(),
					_hal->rc[CH_THR]->getPosition());
			break;
		}
		case MAV_MODE_AUTO: {
			float headingError = _guide->getHeadingCommand()
					- _nav->getYaw();
			if (headingError > 180 * deg2Rad)
				headingError -= 360 * deg2Rad;
			if (headingError < -180 * deg2Rad)
				headingError += 360 * deg2Rad;
			mix(pidStr.update(headingError, _nav->getYawRate(), dt),
				pidThr.update(_guide->getGroundSpeedCommand()
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

#endif /* CONTROLLERTANK_H_ */
