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
public:
	enum {
		ch_mode = 0, ch_str, ch_thrust
	};
	enum {
		k_chMode = k_radioChannelsStart, k_chStr, k_chThrust
	};
	enum {
		k_pidStr = k_controllersStart, k_pidThrust
	};

	ControllerCar(AP_Navigator * nav, AP_Guide * guide,
			AP_HardwareAbstractionLayer * hal) :
				AP_Controller(nav, guide, hal,new AP_ArmingMechanism(hal,ch_thrust,ch_str,0.1,-0.9,0.9), ch_mode),
				pidStr(new AP_Var_group(k_pidStr, PSTR("STR_")), 1, steeringP,
						steeringI, steeringD, steeringIMax, steeringYMax,steeringDFCut),
				pidThrust(new AP_Var_group(k_pidThrust, PSTR("THR_")), 1, throttleP,
						throttleI, throttleD, throttleIMax, throttleYMax,
						throttleDFCut), _strCmd(0), _thrustCmd(0)
   	{
		_hal->debug->println_P(PSTR("initializing car controller"));

		_hal->rc.push_back(
				new AP_RcChannel(k_chMode, PSTR("MODE_"), APM_RC, 5, 1100,
						1500, 1900, RC_MODE_IN, false));
		_hal->rc.push_back(
				new AP_RcChannel(k_chStr, PSTR("STR_"), APM_RC, 3, 1100, 1500,
						1900, RC_MODE_INOUT, false));
		_hal->rc.push_back(
				new AP_RcChannel(k_chThrust, PSTR("THR_"), APM_RC, 2, 1100, 1500,
						1900, RC_MODE_INOUT, false));
	}

private:
	BlockPIDDfb pidStr;
	BlockPID pidThrust;

	float _strCmd, _thrustCmd;

	void manualLoop(const float dt) {
		setAllRadioChannelsManually();
		_strCmd = _hal->rc[ch_str]->getRadioPosition();
		_thrustCmd = _hal->rc[ch_thrust]->getRadioPosition();
	}

	void autoLoop(const float dt) {
		_hal->debug->printf_P(PSTR("hdg cmd: %f, yaw: %f\n"),_guide->getHeadingCommand(),_nav->getYaw());
		float headingError = _guide->getHeadingCommand()
			- _nav->getYaw();
		if (headingError > 180 * deg2Rad)
			headingError -= 360 * deg2Rad;
		if (headingError < -180 * deg2Rad)
			headingError += 360 * deg2Rad;
		_strCmd = pidStr.update(headingError, _nav->getYawRate(), dt);
		_thrustCmd = pidThrust.update(
					_guide->getGroundSpeedCommand()
					- _nav->getGroundSpeed(), dt);
	}

	void setMotors() {

		switch (_hal->getState()) {

			case MAV_STATE_ACTIVE: {
				digitalWrite(_hal->aLedPin, HIGH);
				// turn all motors off if below 0.1 throttle
				if (fabs(_hal->rc[ch_thrust]->getRadioPosition()) < 0.1) {
					setAllRadioChannelsToNeutral();
				} else {
					_hal->rc[ch_thrust]->setPosition(_thrustCmd);
					_hal->rc[ch_str]->setPosition(_strCmd);
				}
				break;
			}
			case MAV_STATE_EMERGENCY: {
				digitalWrite(_hal->aLedPin, LOW);
				setAllRadioChannelsToNeutral();
				break;
			}
			case MAV_STATE_STANDBY: {
				digitalWrite(_hal->aLedPin,LOW);
				setAllRadioChannelsToNeutral();
				break;
			}
			default: {
				digitalWrite(_hal->aLedPin, LOW);
				setAllRadioChannelsToNeutral();
			}

		}
	}


};

} // namespace apo

#endif /* CONTROLLERCAR_H_ */
