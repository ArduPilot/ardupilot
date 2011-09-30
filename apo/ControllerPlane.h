/*
 * ControllerPlane.h
 *
 *  Created on: Jun 30, 2011
 *      Author: jgoppert
 */

#ifndef CONTROLLERPLANE_H_
#define CONTROLLERPLANE_H_

#include "../APO/AP_Controller.h"

namespace apo {

class ControllerPlane: public AP_Controller {
private:
	AP_Var_group _group;
	AP_Var_group _trimGroup;
	AP_Uint8 _mode;
	AP_Uint8 _rdrAilMix;
	bool _needsTrim;
	AP_Float _ailTrim;
	AP_Float _elvTrim;
	AP_Float _rdrTrim;
	AP_Float _thrTrim;
	enum {
		ch_mode = 0, ch_roll, ch_pitch, ch_thr, ch_yaw
	};
	enum {
		k_chMode = k_radioChannelsStart,
		k_chRoll,
		k_chPitch,
		k_chYaw,
		k_chThr,

		k_pidBnkRll = k_controllersStart,
		k_pidSpdPit,
		k_pidPitPit,
		k_pidYwrYaw,
		k_pidHdgBnk,
		k_pidAltThr,

		k_trim = k_customStart
	};
	BlockPID pidBnkRll; // bank error to roll servo deflection
	BlockPID pidSpdPit; // speed error to pitch command
	BlockPID pidPitPit; // pitch error to pitch servo deflection
	BlockPID pidYwrYaw; // yaw rate error to yaw servo deflection
	BlockPID pidHdgBnk; // heading error to bank command
	BlockPID pidAltThr; // altitude error to throttle deflection
	bool requireRadio;

public:
	ControllerPlane(AP_Navigator * nav, AP_Guide * guide,
			AP_HardwareAbstractionLayer * hal) :
				AP_Controller(nav, guide, hal),
				_group(k_cntrl, PSTR("cntrl_")),
				_trimGroup(k_trim, PSTR("trim_")),
				_mode(&_group, 1, MAV_MODE_UNINIT, PSTR("mode")),
				_rdrAilMix(&_group, 2, rdrAilMix, PSTR("rdrAilMix")),
				_needsTrim(false),
				_ailTrim(&_trimGroup, 1, ailTrim, PSTR("ail")),
				_elvTrim(&_trimGroup, 2, elvTrim, PSTR("elv")),
				_rdrTrim(&_trimGroup, 3, rdrTrim, PSTR("rdr")),
				_thrTrim(&_trimGroup, 4, thrTrim, PSTR("thr")),
				pidBnkRll(new AP_Var_group(k_pidBnkRll, PSTR("bnkRll_")), 1,
						pidBnkRllP, pidBnkRllI, pidBnkRllD, pidBnkRllAwu,
						pidBnkRllLim, pidBnkRllDFCut),
				pidPitPit(new AP_Var_group(k_pidPitPit, PSTR("pitPit_")), 1,
						pidPitPitP, pidPitPitI, pidPitPitD, pidPitPitAwu,
						pidPitPitLim, pidPitPitDFCut),
				pidSpdPit(new AP_Var_group(k_pidSpdPit, PSTR("spdPit_")), 1,
						pidSpdPitP, pidSpdPitI, pidSpdPitD, pidSpdPitAwu,
						pidSpdPitLim, pidSpdPitDFCut),
				pidYwrYaw(new AP_Var_group(k_pidYwrYaw, PSTR("ywrYaw_")), 1,
						pidYwrYawP, pidYwrYawI, pidYwrYawD, pidYwrYawAwu,
						pidYwrYawLim, pidYwrYawDFCut),
				pidHdgBnk(new AP_Var_group(k_pidHdgBnk, PSTR("hdgBnk_")), 1,
						pidHdgBnkP, pidHdgBnkI, pidHdgBnkD, pidHdgBnkAwu,
						pidHdgBnkLim, pidHdgBnkDFCut),
				pidAltThr(new AP_Var_group(k_pidAltThr, PSTR("altThr_")), 1,
						pidAltThrP, pidAltThrI, pidAltThrD, pidAltThrAwu,
						pidAltThrLim, pidAltThrDFCut),
				requireRadio(false) {

		_hal->debug->println_P(PSTR("initializing car controller"));

		_hal->rc.push_back(
				new AP_RcChannel(k_chMode, PSTR("mode_"), APM_RC, 5, 1100,
						1500, 1900, RC_MODE_IN));
		_hal->rc.push_back(
				new AP_RcChannel(k_chRoll, PSTR("roll_"), APM_RC, 0, 1200,
						1500, 1800, RC_MODE_INOUT));
		_hal->rc.push_back(
				new AP_RcChannel(k_chPitch, PSTR("pitch_"), APM_RC, 1, 1200,
						1500, 1800, RC_MODE_INOUT));
		_hal->rc.push_back(
				new AP_RcChannel(k_chThr, PSTR("thr_"), APM_RC, 2, 1100, 1100,
						1900, RC_MODE_INOUT));
		_hal->rc.push_back(
				new AP_RcChannel(k_chYaw, PSTR("yaw_"), APM_RC, 3, 1200, 1500,
						1800, RC_MODE_INOUT));
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
		// if the value of the throttle is less than 5% cut motor power
		} else if (requireRadio && _hal->rc[ch_thr]->getRadioPosition() < 0.05) {
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
		if (_hal->rc[ch_mode]->getRadioPosition() > 0) {
			_mode = MAV_MODE_MANUAL;
		} else {
			_mode = MAV_MODE_AUTO;
		}

		// manual mode
		switch (_mode) {

		case MAV_MODE_MANUAL: {
			setAllRadioChannelsManually();

			// force auto to read new manual trim
			if (_needsTrim == false)
				_needsTrim = true;
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

			float aileron = pidBnkRll.update(
					pidHdgBnk.update(headingError, dt) - _nav->getRoll(), dt);
			float elevator = pidPitPit.update(
					-pidSpdPit.update(
							_guide->getAirSpeedCommand() - _nav->getAirSpeed(),
							dt) - _nav->getPitch(), dt);
			float rudder = pidYwrYaw.update(-_nav->getYawRate(), dt);
			// desired yaw rate is zero, needs washout
			float throttle = pidAltThr.update(
					_guide->getAltitudeCommand() - _nav->getAlt(), dt);

			// if needs trim
			if (_needsTrim) {
				// need to subtract current controller deflections so control
				// surfaces are actually at the same position as manual flight
				_ailTrim = _hal->rc[ch_roll]->getRadioPosition() - aileron;
				_elvTrim = _hal->rc[ch_pitch]->getRadioPosition() - elevator;
				_rdrTrim = _hal->rc[ch_yaw]->getRadioPosition() - rudder;
				_thrTrim = _hal->rc[ch_thr]->getRadioPosition() - throttle;
				_needsTrim = false;
			}

			// actuator mixing/ output
			_hal->rc[ch_roll]->setPosition(
					aileron + _rdrAilMix * rudder + _ailTrim);
			_hal->rc[ch_yaw]->setPosition(rudder + _rdrTrim);
			_hal->rc[ch_pitch]->setPosition(elevator + _elvTrim);
			_hal->rc[ch_thr]->setPosition(throttle + _thrTrim);

			//_hal->debug->println("automode");
			

			// heading debug
//			Serial.print("heading command: "); Serial.println(_guide->getHeadingCommand());
//			Serial.print("heading: "); Serial.println(_nav->getYaw());
//			Serial.print("heading error: "); Serial.println(headingError);

			// alt debug
//			Serial.print("alt command: "); Serial.println(_guide->getAltitudeCommand());
//			Serial.print("alt: "); Serial.println(_nav->getAlt());
//			Serial.print("alt error: "); Serial.println(_guide->getAltitudeCommand() - _nav->getAlt());
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

#endif /* CONTROLLERPLANE_H_ */
