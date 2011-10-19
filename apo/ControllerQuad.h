/*
 * ControllerQuad.h
 *
 *  Created on: Jun 30, 2011
 *      Author: jgoppert
 */

#ifndef CONTROLLERQUAD_H_
#define CONTROLLERQUAD_H_

#include "../APO/AP_Controller.h"
#include "../APO/AP_BatteryMonitor.h"
#include "../APO/AP_ArmingMechanism.h"

namespace apo {

class ControllerQuad: public AP_Controller {
public:

	/**
	 * note that these are not the controller radio channel numbers, they are just
	 * unique keys so they can be reaccessed from the hal rc vector
	 */
	enum {
		CH_MODE = 0, // note scicoslab channels set mode, left, right, front, back order
		CH_RIGHT,
		CH_LEFT,
		CH_FRONT,
		CH_BACK,
		CH_ROLL,
		CH_PITCH,
		CH_THRUST,
		CH_YAW
	};

	// must match channel enum
	enum {
		k_chMode = k_radioChannelsStart,
		k_chRight,
		k_chLeft,
		k_chFront,
		k_chBack,
		k_chRoll,
		k_chPitch,
		k_chThr,
		k_chYaw
	};

	enum {
		k_pidGroundSpeed2Throttle = k_controllersStart,
		k_pidStr,
		k_pidPN,
		k_pidPE,
		k_pidPD,
		k_pidRoll,
		k_pidPitch,
		k_pidYawRate,
		k_pidYaw,
	};

	ControllerQuad(AP_Navigator * nav, AP_Guide * guide,
			AP_HardwareAbstractionLayer * hal) :
				AP_Controller(nav, guide, hal),
				pidRoll(new AP_Var_group(k_pidRoll, PSTR("ROLL_")), 1,
						PID_ATT_P, PID_ATT_I, PID_ATT_D, PID_ATT_AWU,
						PID_ATT_LIM, PID_ATT_DFCUT),
				pidPitch(new AP_Var_group(k_pidPitch, PSTR("PITCH_")), 1,
						PID_ATT_P, PID_ATT_I, PID_ATT_D, PID_ATT_AWU,
						PID_ATT_LIM, PID_ATT_DFCUT),
				pidYaw(new AP_Var_group(k_pidYaw, PSTR("YAW_")), 1,
						PID_YAWPOS_P, PID_YAWPOS_I, PID_YAWPOS_D,
						PID_YAWPOS_AWU, PID_YAWPOS_LIM, PID_ATT_DFCUT),
				pidYawRate(new AP_Var_group(k_pidYawRate, PSTR("YAWRT_")), 1,
						PID_YAWSPEED_P, PID_YAWSPEED_I, PID_YAWSPEED_D,
						PID_YAWSPEED_AWU, PID_YAWSPEED_LIM, PID_YAWSPEED_DFCUT),
				pidPN(new AP_Var_group(k_pidPN, PSTR("NORTH_")), 1, PID_POS_P,
						PID_POS_I, PID_POS_D, PID_POS_AWU, PID_POS_LIM, PID_POS_DFCUT),
				pidPE(new AP_Var_group(k_pidPE, PSTR("EAST_")), 1, PID_POS_P,
						PID_POS_I, PID_POS_D, PID_POS_AWU, PID_POS_LIM, PID_POS_DFCUT),
				pidPD(new AP_Var_group(k_pidPD, PSTR("DOWN_")), 1, PID_POS_Z_P,
						PID_POS_Z_I, PID_POS_Z_D, PID_POS_Z_AWU, PID_POS_Z_LIM, PID_POS_DFCUT),
   				_armingMechanism(hal,CH_THRUST,CH_YAW,0.1,-0.9,0.9), _thrustMix(0), _pitchMix(0), _rollMix(0), _yawMix(0),
				_cmdRoll(0), _cmdPitch(0), _cmdYawRate(0), _mode(MAV_MODE_LOCKED) {
		/*
		 * allocate radio channels
		 * the order of the channels has to match the enumeration above
		 */
		_hal->rc.push_back(
				new AP_RcChannel(k_chMode, PSTR("MODE_"), APM_RC, 5, 1100,
						1500, 1900, RC_MODE_IN, false));
		_hal->rc.push_back(
				new AP_RcChannel(k_chRight, PSTR("RIGHT_"), APM_RC, 0, 1100,
						1100, 1900, RC_MODE_OUT, false));
		_hal->rc.push_back(
				new AP_RcChannel(k_chLeft, PSTR("LEFT_"), APM_RC, 1, 1100,
						1100, 1900, RC_MODE_OUT, false));

		_hal->rc.push_back(
				new AP_RcChannel(k_chFront, PSTR("FRONT_"), APM_RC, 2, 1100,
						1100, 1900, RC_MODE_OUT, false));
		_hal->rc.push_back(
				new AP_RcChannel(k_chBack, PSTR("BACK_"), APM_RC, 3, 1100,
						1100, 1900, RC_MODE_OUT, false));
		_hal->rc.push_back(
				new AP_RcChannel(k_chRoll, PSTR("ROLL_"), APM_RC, 0, 1100,
						1500, 1900, RC_MODE_IN, false));
		_hal->rc.push_back(
				new AP_RcChannel(k_chPitch, PSTR("PITCH_"), APM_RC, 1, 1100,
						1500, 1900, RC_MODE_IN, false));
		_hal->rc.push_back(
				new AP_RcChannel(k_chThr, PSTR("THRUST_"), APM_RC, 2, 1100,
						1100, 1900, RC_MODE_IN, false));
		_hal->rc.push_back(
				new AP_RcChannel(k_chYaw, PSTR("YAW_"), APM_RC, 3, 1100, 1500,
						1900, RC_MODE_IN, false));
	}

	virtual void update(const float & dt) {
		//_hal->debug->printf_P(PSTR("thr: %f, yaw: %f\n"),_hal->rc[CH_THRUST]->getRadioPosition(),_hal->rc[CH_YAW]->getRadioPosition());

		_armingMechanism.update(dt);

		// determine flight mode
		//
		// check for heartbeat from gcs, if not found go to failsafe
		if (_hal->heartBeatLost()) {
			_mode = MAV_MODE_FAILSAFE;
			_hal->gcs->sendText(SEVERITY_HIGH, PSTR("configure gcs to send heartbeat"));
		// if battery less than 5%, go to failsafe
		} else if (_hal->batteryMonitor->getPercentage() < 5) {
			_mode = MAV_MODE_FAILSAFE;
			_hal->gcs->sendText(SEVERITY_HIGH, PSTR("recharge battery"));
		// manual/auto switch
		} else {
			// if all emergencies cleared, fall back to standby
			if (_hal->getState()==MAV_STATE_EMERGENCY) _hal->setState(MAV_STATE_STANDBY);
		   	if (_hal->rc[CH_MODE]->getRadioPosition() > 0) _mode = MAV_MODE_MANUAL;
			else _mode = MAV_MODE_AUTO;
		}

		// handle flight modes
		switch(_mode) {

			case MAV_MODE_LOCKED: {
				_hal->setState(MAV_STATE_STANDBY);
				break;
			}

			case MAV_MODE_FAILSAFE: {
				_hal->setState(MAV_STATE_EMERGENCY);
				break;
			}

			case MAV_MODE_MANUAL: {
				manualPositionLoop();
				autoAttitudeLoop(dt);
				break;
			}

			case MAV_MODE_AUTO: {
				// until position loop is tested just
				// go to standby
				_hal->setState(MAV_STATE_STANDBY);

				//attitudeLoop();
				// XXX autoPositionLoop NOT TESTED, don't uncomment yet
				//autoPositionLoop(dt);
				//autoAttitudeLoop(dt);
				break;
			}

			default: {
				_hal->gcs->sendText(SEVERITY_HIGH, PSTR("unknown mode"));
				_hal->setState(MAV_STATE_EMERGENCY);
			}
		}

		// this sends commands to motors
		setMotors();
	}

	virtual MAV_MODE getMode() {
		return (MAV_MODE) _mode.get();
	}

private:

	AP_Uint8 _mode;
	BlockPIDDfb pidRoll, pidPitch, pidYaw;
	BlockPID pidYawRate;
	BlockPIDDfb pidPN, pidPE, pidPD;
	AP_ArmingMechanism _armingMechanism;

	float _thrustMix, _pitchMix, _rollMix, _yawMix;
	float _cmdRoll, _cmdPitch, _cmdYawRate;

	void manualPositionLoop() {
		setAllRadioChannelsManually();
		_cmdRoll = -0.5 * _hal->rc[CH_ROLL]->getPosition();
		_cmdPitch = -0.5 * _hal->rc[CH_PITCH]->getPosition();
		_cmdYawRate = -1 * _hal->rc[CH_YAW]->getPosition();
		_thrustMix = _hal->rc[CH_THRUST]->getPosition();
	}

	void autoPositionLoop(float dt) {
		 float cmdNorthTilt = pidPN.update(_nav->getPN(),_nav->getVN(),dt);
		 float cmdEastTilt = pidPE.update(_nav->getPE(),_nav->getVE(),dt);
		 float cmdDown = pidPD.update(_nav->getPD(),_nav->getVD(),dt);

		 // "transform-to-body"
		 {
			 float trigSin = sin(-_nav->getYaw());
			 float trigCos = cos(-_nav->getYaw());
			 _cmdPitch = cmdEastTilt * trigCos - cmdNorthTilt * trigSin;
			 _cmdRoll = -cmdEastTilt * trigSin + cmdNorthTilt * trigCos;
			 // note that the north tilt is negative of the pitch
		 }
		_cmdYawRate = 0;

		 _thrustMix = THRUST_HOVER_OFFSET + cmdDown;

		 // "thrust-trim-adjust"
		 if (fabs(_cmdRoll) > 0.5) _thrustMix *= 1.13949393;
		 else _thrustMix /= cos(_cmdRoll);

		 if (fabs(_cmdPitch) > 0.5) _thrustMix *= 1.13949393;
		 else _thrustMix /= cos(_cmdPitch);
	}

	void autoAttitudeLoop(float dt) {
		_rollMix = pidRoll.update(_cmdRoll - _nav->getRoll(),
				_nav->getRollRate(), dt);
		_pitchMix = pidPitch.update(_cmdPitch - _nav->getPitch(),
				_nav->getPitchRate(), dt);
		_yawMix = pidYawRate.update(_cmdYawRate - _nav->getYawRate(), dt);
	}

	void setMotors() {

		switch (_hal->getState()) {

			case MAV_STATE_ACTIVE: {
				digitalWrite(_hal->aLedPin, HIGH);
				// turn all motors off if below 0.1 throttle
				if (_hal->rc[CH_THRUST]->getRadioPosition() < 0.1) {
					setAllRadioChannelsToNeutral();
				} else {
					_hal->rc[CH_RIGHT]->setPosition(_thrustMix - _rollMix + _yawMix);
					_hal->rc[CH_LEFT]->setPosition(_thrustMix + _rollMix + _yawMix);
					_hal->rc[CH_FRONT]->setPosition(_thrustMix + _pitchMix - _yawMix);
					_hal->rc[CH_BACK]->setPosition(_thrustMix - _pitchMix - _yawMix);
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

#endif /* CONTROLLERQUAD_H_ */
