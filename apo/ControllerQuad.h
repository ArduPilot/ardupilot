/*
 * ControllerQuad.h
 *
 *  Created on: Jun 30, 2011
 *      Author: jgoppert
 */

#ifndef CONTROLLERQUAD_H_
#define CONTROLLERQUAD_H_

#include "../APO/AP_Controller.h"

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
						PID_ATT_LIM),
				pidPitch(new AP_Var_group(k_pidPitch, PSTR("PITCH_")), 1,
						PID_ATT_P, PID_ATT_I, PID_ATT_D, PID_ATT_AWU,
						PID_ATT_LIM),
				pidYaw(new AP_Var_group(k_pidYaw, PSTR("YAW_")), 1,
						PID_YAWPOS_P, PID_YAWPOS_I, PID_YAWPOS_D,
						PID_YAWPOS_AWU, PID_YAWPOS_LIM),
				pidYawRate(new AP_Var_group(k_pidYawRate, PSTR("YAWRT_")), 1,
						PID_YAWSPEED_P, PID_YAWSPEED_I, PID_YAWSPEED_D,
						PID_YAWSPEED_AWU, PID_YAWSPEED_LIM, PID_YAWSPEED_DFCUT),
				pidPN(new AP_Var_group(k_pidPN, PSTR("NORTH_")), 1, PID_POS_P,
						PID_POS_I, PID_POS_D, PID_POS_AWU, PID_POS_LIM),
				pidPE(new AP_Var_group(k_pidPE, PSTR("EAST_")), 1, PID_POS_P,
						PID_POS_I, PID_POS_D, PID_POS_AWU, PID_POS_LIM),
				pidPD(new AP_Var_group(k_pidPD, PSTR("DOWN_")), 1, PID_POS_Z_P,
						PID_POS_Z_I, PID_POS_Z_D, PID_POS_Z_AWU, PID_POS_Z_LIM) {
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

		// check for heartbeat
		if (_hal->heartBeatLost()) {
			_mode = MAV_MODE_FAILSAFE;
			setAllRadioChannelsToNeutral();
			_hal->setState(MAV_STATE_EMERGENCY);
			_hal->debug->printf_P(PSTR("comm lost, send heartbeat from gcs\n"));
			return;
		// if throttle less than 5% cut motor power
		} else if (_hal->rc[CH_THRUST]->getRadioPosition() < 0.05) {
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

		// manual mode
		if (_hal->rc[CH_MODE]->getRadioPosition() > 0) {
			_mode = MAV_MODE_MANUAL;
		} else {
			_mode = MAV_MODE_AUTO;
		}

		// commands for inner loop
		float cmdRoll = 0;
		float cmdPitch = 0;
		float cmdYawRate = 0;
		float thrustMix = 0;

		switch(_mode) {

		case MAV_MODE_MANUAL: {
			setAllRadioChannelsManually();
			// "mix manual"
			cmdRoll = -0.5 * _hal->rc[CH_ROLL]->getPosition();
			cmdPitch = -0.5 * _hal->rc[CH_PITCH]->getPosition();
			cmdYawRate = -1 * _hal->rc[CH_YAW]->getPosition();
			thrustMix = _hal->rc[CH_THRUST]->getPosition();
			break;
		}

		case MAV_MODE_AUTO: {

			// XXX kills all commands, 
			// auto not currently implemented
			setAllRadioChannelsToNeutral();

			// position loop
			/*
			 float cmdNorthTilt = pidPN.update(_nav->getPN(),_nav->getVN(),dt);
			 float cmdEastTilt = pidPE.update(_nav->getPE(),_nav->getVE(),dt);
			 float cmdDown = pidPD.update(_nav->getPD(),_nav->getVD(),dt);

			 // "transform-to-body"
			 {
			 float trigSin = sin(-yaw);
			 float trigCos = cos(-yaw);
			 _cmdPitch = _cmdEastTilt * trigCos
			 - _cmdNorthTilt * trigSin;
			 _cmdRoll = -_cmdEastTilt * trigSin
			 + _cmdNorthTilt * trigCos;
			 // note that the north tilt is negative of the pitch
			 }

			 //thrustMix += THRUST_HOVER_OFFSET;

			 // "thrust-trim-adjust"
			 if (fabs(_cmdRoll) > 0.5) {
			 _thrustMix *= 1.13949393;
			 } else {
			 _thrustMix /= cos(_cmdRoll);
			 }
			 if (fabs(_cmdPitch) > 0.5) {
			 _thrustMix *= 1.13949393;
			 } else {
			 _thrustMix /= cos(_cmdPitch);
			 }
			 */
		}

		}

		// attitude loop
		float rollMix = pidRoll.update(cmdRoll - _nav->getRoll(),
				_nav->getRollRate(), dt);
		float pitchMix = pidPitch.update(cmdPitch - _nav->getPitch(),
				_nav->getPitchRate(), dt);
		float yawMix = pidYawRate.update(cmdYawRate - _nav->getYawRate(), dt);

		_hal->rc[CH_RIGHT]->setPosition(thrustMix - rollMix + yawMix);
		_hal->rc[CH_LEFT]->setPosition(thrustMix + rollMix + yawMix);
		_hal->rc[CH_FRONT]->setPosition(thrustMix + pitchMix - yawMix);
		_hal->rc[CH_BACK]->setPosition(thrustMix - pitchMix - yawMix);

		//_hal->debug->printf("R: %f\t L: %f\t F: %f\t B: %f\n",
		//_hal->rc[CH_RIGHT]->getPosition(),
		//_hal->rc[CH_LEFT]->getPosition(),
		//_hal->rc[CH_FRONT]->getPosition(),
		//_hal->rc[CH_BACK]->getPosition());

		//_hal->debug->printf(
		//		"rollMix: %f\t pitchMix: %f\t yawMix: %f\t thrustMix: %f\n",
		//		rollMix, pitchMix, yawMix, thrustMix);
		
		//_hal->debug->printf("cmdRoll: %f\t roll: %f\t rollMix: %f\n",
		//		cmdRoll, _nav->getRoll(), rollMix);
		//_hal->debug->printf("cmdPitch: %f\t pitch: %f\t pitchMix: %f\n",
		//		cmdPitch, _nav->getPitch(), pitchMix);
		//_hal->debug->printf("cmdYawRate: %f\t yawRate: %f\t yawMix: %f\n",
		//		cmdYawRate, _nav->getYawRate(), yawMix);

		//_hal->debug->printf("roll pwm: %d\t pitch pwm: %d\t yaw pwm: %d\t thrust pwm: %d\n",
		//_hal->rc[CH_ROLL]->getRadioPwm(),
		//_hal->rc[CH_PITCH]->getRadioPwm(),
		//_hal->rc[CH_YAW]->getRadioPwm(),
		//_hal->rc[CH_THRUST]->getRadioPwm());
	}
	virtual MAV_MODE getMode() {
		return (MAV_MODE) _mode.get();
	}
private:
	AP_Uint8 _mode;
	BlockPIDDfb pidRoll, pidPitch, pidYaw;
	BlockPID pidYawRate;
	BlockPIDDfb pidPN, pidPE, pidPD;

};

} // namespace apo

#endif /* CONTROLLERQUAD_H_ */
