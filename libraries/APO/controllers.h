#ifndef defaultControllers_H
#define defaultControllers_H

#include "AP_Controller.h"
#include "AP_HardwareAbstractionLayer.h"
#include "../AP_Common/AP_Var.h"
#include <avr/pgmspace.h>
#include "AP_Navigator.h"

namespace apo {

class CarController: public AP_Controller {
private:
	// control mode
	AP_Var_group _group;AP_Uint8 _mode;
	enum {
		CH_MODE = 0, CH_STR, CH_THR
	};
	PidDFB2 pidStr;
	Pid2 pidThr;
public:
	CarController(AP_Var::Key cntrlKey, AP_Var::Key pidStrKey,
			AP_Var::Key pidThrKey, AP_Navigator * nav, AP_Guide * guide,
			AP_HardwareAbstractionLayer * hal) :
		AP_Controller(nav, guide, hal), _group(cntrlKey, PSTR("CNTRL_")),
				_mode(&_group, 1, 0, PSTR("MODE")),
				pidStr(pidStrKey, PSTR("STR_"), 1.0, 0, 0, 0, 3),
				pidThr(pidThrKey, PSTR("THR_"), 0.6, 0.5, 0, 1, 3) {
		_hal->debug->println_P(PSTR("initializing car controller"));

		_hal->rc.push_back(
				new AP_RcChannel(k_chMode, PSTR("MODE_"), APM_RC, 7,1100,1500,1900));
		_hal->rc.push_back(
				new AP_RcChannel(k_chStr, PSTR("STR_"), APM_RC, 0,1100,1540,1900));
		_hal->rc.push_back(
				new AP_RcChannel(k_chThr, PSTR("THR_"), APM_RC, 1,1100,1500,1900));
	}
	virtual void update(const float & dt) {
		// read mode switch
		_hal->rc[CH_MODE]->setPwm(_hal->rc[CH_MODE]->readRadio());
		// manual
		if (_hal->rc[CH_MODE]->getPosition() > 0) {
			_hal->rc[CH_STR]->setPwm(_hal->rc[CH_STR]->readRadio());
			_hal->rc[CH_THR]->setPwm(_hal->rc[CH_THR]->readRadio());
			//_hal->debug->println("manual");

		} else { // auto
			float headingError = _guide->headingCommand - _nav->getHeading();
			if (headingError > 180 * deg2Rad)
				headingError -= 360 * deg2Rad;
			if (headingError < -180 * deg2Rad)
				headingError += 360 * deg2Rad;
			_hal->rc[CH_STR]->setPosition(pidStr.update(headingError,_nav->getYawRate(),dt));
			_hal->rc[CH_THR]->setPosition(pidThr.update(_guide->groundSpeedCommand - _nav->getGroundSpeed(),dt));
			//_hal->debug->println("automode");
		}
	}
};

#if CUSTOM_INCLUDES == CUSTOM_MIKROKOPTER
#include "mikrokopter.h"
#endif

class QuadController: public AP_Controller {
public:

	/**
	 * note that these are not the controller radio channel numbers, they are just
	 * unique keys so they can be reaccessed from the hal rc vector
	 */
	enum autoChannel_t {
		CH_MODE = 0, // note scicoslab channels set mode, left, right, front, back order
		CH_LEFT, // this enum must match this
		CH_RIGHT,
		CH_FRONT,
		CH_BACK,
		CH_ROLL,
		CH_PITCH,
		CH_YAW,
		CH_THRUST
	};

	QuadController(AP_Navigator * nav, AP_Guide * guide,
			AP_HardwareAbstractionLayer * hal) :
				AP_Controller(nav, guide, hal),
				pidRoll(k_pidRoll, PSTR("ROLL_"), PID_ATT_P, PID_ATT_I,
						PID_ATT_D, PID_ATT_AWU, PID_ATT_LIM),
				pidPitch(k_pidPitch, PSTR("PITCH_"), PID_ATT_P, PID_ATT_I,
						PID_ATT_D, PID_ATT_AWU, PID_ATT_LIM),
				pidYaw(k_pidYaw, PSTR("YAW_"), PID_YAWPOS_P, PID_YAWPOS_I,
						PID_YAWPOS_D, PID_YAWPOS_AWU, PID_YAWPOS_LIM),
				pidYawRate(k_pidYawRate, PSTR("YAWRATE_"), PID_YAWSPEED_P,
						PID_YAWSPEED_I, PID_YAWSPEED_D, PID_YAWSPEED_AWU,
						PID_YAWSPEED_LIM),
				pidPN(k_pidPN, PSTR("NORTH_"), PID_POS_P,
						PID_POS_I, PID_POS_D, PID_POS_AWU, PID_POS_LIM),
				pidPE(k_pidPE, PSTR("EAST_"), PID_POS_P, PID_POS_I,
						PID_POS_D, PID_POS_AWU, PID_POS_LIM),
				pidPD(k_pidPD, PSTR("DOWN_"), PID_POS_Z_P, PID_POS_Z_I,
						PID_POS_Z_D, PID_POS_Z_AWU, PID_POS_Z_LIM)
	{
		/*
		 * allocate radio channels
		 * the order of the channels has to match the enumeration above
		 */
		_hal->rc.push_back(
		new AP_RcChannel(k_chMode, PSTR("MODE_"), APM_RC, 7, 1100, 1500, 1900, RC_MODE_IN));
		_hal->rc.push_back(
				new AP_RcChannel(k_chLeft, PSTR("LEFT_"), APM_RC, 0, 1100, 1100, 1900, RC_MODE_OUT));
		_hal->rc.push_back(
				new AP_RcChannel(k_chRight, PSTR("RIGHT_"), APM_RC, 1, 1100, 1100, 1900, RC_MODE_OUT));
		_hal->rc.push_back(
				new AP_RcChannel(k_chFront, PSTR("FRONT_"), APM_RC, 2, 1100, 1100, 1900, RC_MODE_OUT));
		_hal->rc.push_back(
				new AP_RcChannel(k_chBack, PSTR("BACK_"), APM_RC, 3, 1100, 1100, 1900, RC_MODE_OUT));
		_hal->rc.push_back(
				new AP_RcChannel(k_chRoll, PSTR("ROLL_"), APM_RC, 0, 1100, 1500, 1900, RC_MODE_IN));
		_hal->rc.push_back(
				new AP_RcChannel(k_chPitch, PSTR("PITCH_"), APM_RC, 1, 1100, 1500, 1900, RC_MODE_IN));
		_hal->rc.push_back(
				new AP_RcChannel(k_chYaw, PSTR("YAW_"), APM_RC, 2, 1100, 1500, 1900, RC_MODE_IN));
		_hal->rc.push_back( // -1 -> 0 maps to 1200, linear 0-1 -> 1200-1800
				new AP_RcChannel(k_chThr, PSTR("THRUST_"), APM_RC, 3, 1100, 1100, 1900, RC_MODE_IN));
	}

	virtual void update(const float & dt) {

			if (commLost()) {
				_mode = MAV_MODE_FAILSAFE;
				_hal->rc[CH_LEFT]->setPosition(0);
				_hal->rc[CH_RIGHT]->setPosition(0);
				_hal->rc[CH_FRONT]->setPosition(0);
				_hal->rc[CH_BACK]->setPosition(0);
			}

			// read and set pwm so they can be read as positions later
			_hal->rc[CH_MODE]->setPwm(_hal->rc[CH_MODE]->readRadio());
			_hal->rc[CH_ROLL]->setPwm(_hal->rc[CH_ROLL]->readRadio());
			_hal->rc[CH_PITCH]->setPwm(_hal->rc[CH_PITCH]->readRadio());
			_hal->rc[CH_YAW]->setPwm(_hal->rc[CH_YAW]->readRadio());
			_hal->rc[CH_THRUST]->setPwm(_hal->rc[CH_THRUST]->readRadio());

			// manual mode
			float mixRemoteWeight = 0;
			if (_hal->rc[CH_MODE]->getPwm() > 1350) mixRemoteWeight = 1;

			// "mix manual"
			float cmdRoll = _hal->rc[CH_ROLL]->getPosition() * mixRemoteWeight;
			float cmdPitch = _hal->rc[CH_PITCH]->getPosition() * mixRemoteWeight;
			float cmdYawRate = _hal->rc[CH_YAW]->getPosition() * mixRemoteWeight;
			float thrustMix = _hal->rc[CH_THRUST]->getPosition() * mixRemoteWeight;

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

			// attitude loop
			float rollMix = pidRoll.update(cmdRoll - _nav->getRoll(),_nav->getRollRate(),dt);
			float pitchMix = pidPitch.update(cmdPitch - _nav->getPitch(),_nav->getPitchRate(),dt);
			float yawMix = pidYawRate.update(cmdYawRate - _nav->getYawRate(),dt);

			_hal->rc[CH_LEFT]->setPosition(thrustMix + rollMix + yawMix);
			_hal->rc[CH_RIGHT]->setPosition(thrustMix - rollMix + yawMix);
			_hal->rc[CH_FRONT]->setPosition(thrustMix + pitchMix - yawMix);
			_hal->rc[CH_BACK]->setPosition(thrustMix - pitchMix - yawMix);

			_hal->debug->printf("L: %f\t R: %f\t F: %f\t B: %f\n",
					_hal->rc[CH_LEFT]->getPosition(),
					_hal->rc[CH_RIGHT]->getPosition(),
					_hal->rc[CH_FRONT]->getPosition(),
					_hal->rc[CH_BACK]->getPosition());

			_hal->debug->printf("rollMix: %f\t pitchMix: %f\t yawMix: %f\t thrustMix: %f\n",
								rollMix,
								pitchMix,
								yawMix,
								thrustMix);

//			_hal->debug->printf("thrust pwm: %d\n",_hal->rc[CH_THRUST]->readRadio());
		}

private:
	PidDFB2 pidRoll, pidPitch, pidYaw, pidPN, pidPE, pidPD;
	Pid2 pidYawRate;
};

/*
 class PlaneController : public AP_Controller
 {
 private:
 // state
 AP_Float roll;
 AP_Float airspeed;
 AP_Float velocity;
 AP_Float heading;

 // servo positions
 AP_Float steering;
 AP_Float throttle;

 // control variables
 AP_Float headingCommand;
 AP_Float airspeedCommand;
 AP_Float rollCommand;

 // channels
 static const uint8_t chRoll = 0;
 static const uint8_t chPitch = 1;
 static const uint8_t chYaw = 2;

 public:
 PlaneController(AP_Var::Key chRollKey, AP_Var::Key chPitchKey, AP_Var::Key chYawKey,
 AP_Var::Key pidRollKey, AP_Var::Key pidPitchKey, AP_Var::Key pidYawKey) : {
 // rc channels
 addCh(new AP_RcChannelSimple(chRollKey,PSTR("ROLL"),APM_RC,chRoll,45));
 addCh(new AP_RcChannelSimple(chPitchKey,PSTR("PTCH"),APM_RC,chPitch,45));
 addCh(new AP_RcChannelSimple(chYawKey,PSTR("YAW"),APM_RC,chYaw,45));

 // pitch control loop
 #if AIRSPEED_SENSOR == ENABLED
 // pitch control loop w/ airspeed
 addBlock(new SumGain(airspeedCommand,AP_Float_unity,airspeed,AP_Float_negative_unity));
 #else
 // cross feed variables
 addBlock(new SumGain(roll,kffPitchCompk,throttleServo,kffT2P));
 #endif
 addBlock(new Pid(pidPitchKey,PSTR("PTCH"),0.1,0,0,1,20));
 addBlock(new ToServo(getRc(chPitch)));

 // roll control loop
 addBlock(new SumGain(headingCommand,one,heading,negOne));
 addBlock(new Pid(headingkP,headingKI,headingKD));
 addBlock(new Sink(rollCommand));
 addBlock(new SumGain(rollCommand,one,roll,negOne));
 addBlock(new Pid(rollKP,rollKI,rollKD));
 addBlock(new ToServo(getRc(chRoll)));

 // throttle control loop
 addBlock(new SumGain(airspeedCommand,one,airspeed,negOne));
 addBlock(new Pid(throttleKP,throttleKI,throttleKD));
 addBlock(new ToServo(getRc(chThr)));
 }
 };
 */

} // namespace apo

#endif // defaultControllers_H
// vim:ts=4:sw=4:expandtab
