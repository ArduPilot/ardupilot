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
        ch_mode = 0, // note scicoslab channels set mode, left, right, front, back order
        ch_right,
        ch_left,
        ch_front,
        ch_back,
        ch_roll,
        ch_pitch,
        ch_thrust,
        ch_yaw
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
        AP_Controller(nav, guide, hal, new AP_ArmingMechanism(hal,ch_thrust,ch_yaw,0.1,-0.9,0.9), ch_mode),
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
        _thrustMix(0), _pitchMix(0), _rollMix(0), _yawMix(0),
        _cmdRoll(0), _cmdPitch(0), _cmdYawRate(0) {
        _hal->debug->println_P(PSTR("initializing quad controller"));

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

private:
    BlockPIDDfb pidRoll, pidPitch, pidYaw;
    BlockPID pidYawRate;
    BlockPIDDfb pidPN, pidPE, pidPD;

    float _thrustMix, _pitchMix, _rollMix, _yawMix;
    float _cmdRoll, _cmdPitch, _cmdYawRate;

    void manualLoop(const float dt) {
        setAllRadioChannelsManually();
        _cmdRoll = -0.5 * _hal->rc[ch_roll]->getPosition();
        _cmdPitch = -0.5 * _hal->rc[ch_pitch]->getPosition();
        _cmdYawRate = -1 * _hal->rc[ch_yaw]->getPosition();
        _thrustMix = _hal->rc[ch_thrust]->getPosition();
        autoAttitudeLoop(dt);
    }

    void autoLoop(const float dt) {
        autoPositionLoop(dt);
        autoAttitudeLoop(dt);

        // XXX currently auto loop not tested, so
        // put vehicle in standby
        _hal->setState(MAV_STATE_STANDBY);
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
            if (fabs(_hal->rc[ch_thrust]->getRadioPosition()) < 0.1) {
                setAllRadioChannelsToNeutral();
            } else {
                _hal->rc[ch_right]->setPosition(_thrustMix - _rollMix + _yawMix);
                _hal->rc[ch_left]->setPosition(_thrustMix + _rollMix + _yawMix);
                _hal->rc[ch_front]->setPosition(_thrustMix + _pitchMix - _yawMix);
                _hal->rc[ch_back]->setPosition(_thrustMix - _pitchMix - _yawMix);
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
