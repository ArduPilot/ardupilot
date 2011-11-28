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
    ControllerQuad(AP_Navigator * nav, AP_Guide * guide,
                   AP_HardwareAbstractionLayer * hal) :
        AP_Controller(nav, guide, hal, new AP_ArmingMechanism(hal,this,ch_thrust,ch_yaw,0.1,-0.9,0.9), ch_mode, k_cntrl),
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
        pidTilt(new AP_Var_group(k_pidTilt, PSTR("TILT_")), 1, PID_TILT_P,
              PID_TILT_I, PID_TILT_D, PID_TILT_AWU, PID_TILT_LIM, PID_TILT_DFCUT),
        pidPD(new AP_Var_group(k_pidPD, PSTR("DOWN_")), 1, PID_POS_Z_P,
              PID_POS_Z_I, PID_POS_Z_D, PID_POS_Z_AWU, PID_POS_Z_LIM, PID_POS_Z_DFCUT),
        _thrustMix(0), _pitchMix(0), _rollMix(0), _yawMix(0),
        _cmdRoll(0), _cmdPitch(0), _cmdYawRate(0) {
        _hal->debug->println_P(PSTR("initializing quad controller"));

        /*
         * allocate radio channels
         * the order of the channels has to match the enumeration above
         */
        _hal->rc.push_back(
            new AP_RcChannel(k_chMode, PSTR("MODE_"), hal->radio, 5, 1100,
                             1500, 1900, RC_MODE_IN, false));
        _hal->rc.push_back(
            new AP_RcChannel(k_chRight, PSTR("RIGHT_"), hal->radio, 0, 1100,
                             1100, 1900, RC_MODE_OUT, false));
        _hal->rc.push_back(
            new AP_RcChannel(k_chLeft, PSTR("LEFT_"), hal->radio, 1, 1100,
                             1100, 1900, RC_MODE_OUT, false));

        _hal->rc.push_back(
            new AP_RcChannel(k_chFront, PSTR("FRONT_"), hal->radio, 2, 1100,
                             1100, 1900, RC_MODE_OUT, false));
        _hal->rc.push_back(
            new AP_RcChannel(k_chBack, PSTR("BACK_"), hal->radio, 3, 1100,
                             1100, 1900, RC_MODE_OUT, false));
        _hal->rc.push_back(
            new AP_RcChannel(k_chRoll, PSTR("ROLL_"), hal->radio, 0, 1100,
                             1500, 1900, RC_MODE_IN, false));
        _hal->rc.push_back(
            new AP_RcChannel(k_chPitch, PSTR("PITCH_"), hal->radio, 1, 1100,
                             1500, 1900, RC_MODE_IN, false));
        _hal->rc.push_back(
            new AP_RcChannel(k_chThr, PSTR("THRUST_"), hal->radio, 2, 1100,
                             1100, 1900, RC_MODE_IN, false));
        _hal->rc.push_back(
            new AP_RcChannel(k_chYaw, PSTR("YAW_"), hal->radio, 3, 1100, 1500,
                             1900, RC_MODE_IN, false));
    }

private:
    // methods
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
    }
    void autoPositionLoop(float dt) {
        // XXX need to add waypoint coordinates
        //float cmdNorthTilt = pidPN.update(_guide->getPNError(),_nav->getVN(),dt);
        //float cmdEastTilt = pidPE.update(_guide->getPEError(),_nav->getVE(),dt);
        float cmdTilt = pidTilt.update(_guide->getDistanceToNextWaypoint(),dt);
        float cmdDown = pidPD.update(_guide->getPDError(),_nav->getVD(),dt);

        _cmdPitch = -cmdTilt*cos(_guide->getHeadingError());
        _cmdRoll = cmdTilt*sin(_guide->getHeadingError());
        _cmdYawRate = pidYaw.update(_guide->getHeadingError(),_nav->getYawRate(),dt); // always points to next waypoint
        _thrustMix = THRUST_HOVER_OFFSET - cmdDown;

        // "thrust-trim-adjust"
        if (fabs(_cmdRoll) > 0.5) _thrustMix *= 1.13949393;
        else _thrustMix /= cos(_cmdRoll);

        if (fabs(_cmdPitch) > 0.5) _thrustMix *= 1.13949393;
        else _thrustMix /= cos(_cmdPitch);

        // debug for position loop
        _hal->debug->printf_P(PSTR("cmd: tilt(%f), down(%f), pitch(%f), roll(%f)\n"),cmdTilt,cmdDown,_cmdPitch,_cmdRoll);
    }
    void autoAttitudeLoop(float dt) {
        _rollMix = pidRoll.update(_cmdRoll - _nav->getRoll(),
                                  _nav->getRollRate(), dt);
        _pitchMix = pidPitch.update(_cmdPitch - _nav->getPitch(),
                                    _nav->getPitchRate(), dt);
        _yawMix = pidYawRate.update(_cmdYawRate - _nav->getYawRate(), dt);
    }
    void setMotors() {
        // turn all motors off if below 0.1 throttle
        if (fabs(_hal->rc[ch_thrust]->getRadioPosition()) < 0.1) {
            setAllRadioChannelsToNeutral();
        } else {
            _hal->rc[ch_right]->setPosition(_thrustMix - _rollMix + _yawMix);
            _hal->rc[ch_left]->setPosition(_thrustMix + _rollMix + _yawMix);
            _hal->rc[ch_front]->setPosition(_thrustMix + _pitchMix - _yawMix);
            _hal->rc[ch_back]->setPosition(_thrustMix - _pitchMix - _yawMix);
        }
    }

    void handleFailsafe() {
        // turn off
        setMode(MAV_MODE_LOCKED);
    }

    // attributes
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
        k_pidTilt,
        k_pidPD,
        k_pidRoll,
        k_pidPitch,
        k_pidYawRate,
        k_pidYaw,
    };
    BlockPIDDfb pidRoll, pidPitch, pidYaw;
    BlockPID pidYawRate;
    BlockPID pidTilt;
    BlockPIDDfb pidPD;
    float _thrustMix, _pitchMix, _rollMix, _yawMix;
    float _cmdRoll, _cmdPitch, _cmdYawRate;
};

} // namespace apo

#endif /* CONTROLLERQUAD_H_ */
// vim:ts=4:sw=4:expandtab
