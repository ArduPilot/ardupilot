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
public:
    ControllerTank(AP_Navigator * nav, AP_Guide * guide,
                   AP_HardwareAbstractionLayer * hal) :
        AP_Controller(nav, guide, hal, new AP_ArmingMechanism(hal,ch_thrust,ch_str,0.1,-0.9,0.9),ch_mode,k_cntrl),
        pidStr(new AP_Var_group(k_pidStr, PSTR("STR_")), 1, steeringP,
               steeringI, steeringD, steeringIMax, steeringYMax),
        pidThr(new AP_Var_group(k_pidThr, PSTR("THR_")), 1, throttleP,
               throttleI, throttleD, throttleIMax, throttleYMax,
               throttleDFCut), _headingOutput(0), _throttleOutput(0) {
        _hal->debug->println_P(PSTR("initializing tank controller"));

        _hal->rc.push_back(
            new AP_RcChannel(k_chMode, PSTR("MODE_"), APM_RC, 5, 1100,
                             1500, 1900, RC_MODE_IN, false));
        _hal->rc.push_back(
            new AP_RcChannel(k_chLeft, PSTR("LEFT_"), APM_RC, 0, 1100, 1500,
                             1900, RC_MODE_OUT, false));
        _hal->rc.push_back(
            new AP_RcChannel(k_chRight, PSTR("RIGHT_"), APM_RC, 1, 1100, 1500,
                             1900, RC_MODE_OUT, false));
        _hal->rc.push_back(
            new AP_RcChannel(k_chStr, PSTR("STR_"), APM_RC, 0, 1100, 1500,
                             1900, RC_MODE_IN, false));
        _hal->rc.push_back(
            new AP_RcChannel(k_chThr, PSTR("THR_"), APM_RC, 1, 1100, 1500,
                             1900, RC_MODE_IN, false));
    }

private:
    // methods
    void manualLoop(const float dt) {
        setAllRadioChannelsManually();
        _headingOutput = _hal->rc[ch_str]->getPosition();
        _throttleOutput = _hal->rc[ch_thrust]->getPosition();
    }
    void autoLoop(const float dt) {
        float headingError = _guide->getHeadingCommand()
                             - _nav->getYaw();
        if (headingError > 180 * deg2Rad)
            headingError -= 360 * deg2Rad;
        if (headingError < -180 * deg2Rad)
            headingError += 360 * deg2Rad;
        _headingOutput = pidStr.update(headingError, -_nav->getYawRate(), dt);
        _throttleOutput = pidThr.update(_guide->getGroundSpeedCommand()
                                        - _nav->getGroundSpeed(), dt);
    }
    void setMotorsActive() {
        // turn all motors off if below 0.1 throttle
        if (fabs(_hal->rc[ch_thrust]->getRadioPosition()) < 0.1) {
            setAllRadioChannelsToNeutral();
        } else {
            _hal->rc[ch_left]->setPosition(_throttleOutput + _headingOutput);
            _hal->rc[ch_right]->setPosition(_throttleOutput - _headingOutput);
        }
    }

    // attributes
    enum {
        k_chMode = k_radioChannelsStart, k_chLeft, k_chRight, k_chStr, k_chThr
    };
    enum {
        k_pidStr = k_controllersStart, k_pidThr
    };
    enum {
        ch_mode = 0, ch_left, ch_right, ch_str, ch_thrust
    };
    BlockPIDDfb pidStr;
    BlockPID pidThr;
    float _headingOutput;
    float _throttleOutput;
};

} // namespace apo

#endif /* CONTROLLERTANK_H_ */
// vim:ts=4:sw=4:expandtab
