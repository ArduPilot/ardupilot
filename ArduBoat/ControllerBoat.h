/*
 * ControllerBoat.h
 *
 *  Created on: Jun 30, 2011
 *      Author: jgoppert
 */

#ifndef CONTROLLERBOAT_H_
#define CONTROLLERBOAT_H_

#include "../APO/AP_Controller.h"

namespace apo {

class ControllerBoat: public AP_Controller {
public:
    ControllerBoat(AP_Navigator * nav, AP_Guide * guide,
                   AP_HardwareAbstractionLayer * hal) :
        AP_Controller(nav, guide, hal,new AP_ArmingMechanism(hal,ch_thrust,ch_str,0.1,-0.9,0.9), ch_mode),
        pidStr(new AP_Var_group(k_pidStr, PSTR("STR_")), 1, steeringP,
               steeringI, steeringD, steeringIMax, steeringYMax,steeringDFCut),
        pidThrust(new AP_Var_group(k_pidThrust, PSTR("THR_")), 1, throttleP,
                  throttleI, throttleD, throttleIMax, throttleYMax,
                  throttleDFCut), _strCmd(0), _thrustCmd(0)
    {
        _hal->debug->println_P(PSTR("initializing boat controller"));

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
    // methdos
    void manualLoop(const float dt) {
        setAllRadioChannelsManually();
        _strCmd = _hal->rc[ch_str]->getRadioPosition();
        _thrustCmd = _hal->rc[ch_thrust]->getRadioPosition();
    }
    void autoLoop(const float dt) {
        _strCmd = pidStr.update(_guide->getHeadingError(), _nav->getYawRate(), dt);
        _thrustCmd = pidThrust.update(
                         _guide->getGroundSpeedCommand()
                         - _nav->getGroundSpeed(), dt);
    }
    void setMotorsActive() {
        // turn all motors off if below 0.1 throttle
        if (fabs(_hal->rc[ch_thrust]->getRadioPosition()) < 0.1) {
            setAllRadioChannelsToNeutral();
        } else {
            _hal->rc[ch_thrust]->setPosition(_thrustCmd);
            _hal->rc[ch_str]->setPosition(_strCmd);
        }
    }

    // attributes
    enum {
        ch_mode = 0, ch_str, ch_thrust
    };
    enum {
        k_chMode = k_radioChannelsStart, k_chStr, k_chThrust
    };
    enum {
        k_pidStr = k_controllersStart, k_pidThrust
    };
    BlockPIDDfb pidStr;
    BlockPID pidThrust;
    float _strCmd, _thrustCmd;
};

} // namespace apo

#endif /* CONTROLLERBOAT_H_ */
// vim:ts=4:sw=4:expandtab
