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
        AP_Controller(nav, guide, hal,new AP_ArmingMechanism(hal,this,ch_sail,ch_str,0.1,-0.9,0.9), ch_mode, k_cntrl),
        pidStr(new AP_Var_group(k_pidStr, PSTR("STR_")), 1, steeringP,
               steeringI, steeringD, steeringIMax, steeringYMax)
    {
        _hal->debug->println_P(PSTR("initializing boat controller"));

        _hal->rc.push_back(
            new AP_RcChannel(k_chMode, PSTR("MODE_"), hal->radio, 5, 1100,
                             1500, 1900, RC_MODE_IN, false));
        _hal->rc.push_back(
            new AP_RcChannel(k_chStr, PSTR("STR_"), hal->radio, 3, 1100, 1500,
                             1900, RC_MODE_INOUT, false));
        _hal->rc.push_back(
            new AP_RcChannel(k_chSail, PSTR("SAIL_"), hal->radio, 2, 1100, 1500,
                             1900, RC_MODE_INOUT, false));
    }

private:
    // methods
    void manualLoop(const float dt) {
        _strCmd = _hal->rc[ch_str]->getRadioPosition();
        _sailCmd = _hal->rc[ch_sail]->getRadioPosition();
    }
    void autoLoop(const float dt) {

        // insert tacking logic here
        
        // neglects heading command derivative
        float steering = pidStr.update(_guide->getHeadingError(), -_nav->getYawRate(), dt);
        _strCmd = steering;

        // insert sail command calculation based on sensor position here
        _sailCmd = 0; 
    }
    void setMotors() {
        _hal->rc[ch_str]->setPosition(_strCmd);
        _hal->rc[ch_sail]->setPosition(_sailCmd);
    }
    void handleFailsafe() {
        // turn off
        setMode(MAV_MODE_LOCKED);
    }

    // attributes
    enum {
        ch_mode = 0, ch_str, ch_sail
    };
    enum {
        k_chMode = k_radioChannelsStart, k_chStr, k_chSail
    };
    enum {
        k_pidStr = k_controllersStart
    };
    BlockPIDDfb pidStr;
    float _strCmd;
    float _sailCmd;
};

} // namespace apo

#endif /* CONTROLLERBOAT_H_ */
// vim:ts=4:sw=4:expandtab
