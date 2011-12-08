/*
 * ControllerSailboat.h
 *
 *  Created on: Jun 30, 2011
 *      Author: jgoppert
 */

#ifndef CONTROLLERSAILBOAT_H_
#define CONTROLLERSAILBOAT_H_

#include "../APO/AP_Controller.h"

namespace apo {

class ControllerSailboat: public AP_Controller {
public:
    ControllerSailboat(AP_Navigator * nav, AP_Guide * guide,
                  AP_Board * board) :
        AP_Controller(nav, guide, board,new AP_ArmingMechanism(board,this,ch_sail,ch_str,0.1,-0.9,0.9), ch_mode, k_cntrl),
        pidStr(new AP_Var_group(k_pidStr, PSTR("STR_")), 1, steeringP,
               steeringI, steeringD, steeringIMax, steeringYMax),
        pidSail(new AP_Var_group(k_pidSail, PSTR("SAIL_")), 1, throttleP,
                  throttleI, throttleD, throttleIMax, throttleYMax,
                  throttleDFCut), _strCmd(0), _sailCmd(0)
    {
        _board->debug->println_P(PSTR("initializing sailboat controller"));

        _board->rc.push_back(
            new AP_RcChannel(k_chMode, PSTR("MODE_"), board->radio, 5, 1100,
                             1500, 1900, RC_MODE_IN, false));
        _board->rc.push_back(
            new AP_RcChannel(k_chStr, PSTR("STR_"), board->radio, 3, 1100, 1500,
                             1900, RC_MODE_INOUT, false));
        _board->rc.push_back(
            new AP_RcChannel(k_chSail, PSTR("SAIL_"), board->radio, 2, 1100, 1500,
                             1900, RC_MODE_INOUT, false));
    }

private:
    // methods
    void manualLoop(const float dt) {
        _strCmd = -_board->rc[ch_str]->getRadioPosition();
        _sailCmd = _board->rc[ch_sail]->getRadioPosition();
        _board->debug->printf_P(PSTR("sail: %f, steering: %f\n"),_sailCmd,_strCmd);
    }
    void autoLoop(const float dt) {
        //_board->debug->printf_P(PSTR("cont: ch1: %f\tch2: %f\n"),_board->rc[ch_sail]->getRadioPosition(), _board->rc[ch_str]->getRadioPosition());
        float windDir = -.339373*analogRead(1)+175.999;       


        // neglects heading command derivative
        float steering = -pidStr.update(_guide->getHeadingError(), -_nav->getYawRate(), dt);
        float sail = 0.00587302*fabs(windDir) - 0.05;
        if (sail < 0.0) sail = 0.0;

        //_board->debug->printf_P(PSTR("heading: %f\n"),heading);       //Print Heading 
     
        //if(fabs(psi)<45)                                    //Tacking Logic
        //{
            //if(psi<-10)
                //alpha = -45;
            //else if(psi>10)
                //alpha = 45;
            //else
            //{
                //if(psi==10)
                    //alpha = 45;
                //else if(psi==-10)
                    //alpha = -45;
                //else 
                    //alpha = alpha;
            //}
        //}*/
        _strCmd = steering;
        _sailCmd = sail;
    }
    void setMotors() {
        _board->rc[ch_str]->setPosition(_strCmd);
        _board->rc[ch_sail]->setPosition(_sailCmd);
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
        k_pidStr = k_controllersStart, k_pidSail
    };
    BlockPIDDfb pidStr;
    BlockPID pidSail;
    float _strCmd, _sailCmd;
};

} // namespace apo

#endif /* CONTROLLERSAILBOAT_H_ */
// vim:ts=4:sw=4:expandtab
