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
                  AP_Board * board) :
        AP_Controller(nav, guide, board,new AP_ArmingMechanism(board,this,ch_thrust,ch_str,0.1,-0.9,0.9), ch_mode, k_cntrl),
        pidStr(new AP_Var_group(k_pidStr, PSTR("STR_")), 1, steeringP,
               steeringI, steeringD, steeringIMax, steeringYMax),
        pidThrust(new AP_Var_group(k_pidThrust, PSTR("THR_")), 1, throttleP,
                  throttleI, throttleD, throttleIMax, throttleYMax,
                  throttleDFCut), _strCmd(0), _thrustCmd(0),
      _rangeFinderFront()
    {
        _board->debug->println_P(PSTR("initializing boat controller"));

        _board->rc.push_back(
            new AP_RcChannel(k_chMode, PSTR("MODE_"), board->radio, 5, 1100,
                             1500, 1900, RC_MODE_IN, false));
        _board->rc.push_back(
            new AP_RcChannel(k_chStr, PSTR("STR_"), board->radio, 3, 1100, 1500,
                             1900, RC_MODE_INOUT, false));
        _board->rc.push_back(
            new AP_RcChannel(k_chThrust, PSTR("THR_"), board->radio, 2, 1100, 1500,
                             1900, RC_MODE_INOUT, false));
        _board->rc.push_back(
            new AP_RcChannel(k_chFwdRev, PSTR("FWDREV_"), board->radio, 4, 1100, 1500,
                             1900, RC_MODE_IN, false));

        for (uint8_t i = 0; i < _board->rangeFinders.getSize(); i++) {
            RangeFinder * rF = _board->rangeFinders[i];
            if (rF == NULL)
                continue;
            if (rF->orientation_x == 1 && rF->orientation_y == 0
                    && rF->orientation_z == 0)
                _rangeFinderFront = rF;
        }
    }

private:
    // methods
    void manualLoop(const float dt) {
        _strCmd = _board->rc[ch_str]->getRadioPosition();
        _thrustCmd = _board->rc[ch_thrust]->getRadioPosition();
        if (useForwardReverseSwitch && _board->rc[ch_FwdRev]->getRadioPosition() < 0.0)
            _thrustCmd = -_thrustCmd;
    }
    void autoLoop(const float dt) {
        //_board->debug->printf_P(PSTR("cont: ch1: %f\tch2: %f\n"),_board->rc[ch_thrust]->getRadioPosition(), _board->rc[ch_str]->getRadioPosition());
        // neglects heading command derivative
        float steering = pidStr.update(_guide->getHeadingError(), -_nav->getYawRate(), dt);
        float thrust = pidThrust.update(
                         _guide->getGroundSpeedCommand()
                         - _nav->getGroundSpeed(), dt);

        // obstacle avoidance overrides
        // try to drive around the obstacle in front. if this fails, stop
        if (_rangeFinderFront) {
            _rangeFinderFront->read();

            int distanceToObstacle = _rangeFinderFront->distance;
            if (distanceToObstacle < 100) {
                thrust = 0;   // Avoidance didn't work out. Stopping
            }
            else if (distanceToObstacle < 650) {
                // Deviating from the course. Deviation angle is inverse proportional
                // to the distance to the obstacle, with 15 degrees min angle, 180 - max
                steering += (15 + 165 *
                    (1 - ((float)(distanceToObstacle - 100)) / 550)) * deg2Rad;
            }
        }

        _strCmd = steering;
        _thrustCmd = thrust;
    }
    void setMotors() {
        _board->rc[ch_str]->setPosition(_strCmd);
        _board->rc[ch_thrust]->setPosition(fabs(_thrustCmd) < 0.1 ? 0 : _thrustCmd);
    }
    void handleFailsafe() {
        // turn off
        setMode(MAV_MODE_LOCKED);
    }

    // attributes
    enum {
        ch_mode = 0, ch_str, ch_thrust, ch_FwdRev
    };
    enum {
        k_chMode = k_radioChannelsStart, k_chStr, k_chThrust, k_chFwdRev
    };
    enum {
        k_pidStr = k_controllersStart, k_pidThrust
    };
    BlockPIDDfb pidStr;
    BlockPID pidThrust;
    float _strCmd, _thrustCmd;

    RangeFinder * _rangeFinderFront;
};

} // namespace apo

#endif /* CONTROLLERBOAT_H_ */
// vim:ts=4:sw=4:expandtab
