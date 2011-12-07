/*
 * ControllerPlane.h
 *
 *  Created on: Jun 30, 2011
 *      Author: jgoppert
 */

#ifndef CONTROLLERPLANE_H_
#define CONTROLLERPLANE_H_

#include "../APO/AP_Controller.h"

namespace apo {

class ControllerPlane: public AP_Controller {
public:
    ControllerPlane(AP_Navigator * nav, AP_Guide * guide,
                    AP_Board * board) :
        AP_Controller(nav, guide, board, new AP_ArmingMechanism(board,this,ch_thrust,ch_yaw,0.1,-0.9,0.9),ch_mode,k_cntrl),
        _trimGroup(k_trim, PSTR("trim_")),
        _rdrAilMix(&_group, 2, rdrAilMix, PSTR("rdrAilMix")),
        _needsTrim(false),
        _ailTrim(&_trimGroup, 1, ailTrim, PSTR("ail")),
        _elvTrim(&_trimGroup, 2, elvTrim, PSTR("elv")),
        _rdrTrim(&_trimGroup, 3, rdrTrim, PSTR("rdr")),
        _thrTrim(&_trimGroup, 4, thrTrim, PSTR("thr")),
        pidBnkRll(new AP_Var_group(k_pidBnkRll, PSTR("bnkRll_")), 1,
                  pidBnkRllP, pidBnkRllI, pidBnkRllD, pidBnkRllAwu,
                  pidBnkRllLim, pidBnkRllDFCut),
        pidPitPit(new AP_Var_group(k_pidPitPit, PSTR("pitPit_")), 1,
                  pidPitPitP, pidPitPitI, pidPitPitD, pidPitPitAwu,
                  pidPitPitLim, pidPitPitDFCut),
        pidSpdPit(new AP_Var_group(k_pidSpdPit, PSTR("spdPit_")), 1,
                  pidSpdPitP, pidSpdPitI, pidSpdPitD, pidSpdPitAwu,
                  pidSpdPitLim, pidSpdPitDFCut),
        pidYwrYaw(new AP_Var_group(k_pidYwrYaw, PSTR("ywrYaw_")), 1,
                  pidYwrYawP, pidYwrYawI, pidYwrYawD, pidYwrYawAwu,
                  pidYwrYawLim, pidYwrYawDFCut),
        pidHdgBnk(new AP_Var_group(k_pidHdgBnk, PSTR("hdgBnk_")), 1,
                  pidHdgBnkP, pidHdgBnkI, pidHdgBnkD, pidHdgBnkAwu,
                  pidHdgBnkLim, pidHdgBnkDFCut),
        pidAltThr(new AP_Var_group(k_pidAltThr, PSTR("altThr_")), 1,
                  pidAltThrP, pidAltThrI, pidAltThrD, pidAltThrAwu,
                  pidAltThrLim, pidAltThrDFCut),
        requireRadio(false), _aileron(0), _elevator(0), _rudder(0), _throttle(0) {

        _board->debug->println_P(PSTR("initializing plane controller"));

        _board->rc.push_back(
            new AP_RcChannel(k_chMode, PSTR("mode_"), board->radio, 5, 1100,
                             1500, 1900, RC_MODE_IN, false));
        _board->rc.push_back(
            new AP_RcChannel(k_chRoll, PSTR("roll_"), board->radio, 0, 1200,
                             1500, 1800, RC_MODE_INOUT, false));
        _board->rc.push_back(
            new AP_RcChannel(k_chPitch, PSTR("pitch_"), board->radio, 1, 1200,
                             1500, 1800, RC_MODE_INOUT, false));
        _board->rc.push_back(
            new AP_RcChannel(k_chThr, PSTR("thr_"), board->radio, 2, 1100, 1100,
                             1900, RC_MODE_INOUT, false));
        _board->rc.push_back(
            new AP_RcChannel(k_chYaw, PSTR("yaw_"), board->radio, 3, 1200, 1500,
                             1800, RC_MODE_INOUT, false));
    }

private:
    // methdos
    void manualLoop(const float dt) {
        setAllRadioChannelsManually();
        // force auto to read new manual trim
        if (_needsTrim == false)
            _needsTrim = true;
    }
    void autoLoop(const float dt) {
        _aileron = pidBnkRll.update(
                       pidHdgBnk.update(_guide->getHeadingError(), dt) - _nav->getRoll(), dt);
        _elevator = pidPitPit.update(
                        -pidSpdPit.update(
                            _guide->getAirSpeedCommand() - _nav->getAirSpeed(),
                            dt) - _nav->getPitch(), dt);
        _rudder = pidYwrYaw.update(-_nav->getYawRate(), dt);

        // desired yaw rate is zero, needs washout
        _throttle = pidAltThr.update(
                        _guide->getAltitudeCommand() - _nav->getAlt(), dt);

        if (_needsTrim) {
            // need to subtract current controller deflections so control
            // surfaces are actually at the same position as manual flight
            _ailTrim = _board->rc[ch_roll]->getRadioPosition() - _aileron;
            _elvTrim = _board->rc[ch_pitch]->getRadioPosition() - _elevator;
            _rdrTrim = _board->rc[ch_yaw]->getRadioPosition() - _rudder;
            _thrTrim = _board->rc[ch_thrust]->getRadioPosition() - _throttle;
            _needsTrim = false;
        }

        // actuator mixing/ output
        _aileron += _rdrAilMix * _rudder + _ailTrim;
        _elevator += _elvTrim;
        _rudder += _rdrTrim;
        _throttle += _thrTrim;
    }
    void setMotors() {
        // turn all motors off if below 0.1 throttle
        if (fabs(_board->rc[ch_thrust]->getRadioPosition()) < 0.1) {
            setAllRadioChannelsToNeutral();
        } else {
            _board->rc[ch_roll]->setPosition(_aileron);
            _board->rc[ch_yaw]->setPosition(_rudder);
            _board->rc[ch_pitch]->setPosition(_elevator);
            _board->rc[ch_thrust]->setPosition(_throttle);
        }
    }
    void handleFailsafe() {
        // note if testing and communication is lost the motors will not shut off,
        // it will attempt to land
        _guide->setMode(MAV_NAV_LANDING);
        setMode(MAV_MODE_AUTO);
    }

    // attributes
    enum {
        ch_mode = 0, ch_roll, ch_pitch, ch_thrust, ch_yaw
    };
    enum {
        k_chMode = k_radioChannelsStart,
        k_chRoll,
        k_chPitch,
        k_chYaw,
        k_chThr,

        k_pidBnkRll = k_controllersStart,
        k_pidSpdPit,
        k_pidPitPit,
        k_pidYwrYaw,
        k_pidHdgBnk,
        k_pidAltThr,

        k_trim = k_customStart
    };
    AP_Var_group _trimGroup;
    AP_Uint8 _rdrAilMix;
    bool _needsTrim;
    AP_Float _ailTrim;
    AP_Float _elvTrim;
    AP_Float _rdrTrim;
    AP_Float _thrTrim;
    BlockPID pidBnkRll; // bank error to roll servo deflection
    BlockPID pidSpdPit; // speed error to pitch command
    BlockPID pidPitPit; // pitch error to pitch servo deflection
    BlockPID pidYwrYaw; // yaw rate error to yaw servo deflection
    BlockPID pidHdgBnk; // heading error to bank command
    BlockPID pidAltThr; // altitude error to throttle deflection
    bool requireRadio;
    float _aileron;
    float _elevator;
    float _rudder;
    float _throttle;
};

} // namespace apo

#endif /* CONTROLLERPLANE_H_ */
// vim:ts=4:sw=4:expandtab
