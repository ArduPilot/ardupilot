/*
 * AP_ArmingMechanism.cpp
 *
 */

#include "AP_ArmingMechanism.h"
#include "AP_Controller.h"
#include "AP_RcChannel.h"
#include "../FastSerial/FastSerial.h"
#include "AP_Board.h"
#include "AP_CommLink.h"

namespace apo {

void AP_ArmingMechanism::update(const float dt) {
    //_board->debug->printf_P(PSTR("ch1: %f\tch2: %f\n"),_board->rc[_ch1]->getRadioPosition(), _board->rc[_ch2]->getRadioPosition());
    // arming
    if ( (_controller->getState() != MAV_STATE_ACTIVE) &&
            (fabs(_board->rc[_ch1]->getRadioPosition()) < _ch1Min) &&
            (_board->rc[_ch2]->getRadioPosition() < _ch2Min) ) {

        // always start clock at 0
        if (_armingClock<0) _armingClock = 0;

        if (_armingClock++ >= 100) {
            _controller->setMode(MAV_MODE_READY);
        } else {
            _board->gcs->sendText(SEVERITY_HIGH, PSTR("arming"));
        }
    }
    // disarming
    else if ( (_controller->getState() == MAV_STATE_ACTIVE) &&
              (fabs(_board->rc[_ch1]->getRadioPosition()) < _ch1Min) &&
              (_board->rc[_ch2]->getRadioPosition() > _ch2Max) ) {

        // always start clock at 0
        if (_armingClock>0) _armingClock = 0;

        if (_armingClock-- <= -100) {
            _controller->setMode(MAV_MODE_LOCKED);
        } else {
            _board->gcs->sendText(SEVERITY_HIGH, PSTR("disarming"));
        }
    }
    // reset arming clock and report status
    else if (_armingClock != 0) {
        _armingClock = 0;
        if (_controller->getState()==MAV_STATE_ACTIVE) _board->gcs->sendText(SEVERITY_HIGH, PSTR("armed"));
        else if (_controller->getState()!=MAV_STATE_ACTIVE) _board->gcs->sendText(SEVERITY_HIGH, PSTR("disarmed"));
    }
}

} // apo

// vim:ts=4:sw=4:expandtab
