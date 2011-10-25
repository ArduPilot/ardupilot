/*
 * AP_ArmingMechanism.cpp
 *
 */


#include "AP_ArmingMechanism.h"
#include "AP_RcChannel.h"
#include "../FastSerial/FastSerial.h"
#include "AP_HardwareAbstractionLayer.h"
#include "AP_CommLink.h"

namespace apo {

void AP_ArmingMechanism::update(const float dt) {

    // arming
    if ( (_hal->getState() != MAV_STATE_ACTIVE) &&
     (fabs(_hal->rc[_ch1]->getRadioPosition()) < _ch1Min) &&
     (_hal->rc[_ch2]->getRadioPosition() < _ch2Min) ) {

        // always start clock at 0
        if (_armingClock<0) _armingClock = 0;

        if (_armingClock++ >= 100) {
            _hal->gcs->sendText(SEVERITY_HIGH, PSTR("armed"));
            _hal->setState(MAV_STATE_ACTIVE);
        } else {
            _hal->gcs->sendText(SEVERITY_HIGH, PSTR("arming"));
        }
    }
    // disarming
    else if ( (_hal->getState() == MAV_STATE_ACTIVE) &&
         (fabs(_hal->rc[_ch1]->getRadioPosition()) < _ch1Min) &&
         (_hal->rc[_ch2]->getRadioPosition() > _ch2Max) ) {

        // always start clock at 0
        if (_armingClock>0) _armingClock = 0;

        if (_armingClock-- <= -100) {
            _hal->gcs->sendText(SEVERITY_HIGH, PSTR("disarmed"));
            _hal->setState(MAV_STATE_STANDBY);
        } else {
            _hal->gcs->sendText(SEVERITY_HIGH, PSTR("disarming"));
        }
    }
    // reset arming clock and report status
    else if (_armingClock != 0) {
        _armingClock = 0;
        if (_hal->getState()==MAV_STATE_ACTIVE) _hal->gcs->sendText(SEVERITY_HIGH, PSTR("armed"));
        else if (_hal->getState()!=MAV_STATE_ACTIVE) _hal->gcs->sendText(SEVERITY_HIGH, PSTR("disarmed"));
    }
}

} // apo

// vim:ts=4:sw=4:expandtab
