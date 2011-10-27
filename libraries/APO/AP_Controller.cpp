/*
 * AP_Controller.cpp
 *
 *  Created on: Apr 30, 2011
 *      Author: jgoppert
 */

#include "AP_Controller.h"
#include "../FastSerial/FastSerial.h"
#include "AP_ArmingMechanism.h"
#include "AP_BatteryMonitor.h"
#include "AP_HardwareAbstractionLayer.h"
#include "../AP_Common/include/menu.h"
#include "AP_RcChannel.h"
#include "../GCS_MAVLink/include/mavlink_types.h"
#include "constants.h"
#include "AP_CommLink.h"

namespace apo {

AP_Controller::AP_Controller(AP_Navigator * nav, AP_Guide * guide,
                             AP_HardwareAbstractionLayer * hal, AP_ArmingMechanism * armingMechanism,
                             const uint8_t chMode, const uint16_t key, const prog_char_t * name) :
    _nav(nav), _guide(guide), _hal(hal), _armingMechanism(armingMechanism),
    _group(key, name ? : PSTR("CNTRL_")),
    _chMode(chMode),
    _mode(&_group, 1, MAV_MODE_LOCKED, PSTR("MODE")) {
    setAllRadioChannelsToNeutral();
}

void AP_Controller::setAllRadioChannelsToNeutral() {
    for (uint8_t i = 0; i < _hal->rc.getSize(); i++) {
        _hal->rc[i]->setPosition(0.0);
    }
}

void AP_Controller::setAllRadioChannelsManually() {
    //_hal->debug->printf_P(PSTR("\tsize: %d\n"),_hal->rc.getSize());
    for (uint8_t i = 0; i < _hal->rc.getSize(); i++) {
        _hal->rc[i]->setUsingRadio();
        //_hal->debug->printf_P(PSTR("\trc %d: %f\n"),i,_hal->rc[i]->getPosition());
    }
}

void AP_Controller::update(const float dt) {
    if (_armingMechanism) _armingMechanism->update(dt);

    // determine flight mode
    //
    // check for heartbeat from gcs, if not found go to failsafe
    if (_hal->heartBeatLost()) {
        _mode = MAV_MODE_FAILSAFE;
        _hal->gcs->sendText(SEVERITY_HIGH, PSTR("configure gcs to send heartbeat"));
        // if battery less than 5%, go to failsafe
    } else if (_hal->batteryMonitor && _hal->batteryMonitor->getPercentage() < 5) {
        _mode = MAV_MODE_FAILSAFE;
        _hal->gcs->sendText(SEVERITY_HIGH, PSTR("recharge battery"));
        // manual/auto switch
    } else {
        // if all emergencies cleared, fall back to standby
        if (_hal->getState()==MAV_STATE_EMERGENCY) _hal->setState(MAV_STATE_STANDBY);
        if (_hal->rc[_chMode]->getRadioPosition() > 0) _mode = MAV_MODE_MANUAL;
        else _mode = MAV_MODE_AUTO;
    }

    // handle flight modes
    switch(_mode) {

    case MAV_MODE_LOCKED: {
        _hal->setState(MAV_STATE_STANDBY);
        break;
    }

    case MAV_MODE_FAILSAFE: {
        _hal->setState(MAV_STATE_EMERGENCY);
        break;
    }

    case MAV_MODE_MANUAL: {
        manualLoop(dt);
        break;
    }

    case MAV_MODE_AUTO: {
        autoLoop(dt);
        break;
    }

    default: {
        _hal->gcs->sendText(SEVERITY_HIGH, PSTR("unknown mode"));
        _hal->setState(MAV_STATE_EMERGENCY);
    }
    }

    // this sends commands to motors
    setMotors();
}

void AP_Controller::setMotors() {
    switch (_hal->getState()) {

    case MAV_STATE_ACTIVE: {
        digitalWrite(_hal->aLedPin, HIGH);
        setMotorsActive();
        break;
    }
    case MAV_STATE_EMERGENCY: {
        digitalWrite(_hal->aLedPin, LOW);
        setMotorsEmergency();
        break;
    }
    case MAV_STATE_STANDBY: {
        digitalWrite(_hal->aLedPin,LOW);
        setMotorsStandby();
        break;
    }
    default: {
        setMotorsStandby();
    }

    }
}


} // namespace apo
// vim:ts=4:sw=4:expandtab
