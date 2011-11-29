/*
 * AP_Controller.cpp
 *
 *  Created on: Apr 30, 2011
 *      Author: jgoppert
 */

#include "../FastSerial/FastSerial.h"
#include "AP_ArmingMechanism.h"
#include "AP_BatteryMonitor.h"
#include "AP_HardwareAbstractionLayer.h"
#include "AP_RcChannel.h"
#include "../GCS_MAVLink/include/mavlink_types.h"
#include "constants.h"
#include "AP_CommLink.h"
#include "AP_Controller.h"

namespace apo {

AP_Controller::AP_Controller(AP_Navigator * nav, AP_Guide * guide,
                             AP_HardwareAbstractionLayer * hal, AP_ArmingMechanism * armingMechanism,
                             const uint8_t chMode, const uint16_t key, const prog_char_t * name) :
    _nav(nav), _guide(guide), _hal(hal), _armingMechanism(armingMechanism),
    _group(key, name ? : PSTR("CNTRL_")),
    _chMode(chMode),
    _mode(MAV_MODE_LOCKED), _state(MAV_STATE_BOOT) {
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
    // handle modes
    //
    // if in locked mode
    if (getMode() == MAV_MODE_LOCKED) {
        // if state is not stanby then set it to standby and alert gcs
        if (getState()!=MAV_STATE_STANDBY) {
            setState(MAV_STATE_STANDBY);
            _hal->gcs->sendText(SEVERITY_HIGH, PSTR("disarmed"));
        }
    }
    // if not locked
    else {

        // if state is not active, set it to active and alert gcs
        if (getState()!=MAV_STATE_ACTIVE) {
            setState(MAV_STATE_ACTIVE);
            _hal->gcs->sendText(SEVERITY_HIGH, PSTR("armed"));
        }

        // handle emergencies
        //
        // check for heartbeat from gcs, if not found go to failsafe
        if (_hal->gcs->heartBeatLost()) {
            setMode(MAV_MODE_FAILSAFE);
            _hal->gcs->sendText(SEVERITY_HIGH, PSTR("configure gcs to send heartbeat"));
            
        // if battery less than 5%, go to failsafe
        } else if (_hal->batteryMonitor && _hal->batteryMonitor->getPercentage() < 5) {
            setMode(MAV_MODE_FAILSAFE);
            _hal->gcs->sendText(SEVERITY_HIGH, PSTR("recharge battery"));
        }

        // if in auto mode and manual switch set, change to manual
        if (_hal->rc[_chMode]->getRadioPosition() > 0) setMode(MAV_MODE_MANUAL);
        else setMode(MAV_MODE_AUTO);

        // handle all possible modes
        if (getMode()==MAV_MODE_MANUAL) {
            manualLoop(dt);
        } else if (getMode()==MAV_MODE_AUTO) {
            autoLoop(dt);
        } else if (getMode()==MAV_MODE_FAILSAFE) {
            handleFailsafe();
        } else {
            _hal->gcs->sendText(SEVERITY_HIGH, PSTR("unknown mode"));
            setMode(MAV_MODE_FAILSAFE);
        }
    }

    // this sends commands to motors
    if(getState()==MAV_STATE_ACTIVE) {
        digitalWrite(_hal->aLedPin, HIGH);
        setMotors();
    } else {
        digitalWrite(_hal->aLedPin, LOW);
        setAllRadioChannelsToNeutral();
    }
}

} // namespace apo
// vim:ts=4:sw=4:expandtab
