/*
 * AP_Controller.cpp
 *
 *  Created on: Apr 30, 2011
 *      Author: jgoppert
 */

#include "../FastSerial/FastSerial.h"
#include "AP_ArmingMechanism.h"
#include "AP_BatteryMonitor.h"
#include "AP_Board.h"
#include "AP_RcChannel.h"
#include "../GCS_MAVLink/include/mavlink_types.h"
#include "constants.h"
#include "AP_CommLink.h"
#include "AP_Controller.h"

namespace apo {

AP_Controller::AP_Controller(AP_Navigator * nav, AP_Guide * guide,
                             AP_Board * board, AP_ArmingMechanism * armingMechanism,
                             const uint8_t chMode, const uint16_t key, const prog_char_t * name) :
    _nav(nav), _guide(guide), _board(board), _armingMechanism(armingMechanism),
    _group(key, name ? : PSTR("CNTRL_")),
    _chMode(chMode),
    _mode(MAV_MODE_LOCKED), _state(MAV_STATE_BOOT) {
    setAllRadioChannelsToNeutral();
}

void AP_Controller::setAllRadioChannelsToNeutral() {
    for (uint8_t i = 0; i < _board->rc.getSize(); i++) {
        _board->rc[i]->setPosition(0.0);
    }
}

void AP_Controller::setAllRadioChannelsManually() {
    //_board->debug->printf_P(PSTR("\tsize: %d\n"),_board->rc.getSize());
    for (uint8_t i = 0; i < _board->rc.getSize(); i++) {
        _board->rc[i]->setUsingRadio();
        //_board->debug->printf_P(PSTR("\trc %d: %f\n"),i,_board->rc[i]->getPosition());
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
            _board->gcs->sendText(SEVERITY_HIGH, PSTR("disarmed"));
        }
    }
    // if not locked
    else {

        // if state is not active, set it to active and alert gcs
        if (getState()!=MAV_STATE_ACTIVE) {
            setState(MAV_STATE_ACTIVE);
            _board->gcs->sendText(SEVERITY_HIGH, PSTR("armed"));
        }

        // handle emergencies
        //
        // check for heartbeat from gcs, if not found go to failsafe
        if (_board->gcs->heartBeatLost()) {
            setMode(MAV_MODE_FAILSAFE);
            _board->gcs->sendText(SEVERITY_HIGH, PSTR("configure gcs to send heartbeat"));
            
        // if battery less than 5%, go to failsafe
        } else if (_board->batteryMonitor && _board->batteryMonitor->getPercentage() < 5) {
            setMode(MAV_MODE_FAILSAFE);
            _board->gcs->sendText(SEVERITY_HIGH, PSTR("recharge battery"));
        }

        // if in auto mode and manual switch set, change to manual
        if (_board->rc[_chMode]->getRadioPosition() > 0) setMode(MAV_MODE_MANUAL);
        else setMode(MAV_MODE_AUTO);

        // handle all possible modes
        if (getMode()==MAV_MODE_MANUAL) {
            manualLoop(dt);
        } else if (getMode()==MAV_MODE_AUTO) {
            autoLoop(dt);
        } else if (getMode()==MAV_MODE_FAILSAFE) {
            handleFailsafe();
        } else {
            _board->gcs->sendText(SEVERITY_HIGH, PSTR("unknown mode"));
            setMode(MAV_MODE_FAILSAFE);
        }
    }

    // this sends commands to motors
    if(getState()==MAV_STATE_ACTIVE) {
        digitalWrite(_board->aLedPin, HIGH);
        setMotors();
    } else {
        digitalWrite(_board->aLedPin, LOW);
        setAllRadioChannelsToNeutral();
    }
}

} // namespace apo
// vim:ts=4:sw=4:expandtab
