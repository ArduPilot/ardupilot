/*
 * AP_Guide.cpp
 *
 *  Created on: Apr 30, 2011
 *      Author: jgoppert
 */

#include "AP_Guide.h"
#include "../FastSerial/FastSerial.h"
#include "AP_Navigator.h"
#include "constants.h"
#include "AP_Board.h"
#include "AP_CommLink.h"

namespace apo {

AP_Guide::AP_Guide(AP_Navigator * nav, AP_Board * board) :
    _nav(nav), _board(board), _command(AP_MavlinkCommand::home),
    _previousCommand(AP_MavlinkCommand::home),
    _headingCommand(0), _airSpeedCommand(0),
    _groundSpeedCommand(0), _altitudeCommand(0),
    _mode(MAV_NAV_LOST),
    _numberOfCommands(1), _cmdIndex(0), _nextCommandCalls(0),
    _nextCommandTimer(0) {
}

void AP_Guide::setCurrentIndex(uint8_t val) {
    _cmdIndex.set_and_save(val);
    _command = AP_MavlinkCommand(getCurrentIndex());
    _previousCommand = AP_MavlinkCommand(getPreviousIndex());
    _board->gcs->sendMessage(MAVLINK_MSG_ID_WAYPOINT_CURRENT);
    updateCommand();
}

float AP_Guide::getHeadingError() {
    return wrapAngle(getHeadingCommand()
                         - _nav->getYaw());
}

float AP_Guide::getDistanceToNextWaypoint() {
    return _command.distanceTo(_nav->getLat_degInt(),
        _nav->getLon_degInt());
}

float AP_Guide::getGroundSpeedError() {
    return _groundSpeedCommand - _nav->getGroundSpeed();
}

MavlinkGuide::MavlinkGuide(AP_Navigator * nav,
                           AP_Board * board, float velCmd, float xt, float xtLim) :
    AP_Guide(nav, board),
    _group(k_guide, PSTR("guide_")),
    _velocityCommand(&_group, 1, velCmd, PSTR("velCmd")),
    _crossTrackGain(&_group, 2, xt, PSTR("xt")),
    _crossTrackLim(&_group, 3, xtLim, PSTR("xtLim")) {
}

float AP_Guide::getYawError(){
    return wrapAngle(_yawCommand - _nav->getYaw());
}

void MavlinkGuide::update() {
    // process mavlink commands
    handleCommand();
}

float MavlinkGuide::getPNError() {
    return -_command.getPN(_nav->getLat_degInt(), _nav->getLon_degInt());
}
float MavlinkGuide::getPEError() {
    return -_command.getPE(_nav->getLat_degInt(), _nav->getLon_degInt());
}
float MavlinkGuide::getPDError() {
    return -_command.getPD(_nav->getAlt_intM());
}

void MavlinkGuide::nextCommand() {
    // within 1 seconds, check if more than 5 calls to next command occur
    // if they do, go to home waypoint
    if (millis() - _nextCommandTimer < 1000) {
        if (_nextCommandCalls > 5) {
            Serial.println("commands loading too fast, returning home");
            setCurrentIndex(0);
            setNumberOfCommands(1);
            _nextCommandCalls = 0;
            _nextCommandTimer = millis();
            return;
        }
        _nextCommandCalls++;
    } else {
        _nextCommandTimer = millis();
        _nextCommandCalls = 0;
    }

    // set the current command
    setCurrentIndex(getNextIndex());
}

void MavlinkGuide::updateCommand() {
    // update guidance mode
    if (_command.getCommand() == MAV_CMD_NAV_WAYPOINT) {
        _mode = MAV_NAV_WAYPOINT;
    } else if (_command.getCommand() == MAV_CMD_NAV_LAND) {
        _mode = MAV_NAV_LANDING;
    } else if (_command.getCommand() == MAV_CMD_NAV_LOITER_TIME) {
        _mode = MAV_NAV_LOITER;
    } else if (_command.getCommand() == MAV_CMD_NAV_LOITER_UNLIM) {
        _mode = MAV_NAV_LOITER;
    } else if (_command.getCommand() == MAV_CMD_NAV_LOITER_TURNS) {
        _mode = MAV_NAV_LOITER;
    } else if (_command.getCommand() == MAV_CMD_NAV_RETURN_TO_LAUNCH) {
        _mode = MAV_NAV_RETURNING;
    } else if (_command.getCommand() == MAV_CMD_NAV_TAKEOFF) {
        _mode = MAV_NAV_LIFTOFF;
    } else {
        _board->debug->printf_P(PSTR("unhandled command"));
        _board->gcs->sendText(SEVERITY_HIGH,PSTR("unhandled command"));
        nextCommand();
        return;
    }

    // TODO handle more commands
    //MAV_CMD_CONDITION_CHANGE_ALT
    //MAV_CMD_CONDITION_DELAY
    //MAV_CMD_CONDITION_DISTANCE
    //MAV_CMD_CONDITION_LAST
    //MAV_CMD_CONDITION_YAW

    //MAV_CMD_DO_CHANGE_SPEED
    //MAV_CMD_DO_CONTROL_VIDEO
    //MAV_CMD_DO_JUMP
    //MAV_CMD_DO_LAST
    //MAV_CMD_DO_LAST
    //MAV_CMD_DO_REPEAT_RELAY
    //MAV_CMD_DO_REPEAT_SERVO
    //MAV_CMD_DO_SET_HOME
    //MAV_CMD_DO_SET_MODE
    //MAV_CMD_DO_SET_PARAMETER
    //MAV_CMD_DO_SET_RELAY
    //MAV_CMD_DO_SET_SERVO

    //MAV_CMD_PREFLIGHT_CALIBRATION
    //MAV_CMD_PREFLIGHT_STORAGE
}

void MavlinkGuide::handleCommand() {

    // for these modes use crosstrack navigation
    if (
        _mode == MAV_NAV_WAYPOINT ||
        _mode == MAV_NAV_LANDING ||
        _mode == MAV_NAV_LIFTOFF ||
        _mode == MAV_NAV_VECTOR) {

        // if we don't have enough waypoint for cross track calcs
        // switch to loiter mode
        if (_numberOfCommands == 1) {
            _mode = MAV_NAV_LOITER;
            return;
        }

        // check if we are at waypoint or if command
        // radius is zero within tolerance
        if ( (getDistanceToNextWaypoint() < _command.getRadius()) | (getDistanceToNextWaypoint() < 1e-5) ) {
            _board->gcs->sendText(SEVERITY_LOW,PSTR("waypoint reached (distance)"));
            _board->debug->printf_P(PSTR("waypoint reached (distance)"));
            nextCommand();
            return;
        }

        // check for along track next waypoint requirement
        float alongTrack = _command.alongTrack(_previousCommand,
                                               _nav->getLat_degInt(),
                                               _nav->getLon_degInt());
        float segmentLength = _previousCommand.distanceTo(_command);
        if (alongTrack > segmentLength) {
            _board->gcs->sendText(SEVERITY_LOW,PSTR("waypoint reached (along track)"));
            _board->debug->printf_P(PSTR("waypoint reached (along track) segmentLength: %f\t alongTrack: %f\n"),segmentLength,alongTrack);
            nextCommand();
            return;
        }

        // calculate altitude and heading commands
        _altitudeCommand = _command.getAlt();
        float dXt = _command.crossTrack(_previousCommand,
                                        _nav->getLat_degInt(),
                                        _nav->getLon_degInt());
        float temp = dXt * _crossTrackGain * deg2Rad; // crosstrack gain, rad/m
        if (temp > _crossTrackLim * deg2Rad)
            temp = _crossTrackLim * deg2Rad;
        if (temp < -_crossTrackLim * deg2Rad)
            temp = -_crossTrackLim * deg2Rad;
        float bearing = _previousCommand.bearingTo(_command);
        _headingCommand = bearing - temp;
        _yawCommand = _command.getYawCommand();
        _board->debug->printf_P(
        	PSTR("nav: bCurrent2Dest: %f\tdXt: %f\tcmdHeading: %f\tnextWpDistance: %f\talongTrack: %f\tyaw command: %f\n"),
        	bearing * rad2Deg, dXt, _headingCommand * rad2Deg, getDistanceToNextWaypoint(), alongTrack, _yawCommand*rad2Deg);

        // for these modes just head to current command
    } else if (
        _mode == MAV_NAV_LOITER ||
        _mode == MAV_NAV_RETURNING) {
        _altitudeCommand = AP_MavlinkCommand::home.getAlt();
        _headingCommand = AP_MavlinkCommand::home.bearingTo(
                              _nav->getLat_degInt(), _nav->getLon_degInt())
                          + 180 * deg2Rad;
        if (_headingCommand > 360 * deg2Rad)
            _headingCommand -= 360 * deg2Rad;

        // do nothing for these modes
    } else if (
        _mode == MAV_NAV_GROUNDED ||
        _mode == MAV_NAV_HOLD  ||
        _mode == MAV_NAV_LOST) {

    }

    // if in unhandled mode, then return
    else {
        _board->debug->printf_P(PSTR("unhandled guide mode"));
        _board->gcs->sendText(SEVERITY_HIGH,PSTR("unhandled guide mode"));
        _mode = MAV_NAV_RETURNING;
    }

    _groundSpeedCommand = _velocityCommand;

    // debug
    //_board->debug->printf_P(
        //PSTR("guide loop, number: %d, current index: %d, previous index: %d\n"),
        //getNumberOfCommands(), getCurrentIndex(), getPreviousIndex());
}

} // namespace apo

// vim:ts=4:sw=4:expandtab
