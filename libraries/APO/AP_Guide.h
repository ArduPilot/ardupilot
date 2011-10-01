/*
 * AP_Guide.h
 * Copyright (C) James Goppert 2010 <james.goppert@gmail.com>
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef AP_Guide_H
#define AP_Guide_H

#include "../GCS_MAVLink/GCS_MAVLink.h"
#include "AP_HardwareAbstractionLayer.h"
#include "AP_Navigator.h"
#include "../AP_Common/AP_Common.h"
#include "../AP_Common/AP_Vector.h"
#include "AP_MavlinkCommand.h"
#include "constants.h"

//#include "AP_CommLink.h"

namespace apo {

/// Guide class
class AP_Guide {
public:

	/**
	 * This is the constructor, which requires a link to the navigator.
	 * @param navigator This is the navigator pointer.
	 */
	AP_Guide(AP_Navigator * navigator, AP_HardwareAbstractionLayer * hal) :
		_navigator(navigator), _hal(hal), _command(0), _previousCommand(0),
				_headingCommand(0), _airSpeedCommand(0),
				_groundSpeedCommand(0), _altitudeCommand(0), _pNCmd(0),
				_pECmd(0), _pDCmd(0), _mode(MAV_NAV_LOST),
				_numberOfCommands(1), _cmdIndex(0), _nextCommandCalls(0),
				_nextCommandTimer(0) {
	}

	virtual void update() = 0;

	virtual void nextCommand() = 0;

	MAV_NAV getMode() const {
		return _mode;
	}
	uint8_t getCurrentIndex() {
		return _cmdIndex;
	}

	void setCurrentIndex(uint8_t val) {
		_cmdIndex.set_and_save(val);
		_command = AP_MavlinkCommand(getCurrentIndex());
		_command.load();
		_previousCommand = AP_MavlinkCommand(getPreviousIndex());
		_previousCommand.load();
		//_hal->gcs->sendMessage(MAVLINK_MSG_ID_WAYPOINT_CURRENT);
	}

	uint8_t getNumberOfCommands() {
		return _numberOfCommands;
	}

	void setNumberOfCommands(uint8_t val) {
		_numberOfCommands.set_and_save(val);
	}

	uint8_t getPreviousIndex() {
		// find previous waypoint, TODO, handle non-nav commands
		int16_t prevIndex = int16_t(getCurrentIndex()) - 1;
		if (prevIndex < 0)
			prevIndex = getNumberOfCommands() - 1;
		return (uint8_t) prevIndex;
	}

	uint8_t getNextIndex() {
		// find previous waypoint, TODO, handle non-nav commands
		int16_t nextIndex = int16_t(getCurrentIndex()) + 1;
		if (nextIndex > (getNumberOfCommands() - 1))
			nextIndex = 0;
		return nextIndex;
	}

	float getHeadingCommand() {
		return _headingCommand;
	}
	float getAirSpeedCommand() {
		return _airSpeedCommand;
	}
	float getGroundSpeedCommand() {
		return _groundSpeedCommand;
	}
	float getAltitudeCommand() {
		return _altitudeCommand;
	}
	float getPNCmd() {
		return _pNCmd;
	}
	float getPECmd() {
		return _pECmd;
	}
	float getPDCmd() {
		return _pDCmd;
	}
	MAV_NAV getMode() {
		return _mode;
	}
	uint8_t getCommandIndex() {
		return _cmdIndex;
	}

protected:
	AP_Navigator * _navigator;
	AP_HardwareAbstractionLayer * _hal;
	AP_MavlinkCommand _command, _previousCommand;
	float _headingCommand;
	float _airSpeedCommand;
	float _groundSpeedCommand;
	float _altitudeCommand;
	float _pNCmd;
	float _pECmd;
	float _pDCmd;
	MAV_NAV _mode;
	AP_Uint8 _numberOfCommands;
	AP_Uint8 _cmdIndex;
	uint16_t _nextCommandCalls;
	uint16_t _nextCommandTimer;
};

class MavlinkGuide: public AP_Guide {
public:
	MavlinkGuide(AP_Navigator * navigator,
			AP_HardwareAbstractionLayer * hal) :
		AP_Guide(navigator, hal), _rangeFinderFront(), _rangeFinderBack(),
				_rangeFinderLeft(), _rangeFinderRight(),
				_group(k_guide, PSTR("guide_")),
				_velocityCommand(&_group, 1, 1, PSTR("velCmd")),
				_crossTrackGain(&_group, 2, 2, PSTR("xt")),
				_crossTrackLim(&_group, 3, 10, PSTR("xtLim")) {

		for (uint8_t i = 0; i < _hal->rangeFinders.getSize(); i++) {
			RangeFinder * rF = _hal->rangeFinders[i];
			if (rF == NULL)
				continue;
			if (rF->orientation_x == 1 && rF->orientation_y == 0
					&& rF->orientation_z == 0)
				_rangeFinderFront = rF;
			else if (rF->orientation_x == -1 && rF->orientation_y == 0
					&& rF->orientation_z == 0)
				_rangeFinderBack = rF;
			else if (rF->orientation_x == 0 && rF->orientation_y == 1
					&& rF->orientation_z == 0)
				_rangeFinderRight = rF;
			else if (rF->orientation_x == 0 && rF->orientation_y == -1
					&& rF->orientation_z == 0)
				_rangeFinderLeft = rF;

		}
	}

	virtual void update() {
//		_hal->debug->printf_P(
//				PSTR("guide loop, number: %d, current index: %d, previous index: %d\n"),
//				getNumberOfCommands(),
//				getCurrentIndex(),
//				getPreviousIndex());

		// if we don't have enough waypoint for cross track calcs
		// go home
		if (_numberOfCommands == 1) {
			_mode = MAV_NAV_RETURNING;
			_altitudeCommand = AP_MavlinkCommand::home.getAlt();
			_headingCommand = AP_MavlinkCommand::home.bearingTo(
					_navigator->getLat_degInt(), _navigator->getLon_degInt())
					+ 180 * deg2Rad;
			if (_headingCommand > 360 * deg2Rad)
				_headingCommand -= 360 * deg2Rad;

//			_hal->debug->printf_P(PSTR("going home: bearing: %f distance: %f\n"),
//			headingCommand,AP_MavlinkCommand::home.distanceTo(_navigator->getLat_degInt(),_navigator->getLon_degInt()));
		} else {
			_mode = MAV_NAV_WAYPOINT;
			_altitudeCommand = _command.getAlt();
			// TODO wrong behavior if 0 selected as waypoint, says previous 0
			float dXt = _command.crossTrack(_previousCommand,
					_navigator->getLat_degInt(),
					_navigator->getLon_degInt());
			float temp = dXt * _crossTrackGain * deg2Rad; // crosstrack gain, rad/m
			if (temp > _crossTrackLim * deg2Rad)
				temp = _crossTrackLim * deg2Rad;
			if (temp < -_crossTrackLim * deg2Rad)
				temp = -_crossTrackLim * deg2Rad;
			float bearing = _previousCommand.bearingTo(_command);
			_headingCommand = bearing - temp;
			float alongTrack = _command.alongTrack(_previousCommand,
					_navigator->getLat_degInt(),
					_navigator->getLon_degInt());
			float distanceToNext = _command.distanceTo(
					_navigator->getLat_degInt(), _navigator->getLon_degInt());
			float segmentLength = _previousCommand.distanceTo(_command);
			if (distanceToNext < _command.getRadius() || alongTrack
					> segmentLength)
			{
				Serial.println("radius reached");
				nextCommand();
			}
//			_hal->debug->printf_P(
//					PSTR("nav: bCurrent2Dest: %f\tdXt: %f\tcmdHeading: %f\tnextWpDistance: %f\talongTrack: %f\n"),
//					bearing * rad2Deg, dXt, _headingCommand * rad2Deg, distanceToNext, alongTrack);
		}

		_groundSpeedCommand = _velocityCommand;

		// TODO : calculate pN,pE,pD from home and gps coordinates
		_pNCmd = _command.getPN(_navigator->getLat_degInt(),
				_navigator->getLon_degInt());
		_pECmd = _command.getPE(_navigator->getLat_degInt(),
				_navigator->getLon_degInt());
		_pDCmd = _command.getPD(_navigator->getAlt_intM());

		// process mavlink commands
		//handleCommand();

		// obstacle avoidance overrides
		// stop if your going to drive into something in front of you
		for (uint8_t i = 0; i < _hal->rangeFinders.getSize(); i++)
			_hal->rangeFinders[i]->read();
		float frontDistance = _rangeFinderFront->distance / 200.0; //convert for other adc
		if (_rangeFinderFront && frontDistance < 2) {
			_mode = MAV_NAV_VECTOR;
			//airSpeedCommand = 0;
			//groundSpeedCommand = 0;
//			_headingCommand -= 45 * deg2Rad;
//			_hal->debug->print("Obstacle Distance (m): ");
//			_hal->debug->println(frontDistance);
//			_hal->debug->print("Obstacle avoidance Heading Command: ");
//			_hal->debug->println(headingCommand);
//			_hal->debug->printf_P(
//					PSTR("Front Distance, %f\n"),
//					frontDistance);
		}
		if (_rangeFinderBack && _rangeFinderBack->distance < 5) {
			_airSpeedCommand = 0;
			_groundSpeedCommand = 0;

		}

		if (_rangeFinderLeft && _rangeFinderLeft->distance < 5) {
			_airSpeedCommand = 0;
			_groundSpeedCommand = 0;
		}

		if (_rangeFinderRight && _rangeFinderRight->distance < 5) {
			_airSpeedCommand = 0;
			_groundSpeedCommand = 0;
		}
	}

	void nextCommand() {
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

		_cmdIndex = getNextIndex();
		Serial.print("cmd       : "); Serial.println(int(_cmdIndex));
		Serial.print("cmd prev  : "); Serial.println(int(getPreviousIndex()));
		Serial.print("cmd num    : "); Serial.println(int(getNumberOfCommands()));
		_command = AP_MavlinkCommand(getCurrentIndex());
		_previousCommand = AP_MavlinkCommand(getPreviousIndex());
	}

	void handleCommand(AP_MavlinkCommand command,
			AP_MavlinkCommand previousCommand) {
		// TODO handle more commands
		switch (command.getCommand()) {
		case MAV_CMD_NAV_WAYPOINT: {
			// if within radius, increment
			float d = previousCommand.distanceTo(_navigator->getLat_degInt(),
					_navigator->getLon_degInt());
			if (d < command.getRadius()) {
				nextCommand();
				Serial.println("radius reached");
			}
			break;
		}
//		case MAV_CMD_CONDITION_CHANGE_ALT:
//		case MAV_CMD_CONDITION_DELAY:
//		case MAV_CMD_CONDITION_DISTANCE:
//		case MAV_CMD_CONDITION_LAST:
//		case MAV_CMD_CONDITION_YAW:
//		case MAV_CMD_DO_CHANGE_SPEED:
//		case MAV_CMD_DO_CONTROL_VIDEO:
//		case MAV_CMD_DO_JUMP:
//	    case MAV_CMD_DO_LAST:
//		case MAV_CMD_DO_LAST:
//		case MAV_CMD_DO_REPEAT_RELAY:
//		case MAV_CMD_DO_REPEAT_SERVO:
//		case MAV_CMD_DO_SET_HOME:
//		case MAV_CMD_DO_SET_MODE:
//		case MAV_CMD_DO_SET_PARAMETER:
//		case MAV_CMD_DO_SET_RELAY:
//		case MAV_CMD_DO_SET_SERVO:
//		case MAV_CMD_PREFLIGHT_CALIBRATION:
//		case MAV_CMD_PREFLIGHT_STORAGE:
//		case MAV_CMD_NAV_LAND:
//		case MAV_CMD_NAV_LAST:
//		case MAV_CMD_NAV_LOITER_TIME:
//		case MAV_CMD_NAV_LOITER_TURNS:
//		case MAV_CMD_NAV_LOITER_UNLIM:
//		case MAV_CMD_NAV_ORIENTATION_TARGET:
//		case MAV_CMD_NAV_PATHPLANNING:
//		case MAV_CMD_NAV_RETURN_TO_LAUNCH:
//		case MAV_CMD_NAV_TAKEOFF:
		default:
			// unhandled command, skip
			Serial.println("unhandled command");
			nextCommand();
			break;
		}
	}
private:
	RangeFinder * _rangeFinderFront;
	RangeFinder * _rangeFinderBack;
	RangeFinder * _rangeFinderLeft;
	RangeFinder * _rangeFinderRight;
	AP_Var_group _group;
	AP_Float _velocityCommand;
	AP_Float _crossTrackGain;
	AP_Float _crossTrackLim;
};

} // namespace apo

#endif // AP_Guide_H
// vim:ts=4:sw=4:expandtab
