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

#include <inttypes.h>
#include "../GCS_MAVLink/GCS_MAVLink.h"
#include "AP_MavlinkCommand.h"
#include "../AP_RangeFinder/AP_RangeFinder.h"

namespace apo {

class AP_Navigator;
class AP_HardwareAbstractionLayer;

/// Guide class
class AP_Guide {
public:

	/**
	 * This is the constructor, which requires a link to the navigator.
	 * @param navigator This is the navigator pointer.
	 */
	AP_Guide(AP_Navigator * navigator, AP_HardwareAbstractionLayer * hal);

	virtual void update() = 0;

	virtual void nextCommand() = 0;

	MAV_NAV getMode() const {
		return _mode;
	}
	uint8_t getCurrentIndex() {
		return _cmdIndex;
	}

	void setCurrentIndex(uint8_t val);

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
			AP_HardwareAbstractionLayer * hal);
	virtual void update();
	void nextCommand();
	void handleCommand();

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
