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
class AP_Board;

/// Guide class
class AP_Guide {
public:

    /**
     * This is the constructor, which requires a link to the navigator.
     * @param navigator This is the navigator pointer.
     */
    AP_Guide(AP_Navigator * nav, AP_Board * board);

    virtual void update() = 0;

    virtual void nextCommand() = 0;

    virtual void updateCommand() {};

    MAV_NAV getMode() const {
        return _mode;
    }

    void setMode(MAV_NAV mode) {
        _mode = mode;
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

    float getHeadingError();

    /// the commanded course over ground for the vehicle
    float getHeadingCommand() {
        return _headingCommand;
    }

    /// wrap an angle between -180 and 180
    float wrapAngle(float y) {
        if (y > 180 * deg2Rad)
            y -= 360 * deg2Rad;
        if (y < -180 * deg2Rad)
            y += 360 * deg2Rad;
        return y;
    }

    /// the yaw attitude error of the vehicle
    float getYawError();

    float getAirSpeedCommand() {
        return _airSpeedCommand;
    }
    float getGroundSpeedCommand() {
        return _groundSpeedCommand;
    }
    float getGroundSpeedError();

    float getAltitudeCommand() {
        return _altitudeCommand;
    }
    float getDistanceToNextWaypoint();

    virtual float getPNError() = 0;
    virtual float getPEError() = 0;
    virtual float getPDError() = 0;

    MAV_NAV getMode() {
        return _mode;
    }
    uint8_t getCommandIndex() {
        return _cmdIndex;
    }

protected:
    AP_Navigator * _nav;
    AP_Board * _board;
    AP_MavlinkCommand _command, _previousCommand;
    float _headingCommand;
    float _yawCommand;
    float _airSpeedCommand;
    float _groundSpeedCommand;
    float _altitudeCommand;
    MAV_NAV _mode;
    AP_Uint8 _numberOfCommands;
    AP_Uint8 _cmdIndex;
    uint16_t _nextCommandCalls;
    uint16_t _nextCommandTimer;
};

class MavlinkGuide: public AP_Guide {
public:
    MavlinkGuide(AP_Navigator * nav,
                 AP_Board * board, float velCmd, float xt, float xtLim);
    virtual void update();
    void nextCommand();
    void handleCommand();
    void updateCommand();
    virtual float getPNError();
    virtual float getPEError();
    virtual float getPDError();

private:
    AP_Var_group _group;
    AP_Float _velocityCommand;
    AP_Float _crossTrackGain;
    AP_Float _crossTrackLim;
};

} // namespace apo

#endif // AP_Guide_H
// vim:ts=4:sw=4:expandtab
