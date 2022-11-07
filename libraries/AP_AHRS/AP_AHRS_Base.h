/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  Base class interface for the AHRS (Attitude Heading Reference System)
 *  frontend for ArduPilot. Designed to provide the minimum accessors required
 *  for other libraries that require vehicle position and orientation.
 *
 *  This class differs from AP_AHRS_View in that AP_AHRS_View is intended to
 *  provide a view of vehicle pitch that is rotated 90 degrees in pitch, for use
 *  in tailsitter attitude control.
 */

#pragma once

#include <AP_Common/Location.h>


class AP_AHRS_Base {
public:

    virtual bool get_home(struct Location &home) const WARN_IF_UNUSED = 0;
    virtual bool get_location(struct Location &loc) const WARN_IF_UNUSED = 0;
    virtual bool get_origin(struct Location &origin) const WARN_IF_UNUSED = 0;

    virtual const Vector3f &get_gyro(void) const = 0;

    // roll euler angle, in radians
    virtual float get_roll() const = 0;
    // pitch euler angle, in radians
    virtual float get_pitch() const = 0;
    // yaw euler angle, in radians
    virtual float get_yaw() const = 0;

    // helper trig value accessors
    virtual float cos_roll() const = 0;
    virtual float cos_pitch() const = 0;
    virtual float cos_yaw() const = 0;
    virtual float sin_roll() const = 0;
    virtual float sin_pitch() const = 0;
    virtual float sin_yaw() const = 0;

    virtual const Matrix3f &get_rotation_body_to_ned(void) const = 0;

};
