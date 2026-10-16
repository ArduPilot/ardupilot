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
#pragma once

#include <type_traits> // 2026-10-05

// 2026-10-05: Match the native enum range, including its end sentinel.
namespace AP_RangeFinder_DistanceSensorTypes {
enum NativeRange {
    MIN_VALUE = 0,
    MAX_VALUE = 5,
};
}

// distance sensor type enum, decoupled from MAVLink MAV_DISTANCE_SENSOR.
// values must match MAVLink to allow direct casting.
enum class AP_RangeFinder_DistanceSensorType :
    std::underlying_type<AP_RangeFinder_DistanceSensorTypes::NativeRange>::type {
    LASER      = 0,
    ULTRASOUND = 1,
    INFRARED   = 2,
    RADAR      = 3,
    UNKNOWN    = 4,
};
