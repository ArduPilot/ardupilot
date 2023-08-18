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

#include <stdint.h>

// interface to set the vehicles mode
enum class ModeReason : uint8_t {
  UNKNOWN = 0,
  RC_COMMAND = 1,
  GCS_COMMAND = 2,
  RADIO_FAILSAFE = 3,
  BATTERY_FAILSAFE = 4,
  GCS_FAILSAFE = 5,
  EKF_FAILSAFE = 6,
  GPS_GLITCH = 7,
  MISSION_END = 8,
  THROTTLE_LAND_ESCAPE = 9,
  FENCE_BREACHED = 10,
  TERRAIN_FAILSAFE = 11,
  BRAKE_TIMEOUT = 12,
  FLIP_COMPLETE = 13,
  AVOIDANCE = 14,
  AVOIDANCE_RECOVERY = 15,
  THROW_COMPLETE = 16,
  TERMINATE = 17,
  TOY_MODE = 18,
  CRASH_FAILSAFE = 19,
  SOARING_FBW_B_WITH_MOTOR_RUNNING = 20,
  SOARING_THERMAL_DETECTED = 21,
  SOARING_THERMAL_ESTIMATE_DETERIORATED = 22,
  VTOL_FAILED_TRANSITION = 23,
  VTOL_FAILED_TAKEOFF = 24,
  FAILSAFE = 25, // general failsafes, prefer specific failsafes over this as much as possible
  INITIALISED = 26,
  SURFACE_COMPLETE = 27,
  BAD_DEPTH = 28,
  LEAK_FAILSAFE = 29,
  SERVOTEST = 30,
  STARTUP = 31,
  SCRIPTING = 32,
  UNAVAILABLE = 33,
  AUTOROTATION_START = 34,
  AUTOROTATION_BAILOUT = 35,
  SOARING_ALT_TOO_HIGH = 36,
  SOARING_ALT_TOO_LOW = 37,
  SOARING_DRIFT_EXCEEDED = 38,
  RTL_COMPLETE_SWITCHING_TO_VTOL_LAND_RTL = 39,
  RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND = 40,
  MISSION_CMD = 41,
  FRSKY_COMMAND = 42,
  FENCE_RETURN_PREVIOUS_MODE = 43,
  QRTL_INSTEAD_OF_RTL = 44,
  AUTO_RTL_EXIT = 45,
  LOITER_ALT_REACHED_QLAND = 46,
  LOITER_ALT_IN_VTOL = 47,
  RADIO_FAILSAFE_RECOVERY = 48,
  QLAND_INSTEAD_OF_RTL = 49,
  DEADRECKON_FAILSAFE = 50,
  MODE_TAKEOFF_FAILSAFE = 51,
};
