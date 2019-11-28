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
  UNKNOWN,
  RC_COMMAND,
  GCS_COMMAND,
  RADIO_FAILSAFE,
  BATTERY_FAILSAFE,
  GCS_FAILSAFE,
  EKF_FAILSAFE,
  GPS_GLITCH,
  MISSION_END,
  THROTTLE_LAND_ESCAPE,
  FENCE_BREACHED,
  TERRAIN_FAILSAFE,
  BRAKE_TIMEOUT,
  FLIP_COMPLETE,
  AVOIDANCE,
  AVOIDANCE_RECOVERY,
  THROW_COMPLETE,
  TERMINATE,
  TOY_MODE,
  CRASH_FAILSAFE,
  SOARING_FBW_B_WITH_MOTOR_RUNNING,
  SOARING_THERMAL_DETECTED,
  SOARING_THERMAL_ESTIMATE_DETERIORATED,
  VTOL_FAILED_TRANSITION,
  VTOL_FAILED_TAKEOFF,
  FAILSAFE, // general failsafes, prefer specific failsafes over this as much as possible
  INITIALISED,
  SURFACE_COMPLETE,
  BAD_DEPTH,
  LEAK_FAILSAFE,
  SERVOTEST,
  STARTUP,
  SCRIPTING,
  UNAVAILABLE,
  AUTOROTATION_START,
  AUTOROTATION_BAILOUT,
};
