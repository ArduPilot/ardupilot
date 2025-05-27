/*
  AP_Nav_Common holds definitions shared by inertial and ekf nav filters

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
#include <AP_Math/AP_Math.h>

#define MAX_EKF_CORES     3 // maximum allowed EKF Cores to be instantiated

// enumeration corresponding to buts within nav_filter_status union.
// Only used for documentation purposes.
enum class NavFilterStatusBit {
    ATTITUDE_VALID     =      1, // attitude estimate valid
    HORIZ_VEL          =      2, // horizontal velocity estimate valid
    VERT_VEL           =      4, // vertical velocity estimate valid
    HORIZ_POS_REL      =      8, // relative horizontal position estimate valid
    HORIZ_POS_ABS      =     16, // absolute horizontal position estimate valid
    VERT_POS           =     32, // vertical position estimate valid
    TERRAIN_ALT        =     64, // terrain height estimate valid
    CONST_POS_MODE     =    128, // in constant position mode
    PRED_HORIZ_POS_REL =    256, // expected good relative horizontal position estimate - used before takeoff
    PRED_HORIZ_POS_ABS =    512, // expected good absolute horizontal position estimate - used before takeoff
    TAKEOFF_DETECTED   =   1024, // optical flow takeoff has been detected
    TAKEOFF_EXPECTED   =   2048, // compensating for baro errors during takeoff
    TOUCHDOWN_EXPECTED =   4096, // compensating for baro errors during touchdown
    USING_GPS          =   8192, // using GPS position
    GPS_GLITCHING      =  16384, // GPS glitching is affecting navigation accuracy
    GPS_QUALITY_GOOD   =  32768, // can use GPS for navigation
    INITALIZED         =  65536, // has ever been healthy
    REJECTING_AIRSPEED = 131072, // rejecting airspeed data
    DEAD_RECKONING     = 262144, // dead reckoning (e.g. no position or velocity source)
};

union nav_filter_status {
    struct {
        bool attitude           : 1; // 0 - true if attitude estimate is valid
        bool horiz_vel          : 1; // 1 - true if horizontal velocity estimate is valid
        bool vert_vel           : 1; // 2 - true if the vertical velocity estimate is valid
        bool horiz_pos_rel      : 1; // 3 - true if the relative horizontal position estimate is valid
        bool horiz_pos_abs      : 1; // 4 - true if the absolute horizontal position estimate is valid
        bool vert_pos           : 1; // 5 - true if the vertical position estimate is valid
        bool terrain_alt        : 1; // 6 - true if the terrain height estimate is valid
        bool const_pos_mode     : 1; // 7 - true if we are in const position mode
        bool pred_horiz_pos_rel : 1; // 8 - true if filter expects it can produce a good relative horizontal position estimate - used before takeoff
        bool pred_horiz_pos_abs : 1; // 9 - true if filter expects it can produce a good absolute horizontal position estimate - used before takeoff
        bool takeoff_detected   : 1; // 10 - true if optical flow takeoff has been detected
        bool takeoff            : 1; // 11 - true if filter is compensating for baro errors during takeoff
        bool touchdown          : 1; // 12 - true if filter is compensating for baro errors during touchdown
        bool using_gps          : 1; // 13 - true if we are using GPS position
        bool gps_glitching      : 1; // 14 - true if GPS glitching is affecting navigation accuracy
        bool gps_quality_good   : 1; // 15 - true if we can use GPS for navigation
        bool initalized         : 1; // 16 - true if the EKF has ever been healthy
        bool rejecting_airspeed : 1; // 17 - true if we are rejecting airspeed data
        bool dead_reckoning     : 1; // 18 - true if we are dead reckoning (e.g. no position or velocity source)
    } flags;
    uint32_t value;
};

static_assert(sizeof(uint32_t) == sizeof(nav_filter_status), "nav_filter_status must be uint32_t");

union nav_gps_status {
    struct {
        bool bad_sAcc           : 1; // 0 - true if reported gps speed accuracy is insufficient to start using GPS
        bool bad_hAcc           : 1; // 1 - true if reported gps horizontal position accuracy is insufficient to start using GPS
        bool bad_yaw            : 1; // 2 - true if EKF yaw errors are too large to start using GPS
        bool bad_sats           : 1; // 3 - true if the number of satellites is insufficient to start using GPS
        bool bad_VZ             : 1; // 4 - true if the vertical velocity is inconsistent with the inertial/baro
        bool bad_horiz_drift    : 1; // 5 - true if the GPS horizontal position is drifting (this check assumes vehicle is static)
        bool bad_hdop           : 1; // 6 - true if the reported HDoP is insufficient to start using GPS
        bool bad_vert_vel       : 1; // 7 - true if the GPS vertical speed is too large to start using GPS (this check assumes vehicle is static)
        bool bad_fix            : 1; // 8 - true if the GPS is not providing a 3D fix
        bool bad_horiz_vel      : 1; // 9 - true if the GPS horizontal speed is excessive (this check assumes the vehicle is static)
        bool bad_vAcc           : 1; // 10 - true if reported gps vertical position accuracy is insufficient to start using GPS
    } flags;
    uint16_t value;
};

static_assert(sizeof(uint16_t) == sizeof(nav_gps_status), "nav_gps_status must be uint16_t");

/*
  structure to hold EKF timing statistics
 */
struct ekf_timing {
    uint32_t count;
    float dtIMUavg_min;
    float dtIMUavg_max;
    float dtEKFavg_min;
    float dtEKFavg_max;
    float delAngDT_max;
    float delAngDT_min;
    float delVelDT_max;
    float delVelDT_min;
};

#define N_MODELS_EKFGSF 5U
