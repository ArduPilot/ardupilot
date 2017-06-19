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

union nav_filter_status {
    struct {
        uint16_t attitude           : 1; // 0 - true if attitude estimate is valid
        uint16_t horiz_vel          : 1; // 1 - true if horizontal velocity estimate is valid
        uint16_t vert_vel           : 1; // 2 - true if the vertical velocity estimate is valid
        uint16_t horiz_pos_rel      : 1; // 3 - true if the relative horizontal position estimate is valid
        uint16_t horiz_pos_abs      : 1; // 4 - true if the absolute horizontal position estimate is valid
        uint16_t vert_pos           : 1; // 5 - true if the vertical position estimate is valid
        uint16_t terrain_alt        : 1; // 6 - true if the terrain height estimate is valid
        uint16_t const_pos_mode     : 1; // 7 - true if we are in const position mode
        uint16_t pred_horiz_pos_rel : 1; // 8 - true if filter expects it can produce a good relative horizontal position estimate - used before takeoff
        uint16_t pred_horiz_pos_abs : 1; // 9 - true if filter expects it can produce a good absolute horizontal position estimate - used before takeoff
        uint16_t takeoff_detected   : 1; // 10 - true if optical flow takeoff has been detected
        uint16_t takeoff            : 1; // 11 - true if filter is compensating for baro errors during takeoff
        uint16_t touchdown          : 1; // 12 - true if filter is compensating for baro errors during touchdown
        uint16_t using_gps          : 1; // 13 - true if we are using GPS position
        uint16_t gps_glitching      : 1; // 14 - true if the the GPS is glitching
    } flags;
    uint16_t value;
};

union nav_gps_status {
    struct {
        uint16_t bad_sAcc           : 1; // 0 - true if reported gps speed accuracy is insufficient to start using GPS
        uint16_t bad_hAcc           : 1; // 1 - true if reported gps horizontal position accuracy is insufficient to start using GPS
        uint16_t bad_yaw            : 1; // 2 - true if EKF yaw errors are too large to start using GPS
        uint16_t bad_sats           : 1; // 3 - true if the number of satellites is insufficient to start using GPS
        uint16_t bad_VZ             : 1; // 4 - true if the vertical velocity is inconsistent with the inertial/baro
        uint16_t bad_horiz_drift    : 1; // 5 - true if the GPS horizontal position is drifting (this check assumes vehicle is static)
        uint16_t bad_hdop           : 1; // 6 - true if the reported HDoP is insufficient to start using GPS
        uint16_t bad_vert_vel       : 1; // 7 - true if the GPS vertical speed is too large to start using GPS (this check assumes vehicle is static)
        uint16_t bad_fix            : 1; // 8 - true if the GPS is not providing a 3D fix
        uint16_t bad_horiz_vel      : 1; // 9 - true if the GPS horizontal speed is excessive (this check assumes the vehicle is static)
        uint16_t bad_vAcc           : 1; // 10 - true if reported gps vertical position accuracy is insufficient to start using GPS
    } flags;
    uint16_t value;
};
