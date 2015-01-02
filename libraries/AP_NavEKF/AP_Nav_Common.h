/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#ifndef AP_Nav_Common
#define AP_Nav_Common

union nav_filter_status {
    struct {
        uint8_t attitude        : 1; // 0 - true if attitude estimate is valid
        uint8_t horiz_vel       : 1; // 1 - true if horizontal velocity estimate is valid
        uint8_t vert_vel        : 1; // 2 - true if the vertical velocity estimate is valid
        uint8_t horiz_pos_rel   : 1; // 3 - true if the relative horizontal position estimate is valid
        uint8_t horiz_pos_abs   : 1; // 4 - true if the absolute horizontal position estimate is valid
        uint8_t vert_pos        : 1; // 5 - true if the vertical position estimate is valid
        uint8_t terrain_alt     : 1; // 6 - true if the terrain height estimate is valid
        uint8_t const_pos_mode  : 1; // 7 - true if we are in const position mode
    } flags;
    uint8_t value;
};

#endif // AP_Nav_Common
