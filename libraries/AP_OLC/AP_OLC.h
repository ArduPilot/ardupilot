/*
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
 *
 * AP_OLC is based on INAV olc.c implemention, thanks @fiam and other contributors.
 */

#pragma once
#include <stdint.h>
#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_PLUSCODE_ENABLE
#define HAL_PLUSCODE_ENABLE BOARD_FLASH_SIZE > 1024
#endif


#if HAL_PLUSCODE_ENABLE
class AP_OLC
{
public:

    // olc_encodes the given coordinates in lat and lon (deg * OLC_DEG_MULTIPLIER)
    // as an OLC code of the given length. It returns the number of characters
    // written to buf.
    static uint32_t olc_encode(int32_t lat, int32_t lon, uint8_t length, char *buf, uint8_t bufsize);

private:
    static const int32_t INITIAL_EXPONENT;
    // Work out the enclosing resolution (in degrees) for the grid algorithm.
    static const int32_t GRID_SIZE;
    // Work out the initial resolution
    static const int32_t INITIAL_RESOLUTION;

    static constexpr char olc_alphabet[] = "23456789CFGHJMPQRVWX";

    static float compute_precision_for_length(uint8_t length);
    static int32_t adjust_latitude(int32_t lat, uint8_t code_len);
    static int32_t normalize_longitude(int32_t lon);
    static uint32_t encode_pairs(uint32_t lat, uint32_t lon, uint8_t length, char *buf, uint8_t bufsize);
    static int32_t encode_grid(uint32_t lat, uint32_t lon, uint8_t length, char *buf, uint8_t bufsize);

};

#endif
