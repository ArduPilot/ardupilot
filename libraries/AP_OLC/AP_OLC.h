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
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_PLUSCODE_ENABLE
#define HAL_PLUSCODE_ENABLE !HAL_MINIMIZE_FEATURES && BOARD_FLASH_SIZE > 1024
#endif


#if HAL_PLUSCODE_ENABLE
class AP_OLC
{
public:

    // olc_encodes the given coordinates in lat and lon (deg * OLC_DEG_MULTIPLIER)
    // as an OLC code of the given length. It returns the number of characters
    // written to buf.
    static int olc_encode(int32_t lat, int32_t lon, size_t length, char *buf, size_t bufsize);

private:

    static constexpr uint8_t SEPARATOR_CHAR = '+';
    static constexpr uint8_t SEPARATOR_POS = 8;
    static constexpr uint8_t PADDING_CHAR = '0';

    static constexpr uint8_t ENCODING_BASE = 20;
    static constexpr uint8_t PAIR_CODE_LEN = 10;
    static constexpr uint8_t CODE_LEN_MAX = 15;

    static constexpr uint8_t GRID_COLS = 4;
    static constexpr uint8_t GRID_ROWS = ENCODING_BASE / GRID_COLS;

    static constexpr int32_t OLC_DEG_MULTIPLIER = 10000000; // 1e7

    static constexpr int32_t LAT_MAX = 90 * OLC_DEG_MULTIPLIER;
    static constexpr int32_t LON_MAX = 180 * OLC_DEG_MULTIPLIER;

    static const int32_t initial_exponent;
    // Work out the enclosing resolution (in degrees) for the grid algorithm.
    static const int32_t grid_size;
    // Work out the initial resolution
    static const int32_t initial_resolution;

    static constexpr char olc_alphabet[] = "23456789CFGHJMPQRVWX";

    static float compute_precision_for_length(int length);
    static int32_t adjust_latitude(int32_t lat, size_t code_len);
    static int32_t normalize_longitude(int32_t lon);
    static unsigned encode_pairs(uint32_t lat, uint32_t lon, size_t length, char *buf, size_t bufsize);
    static int encode_grid(uint32_t lat, uint32_t lon, size_t length, char *buf, size_t bufsize);

};

#endif