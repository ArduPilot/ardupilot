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

#include "AP_OLC.h"

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>

#if HAL_PLUSCODE_ENABLE
// This is a port of https://github.com/google/open-location-code/blob/master/c/olc.c
// to avoid double floating point math and use integer math as much as possible.

#define SEPARATOR_CHAR '+'
#define SEPARATOR_POS 8U
#define PADDING_CHAR '0'

#define ENCODING_BASE 20U
#define PAIR_CODE_LEN 10U
#define CODE_LEN_MAX 15U

#define GRID_COLS 4U
#define GRID_ROWS (ENCODING_BASE / GRID_COLS)

#define OLC_DEG_MULTIPLIER 10000000U // 1e7

#define LAT_MAX int32_t(90 * OLC_DEG_MULTIPLIER)
#define LON_MAX int32_t(180 * OLC_DEG_MULTIPLIER)

const int32_t AP_OLC::initial_exponent  = floorf(logf(2 * (LON_MAX / OLC_DEG_MULTIPLIER)) / logf(ENCODING_BASE));
const int32_t AP_OLC::grid_size = (1 / powf(ENCODING_BASE, PAIR_CODE_LEN / 2 - (initial_exponent + 1))) * OLC_DEG_MULTIPLIER;
const int32_t AP_OLC::initial_resolution = powf(ENCODING_BASE, initial_exponent) * OLC_DEG_MULTIPLIER;

constexpr char AP_OLC::olc_alphabet[];

// Compute the latitude precision value for a given code length.  Lengths <= 10
// have the same precision for latitude and longitude, but lengths > 10 have
// different precisions due to the grid method having fewer columns than rows.
float AP_OLC::compute_precision_for_length(int length)
{
    // Magic numbers!
    if (length <= (int)PAIR_CODE_LEN) {
        return powf(ENCODING_BASE, floorf((length / -2) + 2));
    }

    return powf(ENCODING_BASE, -3) / powf(5, length - (int)PAIR_CODE_LEN);
}

int32_t AP_OLC::adjust_latitude(int32_t lat, size_t code_len)
{

    lat = constrain_int32(lat, -LAT_MAX, LAT_MAX);

    if (lat >= LAT_MAX) {
        // Subtract half the code precision to get the latitude into the code area.
        int32_t precision = compute_precision_for_length(code_len) * OLC_DEG_MULTIPLIER;
        lat -= precision / 2;
    }
    return lat;
}

int32_t AP_OLC::normalize_longitude(int32_t lon)
{
    while (lon < -LON_MAX) {
        lon += LON_MAX;
        lon += LON_MAX;
    }
    while (lon >= LON_MAX) {
        lon -= LON_MAX;
        lon -= LON_MAX;
    }
    return lon;
}

// Encodes positive range lat,lon into a sequence of OLC lat/lon pairs.  This
// uses pairs of characters (latitude and longitude in that order) to represent
// each step in a 20x20 grid.  Each code, therefore, has 1/400th the area of
// the previous code.
unsigned AP_OLC::encode_pairs(uint32_t lat, uint32_t lon, size_t length, char *buf, size_t bufsize)
{
    if ((length + 1) >= bufsize) {
        buf[0] = '\0';
        return 0;
    }

    unsigned pos = 0;
    int32_t resolution = initial_resolution;
    // Add two digits on each pass.
    for (size_t digit_count = 0;
         digit_count < length;
         digit_count += 2, resolution /= ENCODING_BASE) {
        size_t digit_value;

        // Do the latitude - gets the digit for this place and subtracts that
        // for the next digit.
        digit_value = lat / resolution;
        lat -= digit_value * resolution;
        buf[pos++] = olc_alphabet[digit_value];

        // Do the longitude - gets the digit for this place and subtracts that
        // for the next digit.
        digit_value = lon / resolution;
        lon -= digit_value * resolution;
        buf[pos++] = olc_alphabet[digit_value];

        // Should we add a separator here?
        if (pos == SEPARATOR_POS && pos < length) {
            buf[pos++] = SEPARATOR_CHAR;
        }
    }
    while (pos < SEPARATOR_POS) {
        buf[pos++] = PADDING_CHAR;
    }
    if (pos == SEPARATOR_POS) {
        buf[pos++] = SEPARATOR_CHAR;
    }
    buf[pos] = '\0';
    return pos;
}

// Encodes a location using the grid refinement method into an OLC string.  The
// grid refinement method divides the area into a grid of 4x5, and uses a
// single character to refine the area.  The grid squares use the OLC
// characters in order to number the squares as follows:
//
//   R V W X
//   J M P Q
//   C F G H
//   6 7 8 9
//   2 3 4 5
//
// This allows default accuracy OLC codes to be refined with just a single
// character.
int AP_OLC::encode_grid(uint32_t lat, uint32_t lon, size_t length,
                       char *buf, size_t bufsize)
{
    if ((length + 1) >= bufsize) {
        buf[0] = '\0';
        return 0;
    }

    int pos = 0;

    int32_t lat_grid_size = grid_size;
    int32_t lon_grid_size = grid_size;

    lat %= lat_grid_size;
    lon %= lon_grid_size;

    for (size_t i = 0; i < length; i++) {
        int32_t lat_div = lat_grid_size / GRID_ROWS;
        int32_t lon_div = lon_grid_size / GRID_COLS;

        if (lat_div == 0 || lon_div == 0) {
            // This case happens when OLC_DEG_MULTIPLIER doesn't have enough
            // precision for the requested length.
            break;
        }

        // Work out the row and column.
        size_t row = lat / lat_div;
        size_t col = lon / lon_div;
        lat_grid_size /= GRID_ROWS;
        lon_grid_size /= GRID_COLS;
        lat -= row * lat_grid_size;
        lon -= col * lon_grid_size;
        buf[pos++] = olc_alphabet[row * GRID_COLS + col];
    }
    buf[pos] = '\0';
    return pos;
}

int AP_OLC::olc_encode(int32_t lat, int32_t lon, size_t length, char *buf, size_t bufsize)
{
    int pos = 0;

    length = MIN(length, CODE_LEN_MAX);

    // Adjust latitude and longitude so they fall into positive ranges.
    uint32_t alat = adjust_latitude(lat, length) + LAT_MAX;
    uint32_t alon = normalize_longitude(lon) + LON_MAX;

    pos += encode_pairs(alat, alon, MIN(length, PAIR_CODE_LEN), buf + pos, bufsize - pos);
    // If the requested length indicates we want grid refined codes.
    if (length > PAIR_CODE_LEN) {
        pos += encode_grid(alat, alon, length - PAIR_CODE_LEN, buf + pos, bufsize - pos);
    }
    buf[pos] = '\0';
    return pos;
}

#endif 
