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
  Parser for the serial telemetry frames sent by HobbyWing XRotor ESCs
  and by T-Motor Alpha ESCs
 */

#include "AP_ESC_Telem_XRotor_Parser.h"

#if AP_ESC_TELEM_XROTOR_PARSER_ENABLED

#include <string.h>
#include <AP_HAL/utility/sparse-endian.h>

// every frame starts with this byte, then the number of bytes after those two
static constexpr uint8_t FRAME_HEADER = 0x9B;
static constexpr uint8_t FRAME_LENGTH_BYTE = AP_ESC_Telem_XRotor_Parser::FRAME_LEN - 2;

/*
  consume one received byte, returning true when it completes a frame
  whose checksum is good; get_frame() then returns its fields.

  A frame is found by its header and length bytes and checked with the
  16-bit sum of every byte before the checksum. A T-Motor Alpha sends a
  lone header byte about 20 ms before its motor starts, then nothing
  for over half a second, so a header byte where the length byte
  should be starts the frame again.
 */
bool AP_ESC_Telem_XRotor_Parser::parse_byte(uint8_t b)
{
    if (len == 0 && b != FRAME_HEADER) {
        return false;
    }
    if (len == 1 && b != FRAME_LENGTH_BYTE) {
        len = (b == FRAME_HEADER) ? 1 : 0;
        return false;
    }
    buf[len++] = b;
    if (len < FRAME_LEN) {
        return false;
    }

    uint16_t sum = 0;
    for (uint8_t i = 0; i < FRAME_LEN - 2; i++) {
        sum += buf[i];
    }
    if (sum != le16toh_ptr(&buf[FRAME_LEN - 2])) {
        resync();
        return false;
    }
    len = 0;

    frame.counter = be32toh_ptr(&buf[2]);
    frame.throttle_req = be16toh_ptr(&buf[6]);
    frame.throttle = be16toh_ptr(&buf[8]);
    frame.rpm = be16toh_ptr(&buf[10]);
    frame.voltage = be16toh_ptr(&buf[12]);
    frame.current = int16_t(be16toh_ptr(&buf[14]));
    frame.phase_current = int16_t(be16toh_ptr(&buf[16]));
    frame.mos_temperature = buf[18];
    frame.cap_temperature = buf[19];
    frame.status = be16toh_ptr(&buf[20]);
    return true;
}

/*
  restart from the first header byte after the start of a frame that
  failed its checksum, keeping the bytes after it. When a byte is lost,
  the frame absorbs the start of the next one, and this lets that next
  frame still be decoded.
 */
void AP_ESC_Telem_XRotor_Parser::resync()
{
    for (uint8_t i = 1; i < FRAME_LEN; i++) {
        if (buf[i] != FRAME_HEADER) {
            continue;
        }
        if (i + 1 < FRAME_LEN && buf[i + 1] != FRAME_LENGTH_BYTE) {
            continue;
        }
        len = FRAME_LEN - i;
        memmove(buf, &buf[i], len);
        return;
    }
    len = 0;
}

// raw NTC readings and the temperature in degrees C each corresponds to
static const struct {
    uint8_t adc_temp;
    uint8_t temp_C;
} temp_table[] = {
    { 241,   0}, { 240,   1}, { 239,   2}, { 238,   3}, { 237,   4}, { 236,   5}, { 235,   6}, { 234,   7}, { 233,   8}, { 232,   9},
    { 231,  10}, { 230,  11}, { 229,  12}, { 228,  13}, { 227,  14}, { 226,  15}, { 224,  16}, { 223,  17}, { 222,  18}, { 220,  19},
    { 219,  20}, { 217,  21}, { 216,  22}, { 214,  23}, { 213,  24}, { 211,  25}, { 209,  26}, { 208,  27}, { 206,  28}, { 204,  29},
    { 202,  30}, { 201,  31}, { 199,  32}, { 197,  33}, { 195,  34}, { 193,  35}, { 191,  36}, { 189,  37}, { 187,  38}, { 185,  39},
    { 183,  40}, { 181,  41}, { 179,  42}, { 177,  43}, { 174,  44}, { 172,  45}, { 170,  46}, { 168,  47}, { 166,  48}, { 164,  49},
    { 161,  50}, { 159,  51}, { 157,  52}, { 154,  53}, { 152,  54}, { 150,  55}, { 148,  56}, { 146,  57}, { 143,  58}, { 141,  59},
    { 139,  60}, { 136,  61}, { 134,  62}, { 132,  63}, { 130,  64}, { 128,  65}, { 125,  66}, { 123,  67}, { 121,  68}, { 119,  69},
    { 117,  70}, { 115,  71}, { 113,  72}, { 111,  73}, { 109,  74}, { 106,  75}, { 105,  76}, { 103,  77}, { 101,  78}, {  99,  79},
    {  97,  80}, {  95,  81}, {  93,  82}, {  91,  83}, {  90,  84}, {  88,  85}, {  85,  86}, {  84,  87}, {  82,  88}, {  81,  89},
    {  79,  90}, {  77,  91}, {  76,  92}, {  74,  93}, {  73,  94}, {  72,  95}, {  69,  96}, {  68,  97}, {  66,  98}, {  65,  99},
    {  64, 100}, {  62, 101}, {  62, 102}, {  61, 103}, {  59, 104}, {  58, 105}, {  56, 106}, {  54, 107}, {  54, 108}, {  53, 109},
    {  51, 110}, {  51, 111}, {  50, 112}, {  48, 113}, {  48, 114}, {  46, 115}, {  46, 116}, {  44, 117}, {  43, 118}, {  43, 119},
    {  41, 120}, {  41, 121}, {  39, 122}, {  39, 123}, {  39, 124}, {  37, 125}, {  37, 126}, {  35, 127}, {  35, 128}, {  33, 129},
};

/*
  convert a raw NTC reading from a frame to degrees C. The reading falls
  as the ESC heats, so the first table entry at or below it gives the
  temperature, and a reading below the whole table is reported as 130.
 */
uint8_t AP_ESC_Telem_XRotor_Parser::temperature_decode(uint8_t temp_raw)
{
    for (const auto &entry : temp_table) {
        if (entry.adc_temp <= temp_raw) {
            return entry.temp_C;
        }
    }
    return 130U;
}

#endif  // AP_ESC_TELEM_XROTOR_PARSER_ENABLED
