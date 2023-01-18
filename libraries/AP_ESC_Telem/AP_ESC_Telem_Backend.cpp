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

#include "AP_ESC_Telem_Backend.h"
#include "AP_ESC_Telem.h"
#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_ESC_TELEM

#include <AP_Math/AP_Math.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL& hal;

AP_ESC_Telem_Backend::AP_ESC_Telem_Backend() {
    _frontend = AP_ESC_Telem::_singleton;
#if !APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    // we allow for no frontend in example fw and tools to make it
    // possible to run them on hardware with IOMCU
    if (_frontend == nullptr) {
        AP_HAL::panic("No ESC frontend");
    }
#endif
}

// callback to update the rpm in the frontend, should be called by the driver when new data is available
void AP_ESC_Telem_Backend::update_rpm(const uint8_t esc_index, const float new_rpm, const float error_rate) {
    _frontend->update_rpm(esc_index, new_rpm, error_rate);
}

// callback to update the data in the frontend, should be called by the driver when new data is available
void AP_ESC_Telem_Backend::update_telem_data(const uint8_t esc_index, const TelemetryData& new_data, const uint16_t data_present_mask) {
    _frontend->update_telem_data(esc_index, new_data, data_present_mask);
}

/*
  return true if the data is stale
 */
bool AP_ESC_Telem_Backend::TelemetryData::stale(uint32_t now_ms) const volatile
{
    if (now_ms == 0) {
        now_ms = AP_HAL::millis();
    }
    return last_update_ms == 0 || now_ms - last_update_ms > ESC_TELEM_DATA_TIMEOUT_MS;
}

// method shared between HobbyWing and DataLink backends:
void AP_ESC_Telem_Backend::update_motor_data_from_HWESC(uint8_t motor, const HWESC &decoded)
{
    TelemetryData t;
    uint16_t data_mask = 0;
    if (!isnan(decoded.temperature)) {
        t.temperature_cdeg = int16_t(decoded.temperature * 100);
        data_mask |= TelemetryType::TEMPERATURE;
    }
    if (!isnan(decoded.voltage)) {
        t.voltage = decoded.voltage;
        data_mask |= TelemetryType::VOLTAGE;
    }
    if (!isnan(decoded.current)) {
        t.current = decoded.current;
        data_mask |= TelemetryType::CURRENT;
    }

    update_telem_data(motor, t, data_mask);

    if (!isnan(decoded.rpm)) {
        update_rpm(motor, decoded.rpm, 0);
    }
}


static const struct {
    uint8_t adc_temp;
    uint8_t temp_C;
} temp_table[] = {
    { 241, 	0}, 	{ 240, 	1}, 	{ 239, 	2}, 	{ 238, 	3}, 	{ 237, 	4}, 	{ 236, 	5}, 	{ 235, 	6}, 	{ 234, 	7}, 	{ 233, 	8}, 	{ 232, 	9},
    { 231, 	10}, 	{ 230, 	11}, 	{ 229, 	12}, 	{ 228, 	13}, 	{ 227, 	14}, 	{ 226, 	15}, 	{ 224, 	16}, 	{ 223, 	17}, 	{ 222, 	18}, 	{ 220, 	19},
    { 219, 	20}, 	{ 217, 	21}, 	{ 216, 	22}, 	{ 214, 	23}, 	{ 213, 	24}, 	{ 211, 	25}, 	{ 209, 	26}, 	{ 208, 	27}, 	{ 206, 	28}, 	{ 204, 	29},
    { 202, 	30}, 	{ 201, 	31}, 	{ 199, 	32}, 	{ 197, 	33}, 	{ 195, 	34}, 	{ 193, 	35}, 	{ 191, 	36}, 	{ 189, 	37}, 	{ 187, 	38}, 	{ 185, 	39},
    { 183, 	40}, 	{ 181, 	41}, 	{ 179, 	42}, 	{ 177, 	43}, 	{ 174, 	44}, 	{ 172, 	45}, 	{ 170, 	46}, 	{ 168, 	47}, 	{ 166, 	48}, 	{ 164, 	49},
    { 161, 	50}, 	{ 159, 	51}, 	{ 157, 	52}, 	{ 154, 	53}, 	{ 152, 	54}, 	{ 150, 	55}, 	{ 148, 	56}, 	{ 146, 	57}, 	{ 143, 	58}, 	{ 141, 	59},
    { 139, 	60}, 	{ 136, 	61}, 	{ 134, 	62}, 	{ 132, 	63}, 	{ 130, 	64}, 	{ 128, 	65}, 	{ 125, 	66}, 	{ 123, 	67}, 	{ 121, 	68}, 	{ 119, 	69},
    { 117, 	70}, 	{ 115, 	71}, 	{ 113, 	72}, 	{ 111, 	73}, 	{ 109, 	74}, 	{ 106, 	75}, 	{ 105, 	76}, 	{ 103, 	77}, 	{ 101, 	78}, 	{ 99, 	79},
    { 97, 	80}, 	{ 95, 	81}, 	{ 93, 	82}, 	{ 91, 	83}, 	{ 90, 	84}, 	{ 88, 	85}, 	{ 85, 	86}, 	{ 84, 	87}, 	{ 82, 	88}, 	{ 81, 	89},
    { 79, 	90}, 	{ 77, 	91}, 	{ 76, 	92}, 	{ 74, 	93}, 	{ 73, 	94}, 	{ 72, 	95}, 	{ 69, 	96}, 	{ 68, 	97}, 	{ 66, 	98}, 	{ 65, 	99},
    { 64, 	100}, 	{ 62, 	101}, 	{ 62, 	102}, 	{ 61, 	103}, 	{ 59, 	104}, 	{ 58, 	105}, 	{ 56, 	106}, 	{ 54, 	107}, 	{ 54, 	108}, 	{ 53, 	109},
    { 51, 	110}, 	{ 51, 	111}, 	{ 50, 	112}, 	{ 48, 	113}, 	{ 48, 	114}, 	{ 46, 	115}, 	{ 46, 	116}, 	{ 44, 	117}, 	{ 43, 	118}, 	{ 43, 	119},
    { 41, 	120}, 	{ 41, 	121}, 	{ 39, 	122}, 	{ 39, 	123}, 	{ 39, 	124}, 	{ 37, 	125}, 	{ 37, 	126}, 	{ 35, 	127}, 	{ 35, 	128}, 	{ 33, 	129},
};

uint8_t AP_ESC_Telem_Backend::temperature_decode(uint8_t temp_raw)
{
    for (uint8_t i=0; i<ARRAY_SIZE(temp_table); i++) {
        if (temp_table[i].adc_temp <= temp_raw) {
            return temp_table[i].temp_C;
        }
    }
    return 130U;
}


#endif
