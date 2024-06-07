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

#include "AP_WindVane_config.h"

#if AP_WINDVANE_ANALOG_ENABLED

#include "AP_WindVane_Analog.h"

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define WINDVANE_CALIBRATION_VOLT_DIFF_MIN  1.0f    // calibration routine's min voltage difference required for success

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

// constructor
AP_WindVane_Analog::AP_WindVane_Analog(AP_WindVane &frontend) :
    AP_WindVane_Backend(frontend)
{
    _dir_analog_source = hal.analogin->channel(ANALOG_INPUT_NONE);
}

void AP_WindVane_Analog::update_direction()
{
    if (!_dir_analog_source->set_pin(_frontend._dir_analog_pin)) {
        // pin invalid, don't have health monitoring to report yet
        return;
    }
    _current_analog_voltage = _dir_analog_source->voltage_latest();

    const float voltage_ratio = linear_interpolate(0.0f, 1.0f, _current_analog_voltage, _frontend._dir_analog_volt_min, _frontend._dir_analog_volt_max);
    const float direction = (voltage_ratio * radians(360 - _frontend._dir_analog_deadzone)) + radians(_frontend._dir_analog_bearing_offset);

    _frontend._direction_apparent_raw  = wrap_PI(direction);
}

void AP_WindVane_Analog::calibrate()
{

    // start calibration
    if (_cal_start_ms == 0) {
        _cal_start_ms = AP_HAL::millis();
        _cal_volt_max = _current_analog_voltage;
        _cal_volt_min = _current_analog_voltage;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "WindVane: Calibration started, rotate wind vane");
    }

    // record min and max voltage
    _cal_volt_max = MAX(_cal_volt_max, _current_analog_voltage);
    _cal_volt_min = MIN(_cal_volt_min, _current_analog_voltage);

    // calibrate for 30 seconds
    if ((AP_HAL::millis() - _cal_start_ms) > 30000) {
        // check for required voltage difference
        const float volt_diff = _cal_volt_max - _cal_volt_min;
        if (volt_diff >= WINDVANE_CALIBRATION_VOLT_DIFF_MIN) {
            // save min and max voltage
            _frontend._dir_analog_volt_max.set_and_save(_cal_volt_max);
            _frontend._dir_analog_volt_min.set_and_save(_cal_volt_min);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "WindVane: Calibration complete (volt min:%.1f max:%1.f)",
            (double)_cal_volt_min,
            (double)_cal_volt_max);
        } else {
             GCS_SEND_TEXT(MAV_SEVERITY_INFO, "WindVane: Calibration failed (volt diff %.1f below %.1f)",
            (double)volt_diff,
            (double)WINDVANE_CALIBRATION_VOLT_DIFF_MIN);
        }
        _frontend._calibration.set_and_save(0);
        _cal_start_ms = 0;
    }
}

#endif  // AP_WINDVANE_ANALOG_ENABLED
