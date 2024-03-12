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

#include "AP_TemperatureSensor.h"

#if AP_TEMPERATURE_SENSOR_ENABLED
#include "AP_TemperatureSensor_Backend.h"

#include <AP_Logger/AP_Logger.h>
#include <AP_BattMonitor/AP_BattMonitor.h>

/*
    base class constructor.
    This incorporates initialisation as well.
*/
AP_TemperatureSensor_Backend::AP_TemperatureSensor_Backend(AP_TemperatureSensor &front,
                                                            AP_TemperatureSensor::TemperatureSensor_State &state,
                                                            AP_TemperatureSensor_Params &params):
    _front(front),
    _state(state),
    _params(params)
{
}

// returns true if a temperature has been recently updated
bool AP_TemperatureSensor_Backend::healthy(void) const
{
    return (_state.last_time_ms > 0) && (AP_HAL::millis() - _state.last_time_ms < 5000);
}

void AP_TemperatureSensor_Backend::Log_Write_TEMP() const
{
    AP::logger().Write("TEMP",
            "TimeUS,"     "Instance,"       "Temp" , // labels
            "s"               "#"           "O"    , // units
            "F"               "-"           "0"    , // multipliers
            "Q"               "B"           "f"    , // types
     AP_HAL::micros64(), _state.instance, _state.temperature);
}

void AP_TemperatureSensor_Backend::set_temperature(const float temperature)
{
    {
        WITH_SEMAPHORE(_sem);
        _state.temperature = temperature;
        _state.last_time_ms = AP_HAL::millis();
    }

    update_external_libraries(temperature);
}

void AP_TemperatureSensor_Backend::update_external_libraries(const float temperature)
{
#if HAL_WITH_ESC_TELEM
    AP_ESC_Telem_Backend::TelemetryData t;
#endif

    switch ((AP_TemperatureSensor_Params::Source)_params.source.get()) {
#if HAL_WITH_ESC_TELEM
        case AP_TemperatureSensor_Params::Source::ESC:
            t.temperature_cdeg = temperature * 100;
            update_telem_data(_params.source_id-1, t, AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE_EXTERNAL);
            break;

        case AP_TemperatureSensor_Params::Source::Motor:
            t.motor_temp_cdeg = temperature * 100;
            update_telem_data(_params.source_id-1, t, AP_ESC_Telem_Backend::TelemetryType::MOTOR_TEMPERATURE_EXTERNAL);
            break;
#endif

#if AP_BATTERY_ENABLED
        case AP_TemperatureSensor_Params::Source::Battery_Index:
            AP::battery().set_temperature(temperature, _params.source_id-1);
            break;
        case AP_TemperatureSensor_Params::Source::Battery_ID_SerialNumber:
            AP::battery().set_temperature_by_serial_number(temperature, _params.source_id);
            break;
#endif

        case AP_TemperatureSensor_Params::Source::None:
        case AP_TemperatureSensor_Params::Source::Pitot_tube:
        default:
            break;
    }

}

#endif // AP_TEMPERATURE_SENSOR_ENABLED
