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
#include <AP_Servo_Telem/AP_Servo_Telem.h>

/*
  All backends use the same parameter table and set of indices. Therefore, two
  backends must not use the same index. The list of used indices and
  corresponding backends is below.

    1:   AP_TemperatureSensor_DroneCAN.cpp, note there is a clash with analog, but due to different param types we get away with it
    1-7: AP_TemperatureSensor_Analog.cpp
    8-9: AP_TemperatureSensor_MAX31865.cpp

  Usage does not need to be contiguous. The maximum possible index is 63.
*/

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

#if HAL_LOGGING_ENABLED
void AP_TemperatureSensor_Backend::Log_Write_TEMP() const
{
    // @LoggerMessage: TEMP
    // @Description: Temperature Sensor Data
    // @Field: TimeUS: Time since system startup
    // @Field: Instance: temperature sensor instance
    // @Field: Temp: temperature
    AP::logger().Write("TEMP",
            "TimeUS,"     "Instance,"       "Temp" , // labels
            "s"               "#"           "O"    , // units
            "F"               "-"           "0"    , // multipliers
            "Q"               "B"           "f"    , // types
     AP_HAL::micros64(), _state.instance, _state.temperature);
}
#endif

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
#if AP_SERVO_TELEM_ENABLED
    AP_Servo_Telem *servo_telem;
    AP_Servo_Telem::TelemetryData servo_telem_data;
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
        case AP_TemperatureSensor_Params::Source::DroneCAN:
            // Label only, used by AP_Periph
            break;

#if AP_SERVO_TELEM_ENABLED
        case AP_TemperatureSensor_Params::Source::Servo_Motor:
            servo_telem = AP_Servo_Telem::get_singleton();
            if (servo_telem == nullptr) {
                break;
            }
            servo_telem_data.motor_temperature_cdeg = temperature * 100;
            servo_telem_data.present_types = AP_Servo_Telem::TelemetryData::Types::MOTOR_TEMP;
            servo_telem->update_telem_data(_params.source_id-1, servo_telem_data);
            break;

        case AP_TemperatureSensor_Params::Source::Servo_PCB:
            servo_telem = AP_Servo_Telem::get_singleton();
            if (servo_telem == nullptr) {
                break;
            }
            servo_telem_data.pcb_temperature_cdeg = temperature * 100;
            servo_telem_data.present_types = AP_Servo_Telem::TelemetryData::Types::PCB_TEMP;
            servo_telem->update_telem_data(_params.source_id-1, servo_telem_data);
            break;
#endif // AP_SERVO_TELEM_ENABLED

        case AP_TemperatureSensor_Params::Source::None:
        case AP_TemperatureSensor_Params::Source::Pitot_tube:
        default:
            break;
    }

}

#endif // AP_TEMPERATURE_SENSOR_ENABLED
