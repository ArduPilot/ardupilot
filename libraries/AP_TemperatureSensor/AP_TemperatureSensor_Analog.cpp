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

#include "AP_TemperatureSensor_config.h"

#if AP_TEMPERATURE_SENSOR_ANALOG_ENABLED

#include "AP_TemperatureSensor_Analog.h"


extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo AP_TemperatureSensor_Analog::var_info[] = {

    // @Param: PIN
    // @DisplayName: Temperature sensor analog voltage sensing pin
    // @Description: Sets the analog input pin that should be used for temprature monitoring. Values for some autopilots are given as examples. Search wiki for "Analog pins".
    // @Values: -1:Disabled, 2:Pixhawk/Pixracer/Navio2/Pixhawk2_PM1, 5:Navigator, 13:Pixhawk2_PM2/CubeOrange_PM2, 14:CubeOrange, 16:Durandal, 100:PX4-v1
    // @Range: -1 127
    // @User: Standard
    AP_GROUPINFO("PIN", 1, AP_TemperatureSensor_Analog, _pin, -1),

    // @Param: A0
    // @DisplayName: Temperature sensor analog 0th polynomial coefficient
    // @Description: a0 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5
    AP_GROUPINFO("A0", 2, AP_TemperatureSensor_Analog, _a[0], 0),

    // @Param: A1
    // @DisplayName: Temperature sensor analog 1st polynomial coefficient
    // @Description: a1 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5
    AP_GROUPINFO("A1", 3, AP_TemperatureSensor_Analog, _a[1], 0),

    // @Param: A2
    // @DisplayName: Temperature sensor analog 2nd polynomial coefficient
    // @Description: a2 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5
    AP_GROUPINFO("A2", 4, AP_TemperatureSensor_Analog, _a[2], 0),

    // @Param: A3
    // @DisplayName: Temperature sensor analog 3rd polynomial coefficient
    // @Description: a3 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5
    AP_GROUPINFO("A3", 5, AP_TemperatureSensor_Analog, _a[3], 0),

    // @Param: A4
    // @DisplayName: Temperature sensor analog 4th polynomial coefficient
    // @Description: a4 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5
    AP_GROUPINFO("A4", 6, AP_TemperatureSensor_Analog, _a[4], 0),

    // @Param: A5
    // @DisplayName: Temperature sensor analog 5th polynomial coefficient
    // @Description: a5 in polynomial of form temperature in deg = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5
    AP_GROUPINFO("A5", 7, AP_TemperatureSensor_Analog, _a[5], 0),

    // CHECK/UPDATE INDEX TABLE IN AP_TemperatureSensor_Backend.cpp WHEN CHANGING OR ADDING PARAMETERS

    AP_GROUPEND
};

AP_TemperatureSensor_Analog::AP_TemperatureSensor_Analog(AP_TemperatureSensor &front,
                                                         AP_TemperatureSensor::TemperatureSensor_State &state,
                                                         AP_TemperatureSensor_Params &params) :
    AP_TemperatureSensor_Backend(front, state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
    _analog_source = hal.analogin->channel(_pin);
}

// Update function called at 5Hz
void AP_TemperatureSensor_Analog::update()
{
    if ((_analog_source == nullptr) || !_analog_source->set_pin(_pin)) {
        // Invalid pln
        return;
    }

    // Use ratiometric voltage, measured voltage is relative to supply
    const float voltage = _analog_source->voltage_average_ratiometric();

    // Evaluate polynomial
    // temperature (deg) = a0 + a1*voltage + a2*voltage^2 + a3*voltage^3 + a4*voltage^4 + a5*voltage^5
    float temp = 0.0;
    float poly = 1.0;
    for (uint8_t i = 0; i < ARRAY_SIZE(_a); i++) {
        temp += _a[i] * poly;
        poly *= voltage;
    }

    // update state
    set_temperature(temp);
}

#endif // AP_TEMPERATURE_SENSOR_ANALOG_ENABLED

