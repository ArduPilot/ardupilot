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
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class AP_WindVane
{

public:

    enum WindVaneType {
        WINDVANE_DISABLED   = 0,
        WINDVANE_ANALOG_PIN = 1,
        WINDVANE_PWM_PIN    = 2
    };

    AP_WindVane();

    /* Do not allow copies */
    AP_WindVane(const AP_WindVane &other) = delete;
    AP_WindVane &operator=(const AP_WindVane&) = delete;

    // destructor
    ~AP_WindVane(void);

    static AP_WindVane *get_instance();

    // Initialize the rssi object and prepare it for use
    void init();

    // return true if rssi reading is enabled
    bool enabled() const { return _type != WINDVANE_DISABLED; }

    // update wind vane
    void update();

    // get the wind direction in radians
    float get_wind_direction_rad();

    // parameter block
    static const struct AP_Param::GroupInfo var_info[];

private:

    static AP_WindVane *_s_instance;

    // RSSI parameters
    AP_Int8  _type;             // type of windvane being used
    AP_Int8  _pin;              // analog or pwm pin connected to sensor
    AP_Float _analog_volt_low;  // voltage when windvane is pointing at 0 degrees
    AP_Float _analog_volt_high; // voltage when windvane is pointing at 359 degrees
    AP_Int16 _pwm_low;          // PWM when windvane is pointing at 0 degrees
    AP_Int16 _pwm_value;        // PWM when windvane is pointing at 359 degrees

    // pin for reading analog voltage
    AP_HAL::AnalogSource *_analog_source;

    // PWM input
    static struct PWMState {
        uint32_t gpio;              // gpio pin used for reading pwm
        uint32_t last_gpio;         // last gpio pin used for reading pwm (used to recognise change in pin assignment)
        uint32_t value;             // last calculated pwm value
        uint32_t last_reading_ms;   // system time of last read (used for health reporting)
        uint64_t pulse_start_us;    // system time of start of pulse
    } pwm_state;

    // read the RSSI value from an analog pin - returns float in range 0.0 to 1.0
    float read_pin_rssi();

    // read the RSSI value from a PWM value on a RC channel
    float read_channel_rssi();

    // read the PWM value from a pin
    float read_pwm_pin_rssi();

    // Scale and constrain a float rssi value to 0.0 to 1.0 range
    float scale_and_constrain_float_rssi(float current_rssi_value, float low_rssi_range, float high_rssi_range);

    // PWM input handling
    uint32_t get_gpio(uint8_t pin_number) const;
    static int irq_handler(int irq, void *context);
};

namespace AP {
    AP_WindVane *rssi();
};
