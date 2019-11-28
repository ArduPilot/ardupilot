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

class AP_RSSI
{
public:
    enum RssiType {
        RSSI_DISABLED           = 0,
        RSSI_ANALOG_PIN         = 1,
        RSSI_RC_CHANNEL_VALUE   = 2,
        RSSI_RECEIVER           = 3,
        RSSI_PWM_PIN            = 4
    };

    AP_RSSI();

    /* Do not allow copies */
    AP_RSSI(const AP_RSSI &other) = delete;
    AP_RSSI &operator=(const AP_RSSI&) = delete;

    // destructor
    ~AP_RSSI(void);

    static AP_RSSI *get_instance();

    // Initialize the rssi object and prepare it for use
    void init();

    // return true if rssi reading is enabled
    bool enabled() const { return rssi_type != RSSI_DISABLED; }

    // Read the receiver RSSI value as a float 0.0f - 1.0f.
    // 0.0 represents weakest signal, 1.0 represents maximum signal.
    float read_receiver_rssi();

    // Read the receiver RSSI value as an 8-bit integer
    // 0 represents weakest signal, 255 represents maximum signal.
    uint8_t read_receiver_rssi_uint8();   

    // parameter block
    static const struct AP_Param::GroupInfo var_info[];

private:

    static AP_RSSI *_s_instance;

    // RSSI parameters
    AP_Int8         rssi_type;                              // Type of RSSI being used
    AP_Int8         rssi_analog_pin;                        // Analog pin RSSI value found on
    AP_Float        rssi_analog_pin_range_low;              // Voltage value for weakest rssi signal
    AP_Float        rssi_analog_pin_range_high;             // Voltage value for strongest rssi signal
    AP_Int8         rssi_channel;                           // allows rssi to be read from given channel as PWM value
    AP_Int16        rssi_channel_low_pwm_value;             // PWM value for weakest rssi signal
    AP_Int16        rssi_channel_high_pwm_value;            // PWM value for strongest rssi signal

    // Analog Inputs
    // a pin for reading the receiver RSSI voltage. 
    AP_HAL::AnalogSource *rssi_analog_source;

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
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    uint32_t get_gpio(uint8_t pin_number) const;
    static int irq_handler(int irq, void *context);
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    static void irq_handler_chibios(void);
#endif

    static void irq_handler_calc_pwm(bool pin_high);
};

namespace AP {
    AP_RSSI *rssi();
};
