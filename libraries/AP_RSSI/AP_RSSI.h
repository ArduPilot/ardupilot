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

#include "AP_RSSI_config.h"

#if AP_RSSI_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class AP_RSSI
{
public:
    enum class RssiType {
        TYPE_DISABLED      = 0,
        ANALOG_PIN         = 1,
        RC_CHANNEL_VALUE   = 2,
        RECEIVER           = 3,
        PWM_PIN            = 4,
        TELEMETRY_RADIO_RSSI = 5,
    };

    AP_RSSI();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_RSSI);

    // destructor
    ~AP_RSSI(void);

    static AP_RSSI *get_singleton();

    // Initialize the rssi object and prepare it for use
    void init();

    // return true if rssi reading is enabled
    bool enabled() const { return RssiType(rssi_type.get()) != RssiType::TYPE_DISABLED; }

    // Read the receiver RSSI value as a float 0.0f - 1.0f.
    // 0.0 represents weakest signal, 1.0 represents maximum signal.
    float read_receiver_rssi();
    float read_receiver_link_quality();
    // Read the receiver RSSI value as an 8-bit integer
    // 0 represents weakest signal, 255 represents maximum signal.
    uint8_t read_receiver_rssi_uint8();   

    // parameter block
    static const struct AP_Param::GroupInfo var_info[];

private:

    static AP_RSSI *_singleton;

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
    struct PWMState {
        int8_t last_warn_pin; // last pin used for reading pwm (used to recognise change failure in pin assignment)
        uint32_t last_reading_ms;      // system time of last read (used for health reporting)
        float rssi_value;              // last calculated RSSI value
        // the following two members are updated by the interrupt handler
        AP_HAL::PWMSource pwm_source;
    } pwm_state;

    // read the RSSI value from an analog pin - returns float in range 0.0 to 1.0
    float read_pin_rssi();

    // read the RSSI value from a PWM value on a RC channel
    float read_channel_rssi();

    // read the PWM value from a pin
    float read_pwm_pin_rssi();

    // read the (RC) RSSI value from telemetry radio RSSI (e.g. rfd900x pass-through)
    float read_telemetry_radio_rssi();

    // Scale and constrain a float rssi value to 0.0 to 1.0 range
    float scale_and_constrain_float_rssi(float current_rssi_value, float low_rssi_range, float high_rssi_range);
};

namespace AP {
    AP_RSSI *rssi();
};

#endif  // AP_RSSI_ENABLED
