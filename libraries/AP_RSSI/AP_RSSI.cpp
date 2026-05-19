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

#include "AP_RSSI_config.h"

#if AP_RSSI_ENABLED

#include <AP_RSSI/AP_RSSI.h>
#include <GCS_MAVLink/GCS.h>
#include <RC_Channel/RC_Channel.h>

#include <utility>

extern const AP_HAL::HAL& hal;

#ifndef BOARD_RSSI_DEFAULT
#define BOARD_RSSI_DEFAULT 0
#endif

#ifndef BOARD_RSSI_ANA_PIN
#define BOARD_RSSI_ANA_PIN -1
#endif

#ifndef BOARD_RSSI_ANA_PIN_HIGH
#define BOARD_RSSI_ANA_PIN_HIGH 5.0f
#endif

const AP_Param::GroupInfo AP_RSSI::var_info[] = {

    // @Param: TYPE
    // @DisplayName: RSSI Type
    // @Description: Radio Receiver RSSI type. If your radio receiver supports RSSI of some kind, set it here, then set its associated RSSI_XXXXX parameters, if any.
    // @Values: 0:Disabled,1:AnalogPin,2:RCChannelPwmValue,3:ReceiverProtocol,4:PWMInputPin,5:TelemetryRadioRSSI
    // @User: Standard
    AP_GROUPINFO_FLAGS("TYPE", 0, AP_RSSI, rssi_type,  BOARD_RSSI_DEFAULT, AP_PARAM_FLAG_ENABLE),

    // @Param: ANA_PIN
    // @DisplayName: Receiver RSSI sensing pin
    // @Description: Pin used to read the RSSI voltage or PWM value. Analog Airspeed ports can be used for Analog inputs (some autopilots provide others also), Non-IOMCU Servo/MotorOutputs can be used for PWM input when configured as "GPIOs". Values for some autopilots are given as examples. Search wiki for "Analog pins" for analog pin or "GPIOs", if PWM input type, to determine pin number.
    // @Values: 8:V5 Nano,11:Pixracer,13:Pixhawk ADC4,14:Pixhawk ADC3,15:Pixhawk ADC6/Pixhawk2 ADC,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6,103:Pixhawk SBUS
    // @Range: -1 127
    // @User: Standard
    AP_GROUPINFO("ANA_PIN", 1, AP_RSSI, rssi_analog_pin,  BOARD_RSSI_ANA_PIN),

    // @Param: PIN_LOW
    // @DisplayName: RSSI pin's lowest voltage
    // @Description: RSSI pin's voltage received on the RSSI_ANA_PIN when the signal strength is the weakest. Some radio receivers put out inverted values so this value may be higher than RSSI_PIN_HIGH. When using pin 103, the maximum value of the parameter is 3.3V.
    // @Units: V
    // @Increment: 0.01
    // @Range: 0 5.0
    // @User: Standard
    AP_GROUPINFO("PIN_LOW", 2, AP_RSSI, rssi_analog_pin_range_low, 0.0f),

    // @Param: PIN_HIGH
    // @DisplayName: RSSI pin's highest voltage
    // @Description: RSSI pin's voltage received on the RSSI_ANA_PIN when the signal strength is the strongest. Some radio receivers put out inverted values so this value may be lower than RSSI_PIN_LOW. When using pin 103, the maximum value of the parameter is 3.3V.
    // @Units: V
    // @Increment: 0.01
    // @Range: 0 5.0
    // @User: Standard
    AP_GROUPINFO("PIN_HIGH", 3, AP_RSSI, rssi_analog_pin_range_high, BOARD_RSSI_ANA_PIN_HIGH),

    // @Param: CHANNEL
    // @DisplayName: Receiver RSSI channel number
    // @Description: The channel number where RSSI will be output by the radio receiver (5 and above).
    // @Range: 0 16
    // @User: Standard
    AP_GROUPINFO("CHANNEL", 4, AP_RSSI, rssi_channel,  0),

    // @Param: CHAN_LOW
    // @DisplayName: RSSI PWM low value
    // @Description: PWM value that the radio receiver will put on the RSSI_CHANNEL or RSSI_ANA_PIN when the signal strength is the weakest. Some radio receivers output inverted values so this value may be lower than RSSI_CHAN_HIGH
    // @Units: PWM
    // @Range: 0 2000
    // @User: Standard
    AP_GROUPINFO("CHAN_LOW", 5, AP_RSSI, rssi_channel_low_pwm_value,  1000),

    // @Param: CHAN_HIGH
    // @DisplayName: Receiver RSSI PWM high value
    // @Description: PWM value that the radio receiver will put on the RSSI_CHANNEL or RSSI_ANA_PIN when the signal strength is the strongest. Some radio receivers output inverted values so this value may be higher than RSSI_CHAN_LOW
    // @Units: PWM
    // @Range: 0 2000
    // @User: Standard
    AP_GROUPINFO("CHAN_HIGH", 6, AP_RSSI, rssi_channel_high_pwm_value,  2000),

    AP_GROUPEND
};

// Public
// ------

// constructor
AP_RSSI::AP_RSSI()
{       
    AP_Param::setup_object_defaults(this, var_info);
    if (_singleton) {
        AP_HAL::panic("Too many RSSI sensors");
    }
    _singleton = this;
}

// destructor
AP_RSSI::~AP_RSSI(void)
{       
}

/*
 * Get the AP_RSSI singleton
 */
AP_RSSI *AP_RSSI::get_singleton()
{
    return _singleton;
}

// Initialize the rssi object and prepare it for use
void AP_RSSI::init()
{
    // a pin for reading the receiver RSSI voltage. The scaling by 0.25 
    // is to take the 0 to 1024 range down to an 8 bit range for MAVLink    
    rssi_analog_source = hal.analogin->channel(ANALOG_INPUT_NONE);    
}

// Read the receiver RSSI value as a float 0.0f - 1.0f.
// 0.0 represents weakest signal, 1.0 represents maximum signal.
float AP_RSSI::read_receiver_rssi()
{
    switch (RssiType(rssi_type.get())) {
        case RssiType::TYPE_DISABLED:
            return 0.0f;
        case RssiType::ANALOG_PIN:
            return read_pin_rssi();
        case RssiType::RC_CHANNEL_VALUE:
            return read_channel_rssi();
        case RssiType::RECEIVER: {
            int16_t rssi = RC_Channels::get_receiver_rssi();
            if (rssi != -1) {
                return rssi * (1/255.0);
            }
            return 0.0f;
        }
        case RssiType::PWM_PIN:
            return read_pwm_pin_rssi();
        case RssiType::TELEMETRY_RADIO_RSSI:
            return read_telemetry_radio_rssi();
    }
    // should never get to here
    return 0.0f;
}

// Only valid for RECEIVER type RSSI selections. Returns -1 if protocol does not provide link quality report.
float AP_RSSI::read_receiver_link_quality()
{
    if (RssiType(rssi_type.get()) == RssiType::RECEIVER) {
        return RC_Channels::get_receiver_link_quality();
    }
    return -1;
}

// Read the receiver RSSI value as an 8-bit integer
// 0 represents weakest signal, 255 represents maximum signal.
uint8_t AP_RSSI::read_receiver_rssi_uint8()
{
    return read_receiver_rssi() * 255; 
}

// Private
// -------

// read the RSSI value from an analog pin - returns float in range 0.0 to 1.0
float AP_RSSI::read_pin_rssi()
{
    if (!rssi_analog_source || !rssi_analog_source->set_pin(rssi_analog_pin)) {
        return 0;
    }
    float current_analog_voltage = rssi_analog_source->voltage_average();

    return scale_and_constrain_float_rssi(current_analog_voltage, rssi_analog_pin_range_low, rssi_analog_pin_range_high);
}

// read the RSSI value from a PWM value on a RC channel
float AP_RSSI::read_channel_rssi()
{
    RC_Channel *c = rc().channel(rssi_channel-1);
    if (c == nullptr) {
        return 0.0f;
    }
    uint16_t rssi_channel_value = c->get_radio_in();
    float channel_rssi = scale_and_constrain_float_rssi(rssi_channel_value, rssi_channel_low_pwm_value, rssi_channel_high_pwm_value);
    return channel_rssi;    
}

// read the PWM value from a pin
float AP_RSSI::read_pwm_pin_rssi()
{
    // check if pin has changed and configure interrupt handlers if required:
    if (!pwm_state.pwm_source.set_pin(rssi_analog_pin, "RSSI")) {
        // disabled (either by configuration or failure to attach interrupt)
        return 0.0f;
    }

    uint16_t pwm_us = pwm_state.pwm_source.get_pwm_us();

    const uint32_t now = AP_HAL::millis();
    if (pwm_us == 0) {
        // no reading; check for timeout:
        if (now - pwm_state.last_reading_ms > 1000) {
            // no reading for a second - something is broken
            pwm_state.rssi_value = 0.0f;
        }
    } else {
        // a new reading - convert pwm value to rssi value
        pwm_state.rssi_value = scale_and_constrain_float_rssi(pwm_us, rssi_channel_low_pwm_value, rssi_channel_high_pwm_value);
        pwm_state.last_reading_ms = now;
    }

    return pwm_state.rssi_value;
}

float AP_RSSI::read_telemetry_radio_rssi()
{
#if HAL_GCS_ENABLED
    return GCS_MAVLINK::telemetry_radio_rssi();
#else
    return 0;
#endif
}

// Scale and constrain a float rssi value to 0.0 to 1.0 range 
float AP_RSSI::scale_and_constrain_float_rssi(float current_rssi_value, float low_rssi_range, float high_rssi_range)
{    
    float rssi_value_range = fabsf(high_rssi_range - low_rssi_range);
    if (is_zero(rssi_value_range)) {
        // User range isn't meaningful, return 0 for RSSI (and avoid divide by zero)
        return 0.0f;   
    }
    // Note that user-supplied ranges may be inverted and we accommodate that here. 
    // (Some radio receivers put out inverted ranges for RSSI-type values).    
    bool range_is_inverted = (high_rssi_range < low_rssi_range);
    // Constrain to the possible range - values outside are clipped to ends 
    current_rssi_value = constrain_float(current_rssi_value, 
                                        range_is_inverted ? high_rssi_range : low_rssi_range, 
                                        range_is_inverted ? low_rssi_range : high_rssi_range);    

    if (range_is_inverted)
    {
        // Swap values so we can treat them as low->high uniformly in the code that follows
        current_rssi_value = high_rssi_range + fabsf(current_rssi_value - low_rssi_range);
        std::swap(low_rssi_range, high_rssi_range);        
    }

    // Scale the value down to a 0.0 - 1.0 range
    float rssi_value_scaled = (current_rssi_value - low_rssi_range) / rssi_value_range;
    // Make absolutely sure the value is clipped to the 0.0 - 1.0 range. This should handle things if the
    // value retrieved falls outside the user-supplied range.
    return constrain_float(rssi_value_scaled, 0.0f, 1.0f);
}

AP_RSSI *AP_RSSI::_singleton = nullptr;

namespace AP {

AP_RSSI *rssi()
{
    return AP_RSSI::get_singleton();
}

};

#endif  // AP_RSSI_ENABLED
