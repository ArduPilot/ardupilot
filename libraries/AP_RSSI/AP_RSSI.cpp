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

#include <AP_RSSI/AP_RSSI.h>
#include <GCS_MAVLink/GCS.h>
#include <RC_Channel/RC_Channel.h>

#include <utility>

extern const AP_HAL::HAL& hal;

#ifndef BOARD_RSSI_DEFAULT
#define BOARD_RSSI_DEFAULT 0
#endif

#ifndef BOARD_RSSI_ANA_PIN
#define BOARD_RSSI_ANA_PIN 0
#endif

#ifndef BOARD_RSSI_ANA_PIN_HIGH
#define BOARD_RSSI_ANA_PIN_HIGH 5.0f
#endif

const AP_Param::GroupInfo AP_RSSI::var_info[] = {

    // @Param: TYPE
    // @DisplayName: RSSI Type
    // @Description: Radio Receiver RSSI type. If your radio receiver supports RSSI of some kind, set it here, then set its associated RSSI_XXXXX parameters, if any.
    // @Values: 0:Disabled,1:AnalogPin,2:RCChannelPwmValue,3:ReceiverProtocol,4:PWMInputPin
    // @User: Standard
    AP_GROUPINFO_FLAGS("TYPE", 0, AP_RSSI, rssi_type,  BOARD_RSSI_DEFAULT, AP_PARAM_FLAG_ENABLE),

    // @Param: ANA_PIN
    // @DisplayName: Receiver RSSI sensing pin
    // @Description: Pin used to read the RSSI voltage or PWM value
    // @Values: 11:Pixracer,13:Pixhawk ADC4,14:Pixhawk ADC3,15:Pixhawk ADC6,15:Pixhawk2 ADC,50:PixhawkAUX1,51:PixhawkAUX2,52:PixhawkAUX3,53:PixhawkAUX4,54:PixhawkAUX5,55:PixhawkAUX6,103:Pixhawk SBUS
    // @User: Standard
    AP_GROUPINFO("ANA_PIN", 1, AP_RSSI, rssi_analog_pin,  BOARD_RSSI_ANA_PIN),

    // @Param: PIN_LOW
    // @DisplayName: RSSI pin's lowest voltage
    // @Description: RSSI pin's voltage received on the RSSI_ANA_PIN when the signal strength is the weakest. Some radio receivers put out inverted values so this value may be higher than RSSI_PIN_HIGH
    // @Units: V
    // @Increment: 0.01
    // @Range: 0 5.0
    // @User: Standard
    AP_GROUPINFO("PIN_LOW", 2, AP_RSSI, rssi_analog_pin_range_low, 0.0f),

    // @Param: PIN_HIGH
    // @DisplayName: RSSI pin's highest voltage
    // @Description: RSSI pin's voltage received on the RSSI_ANA_PIN when the signal strength is the strongest. Some radio receivers put out inverted values so this value may be lower than RSSI_PIN_LOW
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
    if (_s_instance) {
        AP_HAL::panic("Too many RSSI sensors");
    }
    _s_instance = this;
}

// destructor
AP_RSSI::~AP_RSSI(void)
{       
}

/*
 * Get the AP_RSSI singleton
 */
AP_RSSI *AP_RSSI::get_instance()
{
    return _s_instance;
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
    // Default to 0 RSSI
    float receiver_rssi = 0.0f;  

    switch (rssi_type) {
        case RssiType::RSSI_DISABLED:
            receiver_rssi = 0.0f;
            break;
        case RssiType::RSSI_ANALOG_PIN:
            receiver_rssi = read_pin_rssi();
            break;
        case RssiType::RSSI_RC_CHANNEL_VALUE:
            receiver_rssi = read_channel_rssi();
            break;
        case RssiType::RSSI_RECEIVER: {
            int16_t rssi = RC_Channels::get_receiver_rssi();
            if (rssi != -1) {
                receiver_rssi = rssi / 255.0;
            }
            break;
        }
        case RssiType::RSSI_PWM_PIN:
            receiver_rssi = read_pwm_pin_rssi();
            break;
        default :   
            receiver_rssi = 0.0f;
            break;
    }    
                  
    return receiver_rssi;
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
    rssi_analog_source->set_pin(rssi_analog_pin);
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

void AP_RSSI::check_pwm_pin_rssi()
{
    if (rssi_analog_pin == pwm_state.last_rssi_analog_pin) {
        return;
    }

    // detach last one
    if (pwm_state.last_rssi_analog_pin) {
        if (!hal.gpio->detach_interrupt(pwm_state.last_rssi_analog_pin)) {
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "RSSI: Failed to detach from pin %u",
                            pwm_state.last_rssi_analog_pin);
            // ignore this failure or the user may be stuck
        }
    }

    pwm_state.last_rssi_analog_pin = rssi_analog_pin;

    if (!rssi_analog_pin) {
        // don't need to install handler
        return;
    }

    // install interrupt handler on rising and falling edge
    hal.gpio->pinMode(rssi_analog_pin, HAL_GPIO_INPUT);
    if (!hal.gpio->attach_interrupt(
            rssi_analog_pin,
            FUNCTOR_BIND_MEMBER(&AP_RSSI::irq_handler,
                                void,
                                uint8_t,
                                bool,
                                uint32_t),
            AP_HAL::GPIO::INTERRUPT_BOTH)) {
        // failed to attach interrupt
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "RSSI: Failed to attach to pin %u",
                        rssi_analog_pin);
        return;
    }
}

// read the PWM value from a pin
float AP_RSSI::read_pwm_pin_rssi()
{
    // check if pin has changed and configure interrupt handlers if required:
    check_pwm_pin_rssi();

    if (!pwm_state.last_rssi_analog_pin) {
        // disabled (either by configuration or failure to attach interrupt)
        return 0.0f;
    }

    // disable interrupts and grab state
    void *irqstate = hal.scheduler->disable_interrupts_save();
    const uint32_t irq_value_us = pwm_state.irq_value_us;
    pwm_state.irq_value_us = 0;
    hal.scheduler->restore_interrupts(irqstate);

    const uint32_t now = AP_HAL::millis();
    if (irq_value_us == 0) {
        // no reading; check for timeout:
        if (now - pwm_state.last_reading_ms > 1000) {
            // no reading for a second - something is broken
            pwm_state.rssi_value = 0.0f;
        }
    } else {
        // a new reading - convert pwm value to rssi value
        pwm_state.rssi_value = scale_and_constrain_float_rssi(irq_value_us, rssi_channel_low_pwm_value, rssi_channel_high_pwm_value);
        pwm_state.last_reading_ms = now;
    }

    return pwm_state.rssi_value;
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

// interrupt handler for reading pwm value
void AP_RSSI::irq_handler(uint8_t pin, bool pin_high, uint32_t timestamp_us)
{
    if (pin_high) {
        pwm_state.pulse_start_us = timestamp_us;
    } else {
        if (pwm_state.pulse_start_us != 0) {
            pwm_state.irq_value_us = timestamp_us - pwm_state.pulse_start_us;
            pwm_state.pulse_start_us = 0;
        }
    }
}

AP_RSSI *AP_RSSI::_s_instance = nullptr;

namespace AP {

AP_RSSI *rssi()
{
    return AP_RSSI::get_instance();
}

};
