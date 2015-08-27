/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

extern const AP_HAL::HAL& hal;


const AP_Param::GroupInfo AP_RSSI::var_info[] PROGMEM = {
                                
    // @Param: RSSI_TYPE
    // @DisplayName: RSSI Type
    // @Description: Radio Receiver RSSI type. If your radio receiver supports RSSI of some kind, set it here, then set its associated RSSI_XXXXX parameters, if any.
    // @Values: 0:Disabled,1:AnalogPin,2:RCChannelPwmValue
    // @User: Standard
    AP_GROUPINFO("TYPE", 0, AP_RSSI, rssi_type,  0),
                
    // @Param: RSSI_ANA_PIN
    // @DisplayName: Receiver RSSI analog sensing pin
    // @Description: This selects an analog pin where the receiver RSSI voltage will be read.
    // @Values: 0:APM2 A0, 1:APM2 A1, 13:APM2 A13, 103:Pixhawk SBUS
    // @User: Standard
    AP_GROUPINFO("ANA_PIN", 1, AP_RSSI, rssi_analog_pin,  0),

    // @Param: RSSI_PIN_LOW
    // @DisplayName: Receiver RSSI voltage low
    // @Description: This is the voltage value that the radio receiver will put on the RSSI_PIN when the signal strength is the weakest. Since some radio receivers put out inverted values from what you might otherwise expect, this isn't necessarily a lower value than PIN_RANGE_HIGH. 
    // @Units: Volt
    // @Values: 0.0: 0V, 3.3:3.3V, 5.0:5V. Intermedidate values OK.
    // @User: Standard
    AP_GROUPINFO("PIN_LOW", 2, AP_RSSI, rssi_analog_pin_range_low, 0.0f),

    // @Param: RSSI_PIN_HIGH
    // @DisplayName: Receiver RSSI voltage low
    // @Description: This is the voltage value that the radio receiver will put on the RSSI_PIN when the signal strength is the weakest. Since some radio receivers put out inverted values from what you might otherwise expect, this isn't necessarily a higher value than PIN_RANGE_LOW. 
    // @Units: Volt
    // @Values: 0.0: 0V, 3.3:3.3V, 5.0:5V. Intermedidate values OK.
    // @User: Standard
    AP_GROUPINFO("PIN_HIGH", 3, AP_RSSI, rssi_analog_pin_range_high, 5.0f),
    
    // @Param: RSSI_CHANNEL
    // @DisplayName: Receiver RSSI channel number
    // @Description: The channel number where RSSI will be output by the radio receiver.
    // @Units: 
    // @Values: 0:Disabled,1:Channel1,2:Channel2,3:Channel3,4:Channel4,5:Channel5,6:Channel6,7:Channel7,8:Channel8
    // @User: Standard
    AP_GROUPINFO("CHANNEL", 4, AP_RSSI, rssi_channel,  0),
    
    // @Param: RSSI_CHAN_LOW
    // @DisplayName: Receiver RSSI PWM low value
    // @Description: This is the PWM value that the radio receiver will put on the RSSI_CHANNEL when the signal strength is the weakest. Since some radio receivers put out inverted values from what you might otherwise expect, this isn't necessarily a lower value than RSSI_CHAN_HIGH. 
    // @Units: Microseconds
    // @Range: 0 2000
    // @User: Standard
    AP_GROUPINFO("CHAN_LOW", 5, AP_RSSI, rssi_channel_low_pwm_value,  1000),
    
    // @Param: RSSI_CHAN_HIGH
    // @DisplayName: Receiver RSSI PWM high value
    // @Description: This is the PWM value that the radio receiver will put on the RSSI_CHANNEL when the signal strength is the strongest. Since some radio receivers put out inverted values from what you might otherwise expect, this isn't necessarily a higher value than RSSI_CHAN_LOW. 
    // @Units: Microseconds
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
}

// destructor
AP_RSSI::~AP_RSSI(void)
{       
}

// Initialize the rssi object and prepare it for use
void AP_RSSI::init()
{
    // a pin for reading the receiver RSSI voltage. The scaling by 0.25 
    // is to take the 0 to 1024 range down to an 8 bit range for MAVLink    
    rssi_analog_source = hal.analogin->channel(ANALOG_INPUT_NONE);    
}

// read the receiver RSSI value as an 8 bit number
uint8_t AP_RSSI::read_receiver_rssi()
{
    // Default to 0 RSSI
    uint8_t receiver_rssi = 0;  
            
    switch (rssi_type) {
        case RssiType::RSSI_DISABLED :
            receiver_rssi = 0;
            break;
        case RssiType::RSSI_ANALOG_PIN :
            receiver_rssi = read_pin_rssi();
            break;
        case RssiType::RSSI_RC_CHANNEL_VALUE :
            receiver_rssi = read_channel_rssi();
            break;
        default :   
            receiver_rssi = 0;      
    }    
         
    return receiver_rssi;
}

// Private
// -------
        
// read the RSSI value from an analog pin       
uint8_t AP_RSSI::read_pin_rssi()
{
    const int float_multipler = 1000;
        
    rssi_analog_source->set_pin(rssi_analog_pin);
    float current_analog_voltage = rssi_analog_source->voltage_average();
             
    // Voltage comes in as a float, but the common scaling/clipping/inverting routine takes ints so we convert.
    // (We lose some precision but millivolts shouldn't matter in this context.)
    int rssi_pin_current_voltage_int_value = convert_float_to_int_with_multiplier(current_analog_voltage, float_multipler); 
    int rssi_pin_low_voltage_int_value = convert_float_to_int_with_multiplier(rssi_analog_pin_range_low, float_multipler);
    int rssi_pin_high_voltage_int_value = convert_float_to_int_with_multiplier(rssi_analog_pin_range_high, float_multipler);    
        
    // Scale and constrain integer
    uint8_t voltage_rssi = scale_and_constrain_integer_rssi(rssi_pin_current_voltage_int_value, rssi_pin_low_voltage_int_value, rssi_pin_high_voltage_int_value);
    return voltage_rssi;    
}

// read the RSSI value from a PWM value on a RC channel
uint8_t AP_RSSI::read_channel_rssi()
{
    int rssi_channel_value = hal.rcin->read(rssi_channel-1);
    uint8_t channel_rssi = scale_and_constrain_integer_rssi(rssi_channel_value, rssi_channel_low_pwm_value, rssi_channel_high_pwm_value);
    return channel_rssi;    
}

// Convert a float to an int, using given multiplier first
int AP_RSSI::convert_float_to_int_with_multiplier(float value, int float_multipler)
{
    return static_cast<int>(round(value * float_multipler));    
}

// Scale and constrain an integer rssi value to the 0-255 value RSSI is expressed in
uint8_t AP_RSSI::scale_and_constrain_integer_rssi(int current_rssi_value, int low_rssi_range, int high_rssi_range)
{
    const int rssi_min = 0;
    const int rssi_max = 255;
    
    uint8_t constrained_value = 0;      
    
    // Note that user-supplied ranges may be inverted and we accommodate that here. 
    //(Some radio receivers put out inverted ranges for RSSI-type values).
    bool range_is_inverted = (high_rssi_range < low_rssi_range);
    // First, constrain to the possible range - values outside are clipped to ends 
    int rssi_value_clipped = constrain_int16(current_rssi_value, 
                                             range_is_inverted ? high_rssi_range : low_rssi_range, 
                                             range_is_inverted ? low_rssi_range : high_rssi_range);
    // Then scale to 0-255 RSSI normally is presented in.
    int rssi_value_range = abs(high_rssi_range - low_rssi_range);
    if (rssi_value_range == 0) {
        // User range isn't meaningful, return 0 for RSSI (and avoid divide by zero)
        constrained_value = 0;
    } else {
        float conversion_ratio = (float)rssi_max / (float)rssi_value_range;
        int rssi_offset = abs(rssi_value_clipped - low_rssi_range);
        constrained_value = (int)(rssi_offset * conversion_ratio);
        constrained_value = constrain_int16(constrained_value, rssi_min, rssi_max);
    }   
    return constrained_value;
}







