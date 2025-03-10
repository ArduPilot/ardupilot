#include "AP_VertiqThrottleManager.h"
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

AP_VertiqThrottleManager::AP_VertiqThrottleManager(IFCIPackedMessage * transmission_message):
    _transmission_message(transmission_message)
{

}

void AP_VertiqThrottleManager::Init(uint8_t number_cvs_in_use)
{
    _transmission_message->num_cvs = number_cvs_in_use;
}

uint16_t AP_VertiqThrottleManager::NormalizedThrottleToCv(float normalized_value)
{
    //Take a value from [-1, 1] and place it from [0, 65535]
    //-1 represents the lowest PWM command (throttle down for example)
    //1 represents the highest PWM command (throttle maxed for example)

    //Put the [-1, 1] value to [0, 1]
    float unconstrained_normalized_zero_to_one = (normalized_value + 1.0) * 0.5;

    //Constrain it to our bounds
    float constrained_zero_to_one = constrain_float(unconstrained_normalized_zero_to_one, 0, 1);

    //Finally we can spit back what percentage of 65535 we want
    return (uint16_t)(constrained_zero_to_one * 65535);
}

void AP_VertiqThrottleManager::UpdateThrottleOutputs()
{

    //Go through and grab all of our control values from the generator
    for (uint8_t cv = 0; cv < _transmission_message->num_cvs; cv++) {
        SRV_Channel * output_channel = SRV_Channels::srv_channel(cv);

        if (output_channel != nullptr) {
            //This is a value [-1, 1] where the lowest output PWM maps to -1
            float normalized_output = hal.rcout->scale_esc_to_unity(output_channel->get_output_pwm());

            _transmission_message->commands[cv] = NormalizedThrottleToCv(normalized_output);
        }
    }
}