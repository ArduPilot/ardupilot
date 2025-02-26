#ifndef AP_VERTIQ_THROTTLE_MANAGER_HPP
#define AP_VERTIQ_THROTTLE_MANAGER_HPP

#pragma GCC diagnostic ignored "-Wsuggest-override"

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

#include <generic_interface.hpp>
#include <iquart_flight_controller_interface_client.hpp>
#include <propeller_motor_control_client.hpp>

/**
 * @brief The AP_VertiqThrottleManager is responsible for filling in all Control Values in transmitted IFCI packets. It communicates with the flight controller's
 * SRV_Channels in order to get the PWM output for each channel, converts the value to an IFCI CV, and fills in the IFCI packet's CVs with the correct values.
 */
class AP_VertiqThrottleManager
{

public:

    /**
     * @brief Create a new AP_VertiqThrottleManager object
     *
     * @param transmission_message A pointer to the IFCIPackedMessage whose CVs we will fill
     */
    AP_VertiqThrottleManager(IFCIPackedMessage * transmission_message);

    /**
     * @brief Initialize the throttle manager
     *
     * @param number_cvs_in_use The user set number of control values in use
     *
     */
    void Init(uint8_t number_cvs_in_use);

    /**
     * @brief Updates the CVs to transmit as part of the _transmission_message
     */
    void UpdateThrottleOutputs();

private:
    IFCIPackedMessage * _transmission_message;

    /**
     * @brief Takes in a PWM value [-1, 1] from the flight controller, and converts it into a uint16_t control value that Vertiq motors understand
     *
     * @param normalized_value The value [-1, 1] received from the flight controller through hal.rcout->scale_esc_to_unity(output_channel->get_output_pwm())
     * @return the control value (uint16) equivalent of the normalized_value (float [-1, 1])
     */
    uint16_t NormalizedThrottleToCv(float normalized_value);
};

#endif //AP_VERTIQ_THROTTLE_GENERATION_MANAGER_HPP