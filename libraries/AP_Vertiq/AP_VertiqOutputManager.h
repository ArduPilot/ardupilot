#ifndef AP_VERTIQ_OUTPUT_MANAGER
#define AP_VERTIQ_OUTPUT_MANAGER

#pragma GCC diagnostic ignored "-Wsuggest-override"

#include <iquart_flight_controller_interface_client.hpp>

#include "AP_VertiqTelemetryManager.h"
#include "AP_VertiqThrottleManager.h"
#include "AP_VertiqClientManager.h"

/**
 * @brief The AP_VertiqOutputManger is responsible for creating the IFCI packets that are broadcast to all connected Vertiq modules. Inside are the throttle manager
 * as well as the telemetry manager. Each fills in their own section of the IFCI packet (Control Values or telemetry byte).
 */
class AP_VertiqOutputManager
{

public:

    /**
     * @brief create a new AP_VertiqOutputManager object
     *
     * @param iquart_interface A pointer to the shared IQUART resource
     * @param client_manager A pointer to the client manager
     */
    AP_VertiqOutputManager(GenericInterface * iquart_interface, AP_VertiqClientManager * client_manager);

    /**
     * @brief Initialize the output manager by passing the user set telemetry bitmask to the telemetry manager and the number of CVs to the throttle manager
     *
     * @param telem_bitmask the user set telemetry bitmask
     * @param number_cvs_in_use the user set number of CVs in use
     */
    void Init(int32_t telem_bitmask, uint8_t number_cvs_in_use);

    /**
     * @brief Run at main loop, updates both the throttle and telemetry managers
     */
    void Update();

private:
    GenericInterface * _iquart_interface;

    AP_VertiqThrottleManager _throttle_manager;
    AP_VertiqTelemetryManager _telemetry_manager;

    IQUartFlightControllerInterfaceClient _ifci_control;

    IFCIPackedMessage _transmission_message;
    static const uint16_t MAX_IFCI_MESSAGE = 40; //Up to 16 2 byte commands, one telemetry byte, plus 7 IQUART added bytes
    uint8_t _output_message[MAX_IFCI_MESSAGE];
    uint8_t _output_len;
};

#endif //AP_VERTIQ_OUTPUT_MANAGER