#ifndef AP_VERTIQ_TELEMETRY_MANAGER
#define AP_VERTIQ_TELEMETRY_MANAGER

#pragma GCC diagnostic ignored "-Wsuggest-override"

#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <iquart_flight_controller_interface_client.hpp>

#include "AP_VertiqClientManager.h"

/**
 * @brief The AP_VertiqTelemetryManager is responsible for generating the telemetry ID tail byte of each outgoing IFCI packet as well as processing any
 * received telemetry. Received telemtry is handed off to the flight controller's internal telemetry processing for logging/Mavlink/etc.
 */
class AP_VertiqTelemetryManager : public AP_ESC_Telem_Backend
{

public:

    /**
     * @brief create a new AP_VertiqTelemetryManager object
     *
     * @param transmission_message A pointer to the IFCIPackedMessage that we're going to append our telemetry byte to
     * @param client_manager A pointer to the system client manager
     */
    AP_VertiqTelemetryManager(IFCIPackedMessage * transmission_message, AP_VertiqClientManager * client_manager);

    /**
     * @brief initialize the telemetry manager
     *
     * @param telem_bitmask the user set bitmask that determines the module IDs to check for telemetry
     */
    void Init(int32_t telem_bitmask);

    /**
     * @brief Sets the correct value to set as the telemetry byte in the transmission_message
     */
    void UpdateTelemetryByte();

private:
    static const uint8_t MAX_SUPPORTED_TELEM_MODULE_ID = 31;
    static const uint8_t MAX_SUPPORTED_TELEM_IDS = NUM_SERVO_CHANNELS;
    static const uint32_t TELEM_RESPONSE_TIMEOUT_MS = 50;
    static const uint16_t IMPOSSIBLE_MODULE_ID = 255;

    bool _using_telemetry;

    uint8_t _module_ids_in_use[MAX_SUPPORTED_TELEM_IDS];
    uint8_t _number_telem_ids;
    uint8_t _current_telem_id_index;

    uint32_t _last_telem_request_time;

    IFCIPackedMessage * _transmission_message;

    AP_VertiqClientManager * _client_manager;
    IQUartFlightControllerInterfaceClient * _telemetry_ifci_clients[MAX_SUPPORTED_TELEM_IDS];
    IQUartFlightControllerInterfaceClient * _active_ifci_client;

    /**
     * @brief Processes the current _active_ifci_client to see if there is received telemetry for it
     *
     * @return if new data was received and processed, false otherwise
     */
    bool ProcessIquartTelemResponse();
};

#endif //AP_VERTIQ_TELEMETRY_MANAGER