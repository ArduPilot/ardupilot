#ifndef AP_VERTIQ_CLIENT_MANAGER_HPP
#define AP_VERTIQ_CLIENT_MANAGER_HPP

#pragma GCC diagnostic ignored "-Wsuggest-override"

#include <client_communication.hpp>

/**
 * @brief This class is responsible for managing any Vertiq clients that expect to receive responses. When IQUART data is received, the serial processor passes
 * the received packets to the client manager which then allows all registered clients to read and process the data in their own ways.
 */
class AP_VertiqClientManager
{
public:

    /**
     * @brief Create a new AP_VertiqClientManager object
     */
    AP_VertiqClientManager();

    /**
     * @brief Attempts to add a new client pointer to our list of managed clients
     *
     * @param new_client a pointer to the client that you want to add to the manager
     * @return true if successfully added, false otherwise
     */
    bool RegisterNewClient(ClientAbstract * new_client);

    /**
     * @brief Passes IQUART data to each of the managed clients so that they can read and use it if applicable
     *
     * @param data_received a pointer to the data
     * @param data_received_len the length of the data received
     * @return true if successfully added, false otherwise
     */
    void ProcessClientRx(uint8_t * data_received, uint8_t data_received_len);
private:
    static const uint8_t MAXIMUM_NUMBER_OF_CLIENTS = 10;
    ClientAbstract *_client_array[MAXIMUM_NUMBER_OF_CLIENTS];
    uint8_t _clients_in_use;
};

#endif //AP_VERTIQ_CLIENT_MANAGER_HPP