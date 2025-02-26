#include "AP_VertiqClientManager.h"

AP_VertiqClientManager::AP_VertiqClientManager():
    _clients_in_use(0)
{

}

bool AP_VertiqClientManager::RegisterNewClient(ClientAbstract * new_client)
{
    if (_clients_in_use < MAXIMUM_NUMBER_OF_CLIENTS) {
        _client_array[_clients_in_use] = new_client;
        _clients_in_use++;

        return true;
    }

    //Out of space
    return false;
}

void AP_VertiqClientManager::ProcessClientRx(uint8_t * data_received, uint8_t data_received_len)
{
    //Give everyone a chance to read the data
    for (uint8_t client = 0; client < _clients_in_use; client++) {
        _client_array[client]->ReadMsg(data_received, data_received_len);
    }
}