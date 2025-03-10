#ifndef AP_VERTIQ_SERIAL_MANAGER
#define AP_VERTIQ_SERIAL_MANAGER

#pragma GCC diagnostic ignored "-Wsuggest-override"

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include <generic_interface.hpp>

#include "AP_VertiqClientManager.h"

#define RX_AND_TX_BUFFER_SIZE 128

/**
 *
 * @brief This class is responsible for handling all serial transmission and receiving for control of Vertiq modules. This will interact
 * directly with the serial hardware as well as providing others with the ability to queue new IQUART messages.
 *
 */
class AP_VertiqSerialManager
{

public:

    /**
     * @brief create a new AP_VertiqSerialManager object
     *
     * @param iquart_interface A pointer to the shared IQUART resource
     * @param client_manager A pointer to the client manager
     */
    AP_VertiqSerialManager(GenericInterface * iquart_interface, AP_VertiqClientManager * client_manager);

    /**
     * @brief initialize all resources necessary for IQUART communication
     */
    void initSerial();

    /**
     * @brief handle UART transmission and reception
     */
    void ProcessSerial();

private:
    AP_HAL::UARTDriver * _iquart_hardware_driver;

    GenericInterface * _iquart_interface;

    AP_VertiqClientManager * _client_manager;

    //Buffers for RX and TX
    uint8_t _rx_buf[RX_AND_TX_BUFFER_SIZE];
    uint8_t _tx_buf[RX_AND_TX_BUFFER_SIZE];

    uint8_t _number_of_available_bytes;

    /**
    * @brief check to see if there is any data that we need to transmit over serial
    */
    void ProcessSerialTx();

    /**
    * @brief check to see if there is any received data to send through IQUART processing
    */
    void ProcessSerialRx();
};

#endif //AP_VERTIQ_SERIAL_MANAGER