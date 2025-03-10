#include "AP_VertiqSerialManager.h"

extern const AP_HAL::HAL& hal;

AP_VertiqSerialManager::AP_VertiqSerialManager(GenericInterface * iquart_interface, AP_VertiqClientManager * client_manager):
    _iquart_hardware_driver(nullptr),
    _iquart_interface(iquart_interface),
    _client_manager(client_manager)
{

}

void AP_VertiqSerialManager::initSerial()
{
    AP_SerialManager *system_serial_man = AP_SerialManager::get_singleton();

    //You didn't find the system serial manager
    if (!system_serial_man) {
        return;
    }

    _iquart_hardware_driver = system_serial_man->find_serial(AP_SerialManager::SerialProtocol_IQUART, 0);

    //no one is using us, give up
    if (_iquart_hardware_driver == nullptr) {
        return;
    }
}

void AP_VertiqSerialManager::ProcessSerial()
{
    ProcessSerialTx();
    ProcessSerialRx();
}

void AP_VertiqSerialManager::ProcessSerialTx()
{
    if (_iquart_hardware_driver != nullptr) {
        while (_iquart_interface->GetTxBytes(_tx_buf, _number_of_available_bytes)) {
            _iquart_hardware_driver->write(_tx_buf, _number_of_available_bytes);
        }
    }
}

void AP_VertiqSerialManager::ProcessSerialRx()
{
    if (_iquart_hardware_driver != nullptr) {
        _number_of_available_bytes = _iquart_hardware_driver->available();

        if (_number_of_available_bytes) {

            _number_of_available_bytes = _iquart_hardware_driver->read(_rx_buf, _number_of_available_bytes);
            _iquart_interface->SetRxBytes(_rx_buf, _number_of_available_bytes);

            uint8_t * data_ptr = _rx_buf;

            while (_iquart_interface->PeekPacket(&data_ptr, &_number_of_available_bytes) == 1) {
                _client_manager->ProcessClientRx(data_ptr, _number_of_available_bytes);
                _iquart_interface->DropPacket();
            }
        }
    }
}