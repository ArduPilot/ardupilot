#include <AP_HAL/AP_HAL.h>
#include "AP_Vtx_SerialBackend.h"
#include <AP_SerialManager/AP_SerialManager.h>


extern const AP_HAL::HAL& hal;
/*
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/

AP_Vtx_SerialBackend::AP_Vtx_SerialBackend(
    AP_Vtx_SerialBackend::Vtx_Serial_State &_state,
    AP_Vtx_Params &_params,
    uint8_t serial_instance,
    AP_SerialManager::SerialProtocol protocol
) :
    AP_Vtx_Backend(_state,_params)
{
    uart = AP::serialmanager().find_serial(protocol, serial_instance);
    if (uart != nullptr) {
        uart->begin(initial_baudrate(serial_instance), rx_bufsize(), tx_bufsize());
    }
    state=&_state;
}
/*
   detect if a Vtx is connected. We'll detect by simply
   checking for SerialManager configuration
*/
bool AP_Vtx_SerialBackend::detect(AP_SerialManager::SerialProtocol protocol, uint8_t serial_instance)
{
    return AP::serialmanager().find_serial(protocol, serial_instance) != nullptr;
}



void AP_Vtx_SerialBackend::set_io_status(RqStatus status)
{
    rq_status=status;
}
