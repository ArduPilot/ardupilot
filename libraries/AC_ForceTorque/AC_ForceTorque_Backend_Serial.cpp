#include <AP_HAL/AP_HAL.h>
#include "AC_ForceTorque_Backend_Serial.h"
#include <AP_SerialManager/AP_SerialManager.h>

#include <ctype.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the ForceTorque. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the ForceTorque
*/
AC_ForceTorque_Backend_Serial::AC_ForceTorque_Backend_Serial(
    ForceTorque::ForceTorque_State &_state,
    AC_ForceTorque_Params &_params) :
    AC_ForceTorque_Backend(_state, _params)
{

}

void AC_ForceTorque_Backend_Serial::init_serial(uint8_t serial_instance)
{
    uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_ForceTorque, serial_instance);
    if (uart != nullptr) {
        uart->begin(initial_baudrate(serial_instance), rx_bufsize(), tx_bufsize());
    }
}

uint32_t AC_ForceTorque_Backend_Serial::initial_baudrate(const uint8_t serial_instance) const
{
    return AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_ForceTorque, serial_instance);
}

/*
   detect if a Serial ForceTorque is connected. We'll detect by simply
   checking for SerialManager configuration
*/
bool AC_ForceTorque_Backend_Serial::detect(uint8_t serial_instance)
{
    return AP::serialmanager().have_serial(AP_SerialManager::SerialProtocol_ForceTorque, serial_instance);
}


/*
   update the state of the sensor
*/
void AC_ForceTorque_Backend_Serial::update(void)
{
    if (get_reading(state.force_N, state.torque_Nm,state.force_N2, state.torque_Nm2)) {
        // update range_valid state based on distance measured
        state.last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > read_timeout_ms()) {
        set_status(ForceTorque::Status::NoData);
    }
}