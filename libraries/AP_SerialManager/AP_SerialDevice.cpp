#include "AP_SerialDevice.h"

#include "AP_SerialDevice_UART.h"

void AP_SerialDevice::begin(uint32_t baud) {
    AP_SerialDevice_UART *dev_uart = get_serialdevice_uart();
    if (dev_uart != nullptr) {
        dev_uart->set_baud(baud);
    }
    begin();
}
