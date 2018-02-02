
#pragma once

#include <hal_types.h>

/* While not strictly required, names inside the F4Light namespace are prefixed
 * with F4Light for clarity. (Some of our users aren't familiar with all of the
 * C++ namespace rules.)
 */

namespace F4Light {
    class UARTDriver;
    class USBDriver;
    class SPIDeviceManager;
    class SPIDevice;
    class AnalogSource;
    class AnalogIn;
    class Storage;
    class GPIO;
    class DigitalSource;
    class RCInput;
    class RCOutput;
    class Semaphore;
    class Scheduler;
    class Util;
    class I2CDevice;
    class I2CDeviceManager;
    class _parser;
    class PPM_parser;
    class DSM_parser;
    class SBUS_parser;
    class NRF_parser;
    class SerialDriver;
    class UART_OSD;
    class UART_PPM;
    class MassStorage;
}



