
#ifndef __AP_HAL_NAMESPACE_H__
#define __AP_HAL_NAMESPACE_H__

namespace AP_HAL {

    /* Toplevel pure virtual class Hal.*/
    class HAL;

    /* Toplevel class names for drivers: */
    class UARTDriver;
    class I2CDriver;
    class SPIDriver;
    class AnalogIn;
    class Storage;
    class Log;
    class GPIO;
    class RCInput;
    class RCOutput;
    class Scheduler;

    class EmptyUARTDriver;

    /* Utility Classes */
    class Print;
    class Stream;
    class BetterStream;
}


#endif // __AP_HAL_NAMESPACE_H__
