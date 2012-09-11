
#ifndef __AP_HAL_NAMESPACE_H__
#define __AP_HAL_NAMESPACE_H__

#include <stdint.h>

namespace AP_HAL {

    /* Toplevel pure virtual class Hal.*/
    class HAL;

    /* Toplevel class names for drivers: */
    class UARTDriver;
    class I2CDriver;
    class SPIDriver;
    class AnalogSource;
    class AnalogIn;
    class Storage;
    class Dataflash;
    class GPIO;
    class RCInput;
    class RCOutput;
    class Scheduler;

    class EmptyUARTDriver;

    /* Utility Classes */
    class Print;
    class Stream;
    class BetterStream;

    /* Typdefs for function pointers (Procedure, Timed Procedure) */
    typedef void(*Proc)(void);
    typedef void(*TimedProc)(uint32_t);
}


#endif // __AP_HAL_NAMESPACE_H__
