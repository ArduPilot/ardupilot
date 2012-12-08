
#ifndef __AP_HAL_NAMESPACE_H__
#define __AP_HAL_NAMESPACE_H__

#include <stdint.h>

namespace AP_HAL {

    /* Toplevel pure virtual class Hal.*/
    class HAL;

    /* Toplevel class names for drivers: */
    class UARTDriver;
    class I2CDriver;

    class SPIDeviceDriver;
    class SPIDeviceManager;

    class AnalogSource;
    class AnalogIn;
    class Storage;
    class ConsoleDriver;
    class DigitalSource;
    class GPIO;
    class RCInput;
    class RCOutput;
    class Scheduler;
    class Semaphore;

    class EmptyUARTDriver;

    /* Utility Classes */
    class Print;
    class Stream;
    class BetterStream;

    /* Typdefs for function pointers (Procedure, Timed Procedure) */
    typedef void(*Proc)(void);
    typedef void(*TimedProc)(uint32_t);

    /**
     * Global names for all of the existing SPI devices on all platforms.
     */

    enum SPIDevice {
        SPIDevice_Dataflash,
        SPIDevice_ADS7844,
        SPIDevice_MS5611,
        SPIDevice_MPU6000,
        SPIDevice_ADNS3080_SPI0,
        SPIDevice_ADNS3080_SPI3
    };

}


#endif // __AP_HAL_NAMESPACE_H__
