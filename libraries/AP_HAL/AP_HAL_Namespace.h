
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
    class Console;
    class GPIO;
    class PPMInput;
    class PWMOutput;

    /* AVR / Arduino Implementations for APM1 and APM2 */

    class AVRUARTDriver;
    class AVRI2CDriver;
    class ArduinoSPIDriver;
    class ArduinoAnalogIn;
    class AVREEPROMStorage;
    class DataFlashAPM1Log;
    class DataFlashAPM2Log;
    class AVRUARTConsole;
    class ArduinoGPIO;
    class APM1PPMInput;
    class APM2PPMInput;
    class APM1PWMOutput;
    class APM2PWMOutput;
}


#endif // __AP_HAL_NAMESPACE_H__
