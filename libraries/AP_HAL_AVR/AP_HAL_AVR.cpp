
#include <AP_HAL.h>
#include "AP_HAL_AVR.h"

/* Include AVR-specific implementations of the HAL classes */
#include "UARTDriver.h"
#include "I2CDriver.h"
#include "SPIDriver.h"
#include "AnalogIn.h"
#include "Storage.h"
#include "Log.h"
#include "Console.h"
#include "GPIO.h"
#include "PPMInput.h"
#include "PWMOutput.h"

using namespace AP_HAL;
using namespace AP_HAL_AVR;

AVRUARTDriverISRs(0);
AVRUARTDriverISRs(1);
AVRUARTDriverISRs(3);

static AVRUARTDriverInstance(avrUart0Driver, 0);
static AVRUARTDriverInstance(avrUart1Driver, 1);
static EmptyUARTDriver  avrUart2Driver;
static AVRUARTDriverInstance(avrUart3Driver, 3);

static AVRI2CDriver     avrI2CDriver;
static ArduinoSPIDriver arduinoSPIDriver;
static ArduinoAnalogIn  arduinoAnalogIn;
static AVREEPROMStorage avrEEPROMStorage;
static DataFlashAPM1Log apm1DataFlashLog;
static DataFlashAPM2Log apm2DataFlashLog;
static AVRUARTConsole   avrUartConsole(&avrUart0Driver);
static ArduinoGPIO      arduinoGPIO;
static APM1PPMInput     apm1PPMInput;
static APM2PPMInput     apm2PPMInput;
static APM1PWMOutput    apm1PWMOutput;
static APM2PWMOutput    apm2PWMOutput;

const HAL AP_HAL_AVR_APM1(
        (UARTDriver*) &avrUart0Driver,
        (UARTDriver*) &avrUart1Driver,
        (UARTDriver*) &avrUart2Driver,
        (UARTDriver*) &avrUart3Driver,
        &avrI2CDriver,
        &arduinoSPIDriver,
        &arduinoAnalogIn,
        &avrEEPROMStorage,
        &apm1DataFlashLog,
        &avrUartConsole,
        &arduinoGPIO,
        &apm1PPMInput,
        &apm1PWMOutput );

const HAL AP_HAL_AVR_APM2(
        (UARTDriver*) &avrUart0Driver,
        (UARTDriver*) &avrUart1Driver,
        (UARTDriver*) &avrUart2Driver,
        (UARTDriver*) &avrUart3Driver,
        &avrI2CDriver,
        &arduinoSPIDriver,
        &arduinoAnalogIn,
        &avrEEPROMStorage,
        &apm2DataFlashLog,
        &avrUartConsole,
        &arduinoGPIO,
        &apm2PPMInput,
        &apm2PWMOutput );

