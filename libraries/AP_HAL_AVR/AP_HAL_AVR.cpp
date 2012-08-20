
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

static AVRUARTDriver    avrUart0Driver( 0 );
static AVRUARTDriver    avrUart1Driver( 1 );
static AVRUARTDriver    avrUart2Driver( 2 );
static AVRUARTDriver    avrUart3Driver( 3 );
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
        &avrUart0Driver,
        &avrUart1Driver,
        &avrUart2Driver,
        &avrUart3Driver,
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
        &avrUart0Driver,
        &avrUart1Driver,
        &avrUart2Driver,
        &avrUart3Driver,
        &avrI2CDriver,
        &arduinoSPIDriver,
        &arduinoAnalogIn,
        &avrEEPROMStorage,
        &apm2DataFlashLog,
        &avrUartConsole,
        &arduinoGPIO,
        &apm2PPMInput,
        &apm2PWMOutput );

