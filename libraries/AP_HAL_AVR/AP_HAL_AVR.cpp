
#include <AP_HAL.h>
#include "AP_HAL_AVR.h"

/* Include AVR-specific implementations of the HAL classes */
#include "HAL_AVR.h"
#include "UARTDriver.h"
#include "I2CDriver.h"
#include "SPIDriver.h"
#include "AnalogIn.h"
#include "Storage.h"
#include "Log.h"
#include "GPIO.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "Scheduler.h"

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
static ArduinoGPIO      arduinoGPIO;
static APM1RCInput      apm1RCInput;
static APM2RCInput      apm2RCInput;
static APM1RCOutput     apm1RCOutput;
static APM2RCOutput     apm2RCOutput;
static ArduinoScheduler arduinoScheduler;

const HAL_AVR AP_HAL_AVR_APM1(
        (UARTDriver*) &avrUart0Driver,
        (UARTDriver*) &avrUart1Driver,
        (UARTDriver*) &avrUart2Driver,
        (UARTDriver*) &avrUart3Driver,
        &avrI2CDriver,
        &arduinoSPIDriver,
        &arduinoAnalogIn,
        &avrEEPROMStorage,
        &apm1DataFlashLog,
        (BetterStream*) &avrUart0Driver,
        &arduinoGPIO,
        &apm1RCInput,
        &apm1RCOutput,
        &arduinoScheduler );

const HAL_AVR AP_HAL_AVR_APM2(
        (UARTDriver*) &avrUart0Driver,
        (UARTDriver*) &avrUart1Driver,
        (UARTDriver*) &avrUart2Driver,
        (UARTDriver*) &avrUart3Driver,
        &avrI2CDriver,
        &arduinoSPIDriver,
        &arduinoAnalogIn,
        &avrEEPROMStorage,
        &apm2DataFlashLog,
        (BetterStream *) &avrUart0Driver,
        &arduinoGPIO,
        &apm2RCInput,
        &apm2RCOutput,
        &arduinoScheduler );

