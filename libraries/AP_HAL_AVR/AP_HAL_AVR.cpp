
#include <AP_HAL.h>
#include "AP_HAL_AVR.h"

/* Include AVR-specific implementations of the HAL classes */
#include "HAL_AVR.h"
#include "UARTDriver.h"
#include "I2CDriver.h"
#include "SPIDriver.h"
#include "AnalogIn.h"
#include "Storage.h"
#include "Dataflash.h"
#include "Console.h"
#include "GPIO.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "Scheduler.h"

using namespace AP_HAL;
using namespace AP_HAL_AVR;

AVRUARTDriverISRs(0);
AVRUARTDriverISRs(1);
AVRUARTDriverISRs(2);
AVRUARTDriverISRs(3);

static AVRUARTDriverInstance(avrUart0Driver, 0);
static AVRUARTDriverInstance(avrUart1Driver, 1);
static AVRUARTDriverInstance(avrUart2Driver, 2);
static AVRUARTDriverInstance(avrUart3Driver, 3);
static EmptyUARTDriver  emptyUartDriver;

static AVRI2CDriver     avrI2CDriver;
static ArduinoSPIDriver arduinoSPIDriver;
static APM1SPIDriver    apm1SPIDriver;
static AVRAnalogIn      avrAnalogIn;
static AVREEPROMStorage avrEEPROMStorage;
static APM1Dataflash    apm1Dataflash;
static APM2Dataflash    apm2Dataflash;
static AVRConsoleDriver consoleDriver;
static AVRGPIO          avrGPIO;
static APM1RCInput      apm1RCInput;
static APM2RCInput      apm2RCInput;
static APM1RCOutput     apm1RCOutput;
static APM2RCOutput     apm2RCOutput;
static AVRScheduler     avrScheduler;

const HAL_AVR AP_HAL_AVR_APM1(
        (UARTDriver*) &avrUart0Driver,
        (UARTDriver*) &avrUart1Driver,
        (UARTDriver*) &emptyUartDriver,
        (UARTDriver*) &avrUart3Driver,
        &avrI2CDriver,
        &arduinoSPIDriver,
        &avrEEPROMStorage,
        &apm1Dataflash,
        &consoleDriver,
        &avrGPIO,
        &apm1RCInput,
        &apm1RCOutput,
        &avrScheduler );

const HAL_AVR AP_HAL_AVR_APM2(
        (UARTDriver*) &avrUart0Driver,
        (UARTDriver*) &avrUart1Driver,
        (UARTDriver*) &avrUart2Driver,
        (UARTDriver*) &emptyUartDriver,
        &avrI2CDriver,
        &arduinoSPIDriver,
        &avrEEPROMStorage,
        &apm2Dataflash,
        &consoleDriver,
        &avrGPIO,
        &apm2RCInput,
        &apm2RCOutput,
        &avrScheduler );

