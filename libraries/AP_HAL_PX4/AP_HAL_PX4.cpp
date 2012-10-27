
#include <AP_HAL.h>
#include "AP_HAL_PX4.h"

#include "HAL_PX4.h"
#include "Console.h"

using namespace AP_HAL;
using namespace AP_HAL_PX4;

static PX4ConsoleDriver px4ConsoleDriver;

const HAL_PX4 AP_HAL_PX4_Instance(
        (UARTDriver*) NULL, 
        (UARTDriver*) NULL,
        (UARTDriver*) NULL,
        (UARTDriver*) NULL,
        (I2CDriver*) NULL,
        (SPIDriver*) NULL,
        (AnalogIn*) NULL,
        (Storage*) NULL,
        (Dataflash*) NULL,
        (ConsoleDriver*) &px4ConsoleDriver,
        (GPIO*) NULL,
        (RCInput*) NULL,
        (RCOutput*) NULL,
        (Scheduler*) NULL);
