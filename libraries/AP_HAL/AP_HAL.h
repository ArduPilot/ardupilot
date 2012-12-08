
#ifndef __AP_HAL_H__
#define __AP_HAL_H__

#include "AP_HAL_Namespace.h"

/* HAL Module Classes (all pure virtual) */
#include "UARTDriver.h"
#include "I2CDriver.h"
#include "SPIDriver.h"
#include "AnalogIn.h"
#include "Storage.h"
#include "Console.h"
#include "GPIO.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "Scheduler.h"
#include "Semaphore.h"

#include "utility/Print.h"
#include "utility/Stream.h"
#include "utility/BetterStream.h"

/* HAL Class definition */
#include "HAL.h"

#endif // __AP_HAL_H__

