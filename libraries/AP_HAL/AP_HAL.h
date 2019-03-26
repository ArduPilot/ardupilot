#pragma once

#include <stdint.h>

#include "AP_HAL_Namespace.h"
#include "AP_HAL_Boards.h"
#include "AP_HAL_Macros.h"
#include "AP_HAL_Main.h"

/* HAL Module Classes (all pure virtual) */
#include "UARTDriver.h"
#include "AnalogIn.h"
#include "Storage.h"
#include "GPIO.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "Scheduler.h"
#include "Semaphores.h"
#include "Util.h"
#include "OpticalFlow.h"
#include "Flash.h"

#if HAL_WITH_UAVCAN
#include "CAN.h"
#endif

#include "utility/BetterStream.h"

/* HAL Class definition */
#include "HAL.h"

#include "system.h"
