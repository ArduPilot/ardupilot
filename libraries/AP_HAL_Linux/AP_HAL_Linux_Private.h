
#ifndef __AP_HAL_LINUX_PRIVATE_H__
#define __AP_HAL_LINUX_PRIVATE_H__

/* Umbrella header for all private headers of the AP_HAL_Linux module.
 * Only import this header from inside AP_HAL_Linux
 */

#include "UARTDriver.h"
#include "SPIUARTDriver.h"
#include "RPIOUARTDriver.h"
#include "I2CDriver.h"
#include "SPIDriver.h"
#include "AnalogIn.h"
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2
#include "AnalogIn_ErleBrain2.h"
#else
#include "AnalogIn_Navio.h"
#endif
#include "RaspilotAnalogIn.h"
#include "Storage.h"
#include "GPIO.h"
#include "RCInput.h"
#include "RCInput_AioPRU.h"
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2
#include "RCInput_ErleBrain2.h"
#else
#include "RCInput_Navio.h"
#endif
#include "RCInput_UDP.h"
#include "RCInput_Raspilot.h"
#include "RCOutput_PRU.h"
#include "RCOutput_AioPRU.h"
#include "RCOutput_PCA9685.h"
#include "RCOutput_ZYNQ.h"
#include "RCOutput_Bebop.h"
#include "RCOutput_Raspilot.h"
#include "Semaphores.h"
#include "Scheduler.h"
#include "ToneAlarmDriver.h"
#include "Util.h"

#endif // __AP_HAL_LINUX_PRIVATE_H__

