
#include <AP_HAL.h>
#include <AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SMACCM

#include "HAL_SMACCM_Class.h"
#include "AP_HAL_SMACCM_Private.h"

using namespace SMACCM;

// XXX make sure these are assigned correctly
static SMACCMUARTDriver uartADriver(usart1);
static SMACCMUARTDriver uartBDriver(usart6);
static SMACCMUARTDriver uartCDriver(NULL);

static SMACCMI2CDriver  i2cDriver;
static SMACCMSPIDeviceManager spiDeviceManager;
static SMACCMAnalogIn analogIn;
static SMACCMStorage storageDriver;
static SMACCMConsoleDriver consoleDriver(&uartADriver);
static SMACCMGPIO gpioDriver;
static SMACCMRCInput rcinDriver;
static SMACCMRCOutput rcoutDriver;
static SMACCMScheduler schedulerInstance;
static SMACCMUtil utilInstance;

HAL_SMACCM::HAL_SMACCM() :
    AP_HAL::HAL(
      &uartADriver,
      &uartBDriver,
      &uartCDriver,
      &i2cDriver,
      &spiDeviceManager,
      &analogIn,
      &storageDriver,
      &consoleDriver,
      &gpioDriver,
      &rcinDriver,
      &rcoutDriver,
      &schedulerInstance,
      &utilInstance)
{
}

void HAL_SMACCM::init(int argc,char* const argv[]) const
{
  /* initialize all drivers and private members here.
   * up to the programmer to do this in the correct order.
   * Scheduler should likely come first. */
  scheduler->init(NULL);
  uartA->begin(115200);
  console->init(uartA);
  i2c->begin();
  spi->init(NULL);
  storage->init(NULL);
  rcin->init(NULL);
}

const HAL_SMACCM AP_HAL_SMACCM;

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SMACCM
