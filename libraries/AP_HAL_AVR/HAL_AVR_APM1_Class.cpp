
#include <AP_HAL.h>

/* To save linker space, we need to make sure the HAL_AVR_APM1 class
 * is built iff we are building for HAL_BOARD_APM1. These defines must
 * wrap the whole HAL_AVR_APM1 class declaration and definition. */
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1

#include <AP_HAL_AVR.h>
#include "AP_HAL_AVR_private.h"
#include "HAL_AVR_APM1_Class.h"

using namespace AP_HAL;
using namespace AP_HAL_AVR;

AVRUARTDriverISRs(0);
AVRUARTDriverISRs(1);
AVRUARTDriverISRs(3);

AVRUARTDriverInstance(avrUart0Driver, 0);
AVRUARTDriverInstance(avrUart1Driver, 1);
AVRUARTDriverInstance(avrUart3Driver, 3);

static AVRSemaphore     i2cSemaphore;
static AVRI2CDriver     avrI2CDriver(&i2cSemaphore);
static APM1SPIDeviceManager apm1SPIDriver;
static AVRAnalogIn      avrAnalogIn;
static AVREEPROMStorage avrEEPROMStorage;
static AVRGPIO          avrGPIO;
static APM1RCInput      apm1RCInput;
static APM1RCOutput     apm1RCOutput;
static AVRScheduler     avrScheduler;
static AVRUtil          avrUtil;

static ISRRegistry isrRegistry;

/* On APM1 the physical UART2 is used for SPI. */
HAL_AVR_APM1::HAL_AVR_APM1() :
    AP_HAL::HAL(
        &avrUart0Driver, /* phys UART0 -> uartA */
        &avrUart1Driver, /* phys UART1 -> uartB */
        &avrUart3Driver, /* phys UART3 -> uartC */
        NULL, /* no uartD */
        &avrI2CDriver,
        &apm1SPIDriver,
        &avrAnalogIn,
        &avrEEPROMStorage,
        &avrUart0Driver,
        &avrGPIO,
        &apm1RCInput,
        &apm1RCOutput,
        &avrScheduler,
        &avrUtil )
{}

void HAL_AVR_APM1::init(int argc, char * const argv[]) const {

    scheduler->init((void*)&isrRegistry);
   
    /* uartA is the serial port used for the console, so lets make sure
     * it is initialized at boot */
    uartA->begin(115200);
    /* The AVR RCInput drivers take an AP_HAL_AVR::ISRRegistry*
     * as the init argument */
    rcin->init((void*)&isrRegistry);
    rcout->init(NULL);
    spi->init(NULL);
    i2c->begin();
    i2c->setTimeout(100);
    analogin->init(NULL);

    /* Enable the pullups on the RX pins of the 3 UARTs This is important when
     * the RX line is high-Z: capacitive coupling between input and output pins
     * can cause bytes written to show up as an input. Occasionally this causes
     * us to detect a phantom GPS by seeing our own outgoing config message.
     * PE0 : RX0 (uartA)
     * PD2 : RX1 (uartB)
     * PJ0 : RX3 (uartC)
     */

    PORTE |= _BV(0);
    PORTD |= _BV(2);
    PORTJ |= _BV(0);
};

const HAL_AVR_APM1 AP_HAL_AVR_APM1;

#endif // CONFIG_HAL_BOARD == HAL_BOARD_APM1
