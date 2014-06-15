
#include <AP_HAL.h>

/* To save linker space, we need to make sure the HAL_MPNG class
 * is built iff we are building for HAL_BOARD_MPNG. These defines must
 * wrap the whole HAL_MPNG class declaration and definition. */
#if CONFIG_HAL_BOARD == HAL_BOARD_MPNG

#include <AP_HAL_MPNG.h>
#include "AP_HAL_MPNG_private.h"
#include "HAL_MPNG_Class.h"

using namespace AP_HAL;
using namespace MPNG;

AVRUARTDriverISRs(0);
AVRUARTDriverISRs(1);
AVRUARTDriverISRs(2);
AVRUARTDriverISRs(3);

AVRUARTDriverInstance(avrUart0Driver, 0);
AVRUARTDriverInstance(avrUart1Driver, 1);
AVRUARTDriverInstance(avrUart2Driver, 2);
AVRUARTDriverInstance(avrUart3Driver, 3);

static AVRSemaphore     i2cSemaphore;
static AVRI2CDriver     avrI2CDriver(&i2cSemaphore);
static MPNGSPIDeviceManager mpngSPIDriver;
static AVRAnalogIn      avrAnalogIn;
static AVREEPROMStorage avrEEPROMStorage;
static AVRGPIO          avrGPIO;
static MPNGRCInput      mpngRCInput;
static MPNGRCOutput     mpngRCOutput;
static AVRScheduler     avrScheduler;
static AVRUtil          avrUtil;

static ISRRegistry isrRegistry;

HAL_MPNG::HAL_MPNG() :
    AP_HAL::HAL(
        &avrUart0Driver, /* phys UART0 -> uartA */
        &avrUart2Driver, /* phys UART2 -> uartB */
        &avrUart3Driver, /* phys UART3 -> uartC */
		&avrUart1Driver, /* phys UART1 -> uartD */
        &avrI2CDriver,
        &mpngSPIDriver,
        &avrAnalogIn,
        &avrEEPROMStorage,
        &avrUart0Driver,
        &avrGPIO,
        &mpngRCInput,
        &mpngRCOutput,
        &avrScheduler,
        &avrUtil )
{}

void HAL_MPNG::init(int argc, char * const argv[]) const {

    scheduler->init((void*)&isrRegistry);

    /* uartA is the serial port used for the console, so lets make sure
     * it is initialized at boot */
    uartA->begin(115200);
    /* The AVR RCInput drivers take an AP_HAL_MPNG::ISRRegistry*
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
     * PH0 : RX2 (uartB)
     * PJ0 : RX3 (uartC)
     * PD2 : RX1 (uartD)
     */

    PORTE |= _BV(0); // S0
    PORTD |= _BV(2); // S1
    PORTH |= _BV(0); // S2
    PORTJ |= _BV(0); // S3
};

const HAL_MPNG AP_HAL_MPNG;

#endif // CONFIG_HAL_BOARD == HAL_BOARD_MPNG
