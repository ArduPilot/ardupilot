/*
  simple test of UART interfaces
 */

#include <AP_HAL/AP_HAL.h>

#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  setup one UART at 57600
 */
static void setup_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->begin(57600);
}


void setup(void)
{
    /*
      start all UARTs at 57600 with default buffer sizes
    */

    hal.scheduler->delay(1000); //Ensure that the uartA can be initialized

    setup_uart(hal.serial(0), "uartA");  // console
    setup_uart(hal.serial(3), "uartB");  // 1st GPS
    setup_uart(hal.serial(1), "uartC");  // telemetry 1
    setup_uart(hal.serial(2), "uartD");  // telemetry 2
    setup_uart(hal.serial(3), "uartE");  // 2nd GPS
}

static void test_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->printf("Hello on UART %s at %.3f seconds\n",
                 name, (double)(AP_HAL::millis() * 0.001f));
}

void loop(void)
{
    test_uart(hal.serial(0), "uartA");
    test_uart(hal.serial(3), "uartB");
    test_uart(hal.serial(1), "uartC");
    test_uart(hal.serial(2), "uartD");
    test_uart(hal.serial(3), "uartE");

    // also do a raw printf() on some platforms, which prints to the
    // debug console
#if HAL_OS_POSIX_IO
    ::printf("Hello on debug console at %.3f seconds\n", (double)(AP_HAL::millis() * 0.001f));
#endif

    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
