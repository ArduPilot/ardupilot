/*
  simple test of UART interfaces
 */
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Empty.h>
#include <AP_Common.h>
#include <AP_Baro.h>
#include <AP_ADC.h>
#include <AP_GPS.h>
#include <AP_InertialSensor.h>
#include <AP_Notify.h>
#include <DataFlash.h>
#include <GCS_MAVLink.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <SITL.h>
#include <Filter.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_Math.h>
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_NavEKF.h>
#include <AP_Rally.h>
#include <AP_Scheduler.h>
#include <UARTDriver.h>

#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

static AP_HAL::UARTDriver* uarts[] = {
    hal.uartA, // console
};
#define NUM_UARTS (sizeof(uarts)/sizeof(uarts[0]))


/*
  setup one UART at 57600
 */
static void setup_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == NULL) {
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
    setup_uart(hal.uartA, "uartA"); // console
    setup_uart(hal.uartB, "uartB"); // 1st GPS
    setup_uart(hal.uartC, "uartC"); // telemetry 1
    setup_uart(hal.uartD, "uartD"); // telemetry 2
    setup_uart(hal.uartE, "uartE"); // 2nd GPS
}

static void test_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == NULL) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->printf("Hello on UART %s at %.3f seconds\n",
                 name, hal.scheduler->millis()*0.001f);
}

void loop(void) 
{	
    test_uart(hal.uartA, "uartA");
    test_uart(hal.uartB, "uartB");
    test_uart(hal.uartC, "uartC");
    test_uart(hal.uartD, "uartD");
    test_uart(hal.uartE, "uartE");

    // also do a raw printf() on some platforms, which prints to the
    // debug console
#if HAL_OS_POSIX_IO
    ::printf("Hello on debug console at %.3f seconds\n", hal.scheduler->millis()*0.001f);
#endif

    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
