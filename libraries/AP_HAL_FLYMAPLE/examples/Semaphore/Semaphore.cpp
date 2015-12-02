#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_FLYMAPLE/AP_HAL_FLYMAPLE.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
 * You'll want to use a logic analyzer to watch the effects of this test.
 * Define each of the pins below to the pins used during the test on your
 * board.
 */
#define PIN_A0 15
#define PIN_A1 16
#define PIN_A2 17
#define PIN_A3 18

/** 
 * Create a Semaphore for this test.
 */

AP_HAL::Semaphore *sem;

void blink_a0() {
    volatile int i;
    hal.gpio->write(PIN_A0, 1);
    for (i = 0; i < 10; i++);
    hal.gpio->write(PIN_A0, 0);
}

void blink_a1() {
    volatile int i;
    hal.gpio->write(PIN_A1, 1);
    for (i = 0; i < 10; i++);
    hal.gpio->write(PIN_A1, 0);
}


void blink_a2() {
    volatile int i;
    hal.gpio->write(PIN_A2, 1);
    for (i = 0; i < 10; i++);
    hal.gpio->write(PIN_A2, 0);
}

void blink_a3() {
    volatile int i;
    hal.gpio->write(PIN_A3, 1);
    for (i = 0; i < 10; i++);
    hal.gpio->write(PIN_A3, 0);
}

void setup_pin(int pin_num) {
    hal.console->printf("Setup pin %d\r\n", pin_num);
    hal.gpio->pinMode(pin_num,HAL_GPIO_OUTPUT);
    /* Blink so we can see setup on the logic analyzer.*/
    hal.gpio->write(pin_num,1);
    hal.gpio->write(pin_num,0);
}

void setup (void) {
    hal.console->printf("Starting Semaphore test\r\n");

    setup_pin(PIN_A0);
    setup_pin(PIN_A1);
    setup_pin(PIN_A2);
    setup_pin(PIN_A3);
    
    hal.console->printf("Using SPIDeviceManager builtin Semaphore\r\n");

    AP_HAL::SPIDeviceDriver *dataflash = hal.spi->device(AP_HAL::SPIDevice_Dataflash); // not really

    if (dataflash == NULL) {
        AP_HAL::panic("Error: No SPIDeviceDriver!");
    }
   
    sem = dataflash->get_semaphore();
    if (sem == NULL) {
        AP_HAL::panic("Error: No SPIDeviceDriver semaphore!");
    }

    hal.scheduler->register_timer_process(async_blinker);
}


uint32_t async_last_run = 0;
void async_blinker(uint32_t millis) {
    if (async_last_run - millis < 5)  {
        return;
    }
    
    if (sem->take_nonblocking()) {
        blink_a0();
        sem->give();
    } else {
        blink_a1();
    }
}

void loop (void) {

    if (sem->take(1)) {
        blink_a2();
        sem->give();
    } else {
        /* This should never happen: */
        blink_a3(); 
    }
}

AP_HAL_MAIN();
