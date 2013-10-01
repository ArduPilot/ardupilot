#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

/** 
 * You'll want to use a logic analyzer to watch the effects of this test.
 * On the APM2 its pretty easy to hook up an analyzer to pins A0 through A3.
 */
#define PIN_A0 54  /* A0 */
#define PIN_A1 55  /* A1 */
#define PIN_A2 56  /* A2 */
#define PIN_A3 57  /* A3 */

/** 
 * Create an AVRSemaphore for this test.
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
    hal.console->printf_P(PSTR("Setup pin %d\r\n"), pin_num);
    hal.gpio->pinMode(pin_num,GPIO_OUTPUT);
    /* Blink so we can see setup on the logic analyzer.*/
    hal.gpio->write(pin_num,1);
    hal.gpio->write(pin_num,0);
}

void setup (void) {
    hal.console->printf_P(PSTR("Starting Semaphore test\r\n"));

    setup_pin(PIN_A0);
    setup_pin(PIN_A1);
    setup_pin(PIN_A2);
    setup_pin(PIN_A3);
    
    hal.console->printf_P(PSTR("Using Dataflash's Semaphore\r\n"));

    AP_HAL::SPIDeviceDriver *dataflash = hal.spi->device(
            AP_HAL::SPIDevice_Dataflash);

    if (dataflash == NULL) {
        hal.scheduler->panic(PSTR("Error: No SPIDevice_Dataflash driver!"));
    }
   
    sem = dataflash->get_semaphore();
    if (sem == NULL) {
        hal.scheduler->panic(PSTR("Error: No SPIDeviceDriver semaphore!"));
    }

    hal.scheduler->register_timer_process(async_blinker, NULL);
}


static uint32_t async_last_run;

void async_blinker(void *) 
{
    uint32_t now = hal.scheduler->millis();
    if ((now - async_last_run) < 5)  {
        return;
    }
    async_last_run = now;
    
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
