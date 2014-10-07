
#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>

#include <AP_HAL.h>
#include <AP_HAL_YUNEEC.h>
#include <utility/pinmap_typedef.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

/** 
 * You'll want to use a logic analyzer to watch the effects of this test.
 * On the APM2 its pretty easy to hook up an analyzer to pins A0 through A3.
 */
#define DELAY_TOGGLE_PIN       PC13
#define FAILSAFE_TOGGLE_PIN    PC13
#define SCHEDULED_TOGGLE_PIN   PC14


void delay_toggle() {
    volatile int i;
    hal.gpio->write(DELAY_TOGGLE_PIN, 0);
    for (i = 0; i < 10; i++);
    hal.gpio->write(DELAY_TOGGLE_PIN, 1);
}

void failsafe_toggle(void) {
    volatile int i;
    hal.gpio->write(FAILSAFE_TOGGLE_PIN, 0);
    for (i = 0; i < 10; i++);
    hal.gpio->write(FAILSAFE_TOGGLE_PIN, 1);
}


void schedule_toggle(void) {
    volatile int i;
    hal.gpio->write(SCHEDULED_TOGGLE_PIN, 0);
    for (i = 0; i < 10; i++);
    hal.gpio->write(SCHEDULED_TOGGLE_PIN, 1);
}

void schedule_toggle_hang(void) {
    hal.gpio->write(SCHEDULED_TOGGLE_PIN, 0);
    for(;;);
}

void setup_pin(int pin_num) {
    hal.console->printf_P(PSTR("Setup pin %d\r\n"), pin_num);
    hal.gpio->pinMode(pin_num,HAL_GPIO_OUTPUT);
    /* Blink so we can see setup on the logic analyzer.*/
	hal.gpio->write(pin_num,0);
	hal.scheduler->delay(500);
	hal.gpio->write(pin_num,1);
	hal.scheduler->delay(500);
}

void setup (void) {

    hal.console->printf_P(PSTR("Starting YUNEEC::Scheduler test\r\n"));

    setup_pin(DELAY_TOGGLE_PIN);
    setup_pin(FAILSAFE_TOGGLE_PIN);
    setup_pin(SCHEDULED_TOGGLE_PIN);

    hal.console->printf_P(PSTR("Testing delay callback. "
                "Pin %d should toggle at 1hz:\r\n"),
            (int) DELAY_TOGGLE_PIN);

    hal.scheduler->register_delay_callback(delay_toggle,1000);
    hal.scheduler->delay(5000);

    hal.scheduler->register_delay_callback(NULL,0);
    hal.scheduler->delay(3000);

    hal.console->printf_P(PSTR("Testing failsafe callback. "
                "Pin %d should toggle at 1khz:\r\n"),
            (int) FAILSAFE_TOGGLE_PIN);

    hal.scheduler->register_timer_failsafe(failsafe_toggle, 1000);
    hal.scheduler->delay(5000);

    hal.console->printf_P(PSTR("Testing running timer processes.\r\n"));
    hal.console->printf_P(PSTR("Pin %d should toggle at 1khz.\r\n"),
            (int) SCHEDULED_TOGGLE_PIN);

    hal.scheduler->register_timer_process(schedule_toggle);
    hal.scheduler->delay(5000);

    hal.console->printf_P(PSTR("Test running a pathological timer process.\r\n"
                "Failsafe should continue even as pathological process "
                "dominates the processor."));
    hal.console->printf_P(PSTR("Pin %d should toggle then go high forever.\r\n"),
            (int) SCHEDULED_TOGGLE_PIN);
    hal.scheduler->delay(200);
    hal.scheduler->register_timer_process(schedule_toggle_hang);


//     Wait a little then test reboot
    hal.scheduler->delay(5000);
    hal.scheduler->reboot(false);
}

void loop (void) {
//    hal.gpio->write(DELAY_TOGGLE_PIN, 0);
//    hal.scheduler->delay_microseconds(1000);
//    hal.gpio->write(DELAY_TOGGLE_PIN, 1);
//    hal.scheduler->delay_microseconds(1000);
}

AP_HAL_MAIN();
