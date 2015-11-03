
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
#define DELAY_TOGGLE_PIN       15  /* A0 = pin 15 */
#define FAILSAFE_TOGGLE_PIN    16  /* A1 = pin 16 */
#define SCHEDULED_TOGGLE_PIN_1 17  /* A2 = pin 17 */
#define SCHEDULED_TOGGLE_PIN_2 18  /* A3 = pin 18 */

void delay_toggle() {
    volatile int i;
    hal.gpio->write(DELAY_TOGGLE_PIN, 1);
    for (i = 0; i < 10; i++);
    hal.gpio->write(DELAY_TOGGLE_PIN, 0);
}

void failsafe_toggle(void) {
    volatile int i;
    hal.gpio->write(FAILSAFE_TOGGLE_PIN, 1);
    for (i = 0; i < 10; i++);
    hal.gpio->write(FAILSAFE_TOGGLE_PIN, 0);
}


void schedule_toggle_1(void) {
    volatile int i;
    hal.gpio->write(SCHEDULED_TOGGLE_PIN_1, 1);
    for (i = 0; i < 10; i++);
    hal.gpio->write(SCHEDULED_TOGGLE_PIN_1, 0);
}

void schedule_toggle_2(void) {
    volatile int i;
    hal.gpio->write(SCHEDULED_TOGGLE_PIN_2, 1);
    for (i = 0; i < 10; i++);
    hal.gpio->write(SCHEDULED_TOGGLE_PIN_2, 0);
}

void schedule_toggle_hang(void) {
    hal.gpio->write(SCHEDULED_TOGGLE_PIN_2, 1); 
    for(;;);
}

void setup_pin(int pin_num) {
    hal.console->printf("Setup pin %d\r\n", pin_num);
    hal.gpio->pinMode(pin_num,HAL_GPIO_OUTPUT);
    /* Blink so we can see setup on the logic analyzer.*/
    hal.gpio->write(pin_num,1);
    hal.gpio->write(pin_num,0);
}

void setup (void) {
//    hal.scheduler->delay(5000);
    hal.console->printf("Starting AP_HAL::Scheduler test\r\n");

    setup_pin(DELAY_TOGGLE_PIN);
    setup_pin(FAILSAFE_TOGGLE_PIN);
    setup_pin(SCHEDULED_TOGGLE_PIN_1);
    setup_pin(SCHEDULED_TOGGLE_PIN_2);

    hal.console->printf("Testing delay callback. "
                "Pin %d should toggle at 1khz:\r\n",
            (int) DELAY_TOGGLE_PIN);

//    hal.scheduler->register_delay_callback(delay_toggle,0);

    hal.scheduler->delay(2000);
    hal.console->printf("Testing failsafe callback. "
                "Pin %d should toggle at 1khz:\r\n",
            (int) FAILSAFE_TOGGLE_PIN);

    hal.scheduler->register_timer_failsafe(failsafe_toggle, 1000);

    hal.scheduler->delay(2000);

    hal.console->printf("Testing running timer processes.\r\n");
    hal.console->printf("Pin %d should toggle at 1khz.\r\n",
            (int) SCHEDULED_TOGGLE_PIN_1);
    hal.console->printf("Pin %d should toggle at 1khz.\r\n",
            (int) SCHEDULED_TOGGLE_PIN_2);

    hal.scheduler->register_timer_process(schedule_toggle_1);
    hal.scheduler->register_timer_process(schedule_toggle_2);

    hal.scheduler->delay(2000);

    // not yet working on flymaple: see FLYMAPLEScheduler::_timer_procs_timer_event()
#if 1
    hal.console->printf("Test running a pathological timer process.\r\n"
                "Failsafe should continue even as pathological process "
                "dominates the processor.");
    hal.console->printf("Pin %d should toggle then go high forever.\r\n",
            (int) SCHEDULED_TOGGLE_PIN_2);
    hal.scheduler->delay(200);
    hal.scheduler->register_timer_process(schedule_toggle_hang);
#endif

    // Wait a little then test reboot
//    hal.scheduler->delay(5000);
//    hal.scheduler->reboot();
}

void loop (void) { }

AP_HAL_MAIN();
