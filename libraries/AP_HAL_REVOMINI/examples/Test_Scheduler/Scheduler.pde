
#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>

#include <AP_HAL.h>
#include <AP_HAL_VRBRAIN.h>

const AP_HAL::HAL& hal = AP_HAL_VRBRAIN;


/** 
 * You'll want to use a logic analyzer to watch the effects of this test.
 * On the APM2 its pretty easy to hook up an analyzer to pins A0 through A3.
 */
#define DELAY_TOGGLE_PIN 19        /* A0 */
#define FAILSAFE_TOGGLE_PIN 20     /* A1 */
#define SCHEDULED_TOGGLE_PIN_1 21  /* A2 */
#define SCHEDULED_TOGGLE_PIN_2 19  /* A3 */
static int32_t toggle_1_time;
static int32_t toggle_2_time;

void delay_toggle() {
    volatile int i;
    hal.gpio->write(DELAY_TOGGLE_PIN, 1);
    for (i = 0; i < 10; i++);
    hal.gpio->write(DELAY_TOGGLE_PIN, 0);
}

void failsafe_toggle(uint32_t machtnichts) {
    volatile int i;
    hal.gpio->write(FAILSAFE_TOGGLE_PIN, 1);
    for (i = 0; i < 10; i++);
    hal.gpio->write(FAILSAFE_TOGGLE_PIN, 0);
}


void schedule_toggle_1(uint32_t machtnichts) {
    if((hal.scheduler->millis() - toggle_1_time) >= 500){
	toggle_1_time = hal.scheduler->millis();
	if(hal.gpio->read(SCHEDULED_TOGGLE_PIN_1)){
	    hal.gpio->write(SCHEDULED_TOGGLE_PIN_1, 0);
	} else {
	    hal.gpio->write(SCHEDULED_TOGGLE_PIN_1, 1);
	}
    }
}

void schedule_toggle_2(uint32_t machtnichts) {
    if((hal.scheduler->millis() - toggle_2_time) >= 1000){
	toggle_2_time = hal.scheduler->millis();
	if(hal.gpio->read(SCHEDULED_TOGGLE_PIN_2)){
	    hal.gpio->write(SCHEDULED_TOGGLE_PIN_2, 0);
	} else {
	    hal.gpio->write(SCHEDULED_TOGGLE_PIN_2, 1);
	}
    }
}

void schedule_toggle_hang(uint32_t machtnichts) {
    hal.gpio->write(SCHEDULED_TOGGLE_PIN_2, 1); 
    for(;;);
}

void setup_pin(int pin_num) {
    hal.console->printf_P(PSTR("Setup pin %d\r\n"), pin_num);
    hal.gpio->pinMode(pin_num,OUTPUT);
    /* Blink so we can see setup on the logic analyzer.*/
    hal.gpio->write(pin_num,1);
    hal.scheduler->delay(1000);
    hal.gpio->write(pin_num,0);
}

void setup (void) {
    hal.console->printf_P(PSTR("Starting AP_HAL_AVR::Scheduler test\r\n"));

    setup_pin(DELAY_TOGGLE_PIN);
    hal.scheduler->delay(1000);
    setup_pin(FAILSAFE_TOGGLE_PIN);
    hal.scheduler->delay(1000);
    setup_pin(SCHEDULED_TOGGLE_PIN_1);
    hal.scheduler->delay(1000);
    setup_pin(SCHEDULED_TOGGLE_PIN_2);
    /*
    hal.console->printf_P(PSTR("Testing delay callback. "
                "Pin %d should toggle at 1khz:\r\n"),
            (int) DELAY_TOGGLE_PIN);

    hal.scheduler->register_delay_callback(delay_toggle,0);

    hal.scheduler->delay(100);

    hal.console->printf_P(PSTR("Testing failsafe callback. "
                "Pin %d should toggle at 1khz:\r\n"),
            (int) FAILSAFE_TOGGLE_PIN);

    hal.scheduler->register_timer_failsafe(failsafe_toggle, 1000);

    hal.scheduler->delay(100);
*/
    hal.console->printf_P(PSTR("Testing running timer processes.\r\n"));
    hal.console->printf_P(PSTR("Pin %d should toggle at 1hz.\r\n"),
            (int) SCHEDULED_TOGGLE_PIN_1);
    hal.console->printf_P(PSTR("Pin %d should toggle at 1hz.\r\n"),
            (int) SCHEDULED_TOGGLE_PIN_2);

    hal.scheduler->register_timer_process(schedule_toggle_1);
    hal.scheduler->register_timer_process(schedule_toggle_2);

    hal.scheduler->delay(100);
/*
    hal.console->printf_P(PSTR("Test running a pathological timer process.\r\n"
                "Failsafe should continue even as pathological process "
                "dominates the processor."));
    hal.console->printf_P(PSTR("Pin %d should toggle then go high forever.\r\n"),
            (int) SCHEDULED_TOGGLE_PIN_2);
    hal.scheduler->register_timer_process(schedule_toggle_hang);
    */
}

void loop (void) { }

AP_HAL_MAIN();
