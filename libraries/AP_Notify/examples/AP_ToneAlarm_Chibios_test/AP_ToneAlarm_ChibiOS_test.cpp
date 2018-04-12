/*
 *       Example of AC_Notify library .
 *       DIYDrones.com
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#include <AP_HAL_ChibiOS/Util.h>
#include <AP_HAL_ChibiOS/ToneAlarm.h>
#include <AP_Notify/AP_Notify.h>          // Notify library
#include <AP_Notify/ToneAlarm_ChibiOS.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup()
{
    ::printf("AP_Notify ToneAlarm_ChibiOS test\n");

    ChibiOS::Util::from(hal.util)->toneAlarm_init();

    // turn on initialising notification
    AP_Notify::flags.initialising = true;
    AP_Notify::flags.gps_status = 1;
    AP_Notify::flags.armed = 1;
    AP_Notify::flags.pre_arm_check = 1;
}

// ChibiOS::Util has a static instance of ChibiOS::ToneAlarm
// the ChibiOS _toneAlarm_thread calls Util::_toneAlarm_timer_tick() at 50Hz (100?)
// which implements a state machine:
// (state 0) if init_tune() -> (state 1) complete=false
// (state 1) if set_note -> (state 2) -> if play -> (state 3)
//   play returns true if tune has changed or tune is complete (repeating tunes never complete)
// (state 3) -> (state 1)
// (on every tick) if (complete) -> (state 0)

void loop()
{
    static int tune = 0;
    static uint32_t then = AP_HAL::millis();

    hal.scheduler->delay(20);
    uint32_t now = AP_HAL::millis();
    if ((now - then) > 4000) {
        then = now;
        if (tune >= TONE_NUMBER_OF_TUNES) {
            tune = 0;
        }
        ::printf("tune: %u\n", tune);
        ChibiOS::Util::from(hal.util)->toneAlarm_set_tune(tune);
        tune++;
    }
}

AP_HAL_MAIN();
#endif