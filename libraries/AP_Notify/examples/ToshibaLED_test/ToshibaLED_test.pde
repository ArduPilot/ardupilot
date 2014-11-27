// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Param.h>
#include <Filter.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>
#include <AP_GPS.h>
#include <AP_Baro.h>
#include <DataFlash.h>
#include <GCS_MAVLink.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <AP_Declination.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Empty.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_Notify.h>          // Notify library
#include <ToshibaLED.h>
#include <AP_AHRS.h>
#include <AP_NavEKF.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_Compass.h>
#include <AP_Declination.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
static ToshibaLED_PX4 toshiba_led;
#else
static ToshibaLED_I2C toshiba_led;
#endif

static uint8_t led_state;
static uint8_t red, green, blue;

void setup(void)
{
    // display welcome message
    hal.console->print_P(PSTR("Toshiba LED test ver 0.1\n"));

    // initialise LED
    toshiba_led.init();

    // check if healthy
    if (!toshiba_led.healthy()) {
        hal.console->print_P(PSTR("Failed to initialise Toshiba LED\n"));
    }

    // turn on initialising notification
    AP_Notify::flags.initialising = false;
    AP_Notify::flags.save_trim = true;
    AP_Notify::flags.gps_status = 1;
    AP_Notify::flags.armed = 1;
    AP_Notify::flags.pre_arm_check = 1;
}

void loop(void)
{
    // blink test
    //hal.console->print_P(PSTR("Blink test\n"));
    //blink();
    /*
    // full spectrum test
    hal.console->print_P(PSTR("Spectrum test\n"));
    full_spectrum();
    */

    // update the toshiba led
    toshiba_led.update();

    // wait 1/50th of a second
    hal.scheduler->delay(20);
}

// full_spectrum - runs through the full spectrum of colours the led can display
void full_spectrum()
{
    // go through the full range of colours but only up to the dim light level
    for (uint8_t red=0; red<=0x05; red++) {
        for (uint8_t green=0; green<=0x05; green++) {
            for (uint8_t blue=0; blue<=0x05; blue++) {
                toshiba_led.set_rgb(red,green,blue);
                hal.scheduler->delay(5);
            }
        }
    }
}

#define LED_DIM 0x11

// blink - blink the led at 10hz for 10 seconds
void blink()
{
    // set colour to red
    toshiba_led.set_rgb(LED_DIM,0,0);

    // full spectrum test
    for (uint8_t c=0; c<=2; c++ ) {
        if (c==0) {
            toshiba_led.set_rgb(LED_DIM,0,0);   // red
        }else if (c==1) {
            toshiba_led.set_rgb(0,LED_DIM,0);   // green
        }else{
            toshiba_led.set_rgb(0,0,LED_DIM);   // blue
        }
    }
}

AP_HAL_MAIN();
