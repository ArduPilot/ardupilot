// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//
// Simple test for the AP_InertialSensor driver.
//

#include <stdarg.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
#define A_LED_PIN 27
#define C_LED_PIN 25
#else
#define A_LED_PIN 37
#define C_LED_PIN 35
#endif

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
AP_InertialSensor_MPU6000 ins;

static void flash_leds(bool on) {
    hal.gpio->write(A_LED_PIN, on);
    hal.gpio->write(C_LED_PIN, ~on);
}

void setup(void)
{
    hal.console->println("AP_InertialSensor startup...");

    hal.gpio->pinMode(A_LED_PIN, GPIO_OUTPUT);
    hal.gpio->pinMode(C_LED_PIN, GPIO_OUTPUT);

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    // we need to stop the barometer from holding the SPI bus
    hal.gpio->pinMode(40, GPIO_OUTPUT);
    hal.gpio->write(40, 1);
#endif

    ins.init(AP_InertialSensor::COLD_START, 
			 AP_InertialSensor::RATE_100HZ,
			 NULL);

    // display initial values
    display_offsets_and_scaling();
    hal.console->println("Complete. Reading:");
}

void loop(void)
{
    int16_t user_input;

    hal.console->println();
    hal.console->println_P(PSTR(
    "Menu:\r\n"
    "    c) calibrate accelerometers\r\n"
    "    d) display offsets and scaling\r\n"
    "    l) level (capture offsets from level)\r\n"
    "    t) test\r\n"
    "    r) reboot"));

    // wait for user input
    while( !hal.console->available() ) {
        hal.scheduler->delay(20);
    }

    // read in user input
    while( hal.console->available() ) {
        user_input = hal.console->read();

        if( user_input == 'c' || user_input == 'C' ) {
            run_calibration();
            display_offsets_and_scaling();
        }

        if( user_input == 'd' || user_input == 'D' ) {
            display_offsets_and_scaling();
        }

        if( user_input == 'l' || user_input == 'L' ) {
            run_level();
            display_offsets_and_scaling();
        }

        if( user_input == 't' || user_input == 'T' ) {
            run_test();
        }

        if( user_input == 'r' || user_input == 'R' ) {
			hal.scheduler->reboot();
        }
    }
}

void run_calibration()
{
    // clear off any other characters (like line feeds,etc)
    while( hal.console->available() ) {
        hal.console->read();
    }


#if !defined( __AVR_ATmega1280__ )
    AP_InertialSensor_UserInteractStream interact(hal.console);
    ins.calibrate_accel(NULL, &interact);
#else
	hal.console->println_P(PSTR("calibrate_accel not available on 1280"));
#endif
}

void display_offsets_and_scaling()
{
    Vector3f accel_offsets = ins.get_accel_offsets();
    Vector3f accel_scale = ins.get_accel_scale();
    Vector3f gyro_offsets = ins.get_gyro_offsets();

    // display results
    hal.console->printf_P(
            PSTR("\nAccel Offsets X:%10.8f \t Y:%10.8f \t Z:%10.8f\n"),
                    accel_offsets.x,
                    accel_offsets.y,
                    accel_offsets.z);
    hal.console->printf_P(
            PSTR("Accel Scale X:%10.8f \t Y:%10.8f \t Z:%10.8f\n"),
                    accel_scale.x,
                    accel_scale.y,
                    accel_scale.z);
    hal.console->printf_P(
            PSTR("Gyro Offsets X:%10.8f \t Y:%10.8f \t Z:%10.8f\n"),
                    gyro_offsets.x,
                    gyro_offsets.y,
                    gyro_offsets.z);
}

void run_level()
{
    // clear off any input in the buffer
    while( hal.console->available() ) {
        hal.console->read();
    }

    // display message to user
    hal.console->print("Place APM on a level surface and press any key..\n");

    // wait for user input
    while( !hal.console->available() ) {
        hal.scheduler->delay(20);
    }
    while( hal.console->available() ) {
        hal.console->read();
    }

    // run accel level
    ins.init_accel(flash_leds);

    // display results
    display_offsets_and_scaling();
}

void run_test()
{
    Vector3f accel;
    Vector3f gyro;
    float temperature;
    float length;
	uint8_t counter = 0;

    // flush any user input
    while( hal.console->available() ) {
        hal.console->read();
    }

    // clear out any existing samples from ins
    ins.update();

    // loop as long as user does not press a key
    while( !hal.console->available() ) {

        // wait until we have a sample
        while (ins.num_samples_available() == 0) /* noop */ ;

        // read samples from ins
        ins.update();
        accel = ins.get_accel();
        gyro = ins.get_gyro();
        temperature = ins.temperature();

        length = accel.length();

		if (counter++ % 50 == 0) {
			// display results
			hal.console->printf_P(PSTR("Accel X:%4.2f \t Y:%4.2f \t Z:%4.2f \t len:%4.2f \t Gyro X:%4.2f \t Y:%4.2f \t Z:%4.2f \t Temp:%4.2f\n"), 
								  accel.x, accel.y, accel.z, length, gyro.x, gyro.y, gyro.z, temperature);
		}
    }

    // clear user input
    while( hal.console->available() ) {
        hal.console->read();
    }
}

AP_HAL_MAIN();
