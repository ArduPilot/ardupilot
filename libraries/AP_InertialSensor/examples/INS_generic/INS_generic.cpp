// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//
// Simple test for the AP_InertialSensor driver.
//

#include <AP_ADC/AP_ADC.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_InertialSensor/AP_InertialSensor.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

AP_InertialSensor ins;

static void display_offsets_and_scaling();
static void run_test();

void setup(void)
{
    hal.console->println("AP_InertialSensor startup...");

    ins.init(100);

    // display initial values
    display_offsets_and_scaling();
    hal.console->println("Complete. Reading:");
}

void loop(void)
{
    int16_t user_input;

    hal.console->println();
    hal.console->println(
    "Menu:\r\n"
    "    d) display offsets and scaling\r\n"
    "    l) level (capture offsets from level)\r\n"
    "    t) test\r\n"
    "    r) reboot");

    // wait for user input
    while( !hal.console->available() ) {
        hal.scheduler->delay(20);
    }

    // read in user input
    while( hal.console->available() ) {
        user_input = hal.console->read();

        if( user_input == 'd' || user_input == 'D' ) {
            display_offsets_and_scaling();
        }

        if( user_input == 't' || user_input == 'T' ) {
            run_test();
        }

        if( user_input == 'r' || user_input == 'R' ) {
			hal.scheduler->reboot(false);
        }
    }
}

static void display_offsets_and_scaling()
{
    Vector3f accel_offsets = ins.get_accel_offsets();
    Vector3f accel_scale = ins.get_accel_scale();
    Vector3f gyro_offsets = ins.get_gyro_offsets();

    // display results
    hal.console->printf(
            "\nAccel Offsets X:%10.8f \t Y:%10.8f \t Z:%10.8f\n",
                    accel_offsets.x,
                    accel_offsets.y,
                    accel_offsets.z);
    hal.console->printf(
            "Accel Scale X:%10.8f \t Y:%10.8f \t Z:%10.8f\n",
                    accel_scale.x,
                    accel_scale.y,
                    accel_scale.z);
    hal.console->printf(
            "Gyro Offsets X:%10.8f \t Y:%10.8f \t Z:%10.8f\n",
                    gyro_offsets.x,
                    gyro_offsets.y,
                    gyro_offsets.z);
}

static void run_test()
{
    Vector3f accel;
    Vector3f gyro;
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
        ins.wait_for_sample();

        // read samples from ins
        ins.update();
        accel = ins.get_accel();
        gyro = ins.get_gyro();

        length = accel.length();

		if (counter++ % 50 == 0) {
			// display results
			hal.console->printf("Accel X:%4.2f \t Y:%4.2f \t Z:%4.2f \t len:%4.2f \t Gyro X:%4.2f \t Y:%4.2f \t Z:%4.2f\n", 
								  accel.x, accel.y, accel.z, length, gyro.x, gyro.y, gyro.z);
		}
    }

    // clear user input
    while( hal.console->available() ) {
        hal.console->read();
    }
}

AP_HAL_MAIN();
