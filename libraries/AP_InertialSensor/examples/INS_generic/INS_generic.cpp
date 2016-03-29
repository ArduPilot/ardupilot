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

    // display number of detected accels/gyros
    hal.console->printf("\n");
    hal.console->printf("Number of detected accels : %d\n",ins.get_accel_count());
    hal.console->printf("Number of detected gyros  : %d\n\n",ins.get_gyro_count());

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
	uint8_t counter = 0;
	static uint8_t accel_count = ins.get_accel_count();
	static uint8_t gyro_count = ins.get_gyro_count();
	static uint8_t ins_count;
	char accel_state;
	char gyro_state;

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

        // print each accel/gyro result every 50 cycles
        if (counter++ % 50 == 0)
        {
            // determine how many sensors to print for
            if (accel_count>gyro_count)
            {
                ins_count = accel_count;
            }
            else
            {
                ins_count = gyro_count;
            }

            // loop and print each sensor
            for (uint8_t ii = 0; ii<ins_count; ii++)
            {
                hal.console->printf("%u - ",ii);

                // print accel
                accel = ins.get_accel(ii);

                if (ins.get_accel_health(ii)) { accel_state = 'h'; } // Healthy accel
                else if (ii+1>accel_count) { accel_state = '-'; }    // No accel present
                else { accel_state = 'x'; }                          // Accel present but not healthy

                hal.console->printf("Accel (%c) : X:%6.2f Y:%6.2f Z:%6.2f norm:%5.2f",
                                       accel_state, accel.x, accel.y, accel.z, accel.length());

                // print gyro
                gyro = ins.get_gyro(ii);

                if (ins.get_gyro_health(ii)) { gyro_state = 'h'; } // Healthy gyro
                else if (ii+1>gyro_count) { gyro_state = '-'; }    // No gyro present
                else { gyro_state = 'x'; }                         // Gyro present but not healthy

                hal.console->printf("   Gyro (%c) : X:%6.2f Y:%6.2f Z:%6.2f\n",
                        gyro_state,gyro.x, gyro.y, gyro.z);

            }
        }
    }

    // clear user input
    while( hal.console->available() ) {
        hal.console->read();
    }
}

AP_HAL_MAIN();
