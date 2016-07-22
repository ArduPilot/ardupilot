/*
 *       Example of APM_Compass library (HMC5843 sensor).
 *       Code by Jordi MuÒoz and Jose Julio. DIYDrones.com
 */

#include <AP_Compass/AP_Compass.h>
#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static Compass compass;

uint32_t timer;

void setup() {
    hal.console->println("Compass library test");

    if (!compass.init()) {
        AP_HAL::panic("compass initialisation failed!");
    }
    hal.console->printf("init done - %u compasses detected\n", compass.get_count());

    // set offsets to account for surrounding interference
    compass.set_and_save_offsets(0, 0, 0, 0);
    // set local difference between magnetic north and true north
    compass.set_declination(ToRad(0.0f));

    hal.scheduler->delay(1000);
    timer = AP_HAL::micros();
}

void loop()
{
    static const uint8_t compass_count = compass.get_count();
    static float min[COMPASS_MAX_INSTANCES][3];
    static float max[COMPASS_MAX_INSTANCES][3];
    static float offset[COMPASS_MAX_INSTANCES][3];

    compass.accumulate();

    if ((AP_HAL::micros() - timer) > 100000L) {
        timer = AP_HAL::micros();
        compass.read();
        unsigned long read_time = AP_HAL::micros() - timer;

        for (uint8_t i = 0; i < compass_count; i++) {
            float heading;

            hal.console->printf("Compass #%u: ", i);

            if (!compass.healthy()) {
                hal.console->println("not healthy");
                continue;
            }

            Matrix3f dcm_matrix;
            // use roll = 0, pitch = 0 for this example
            dcm_matrix.from_euler(0, 0, 0);
            heading = compass.calculate_heading(dcm_matrix, i);
            compass.learn_offsets();

            const Vector3f &mag = compass.get_field(i);

            // capture min
            min[i][0] = MIN(mag.x, min[i][0]);
            min[i][1] = MIN(mag.y, min[i][1]);
            min[i][2] = MIN(mag.z, min[i][2]);

            // capture max
            max[i][0] = MAX(mag.x, max[i][0]);
            max[i][1] = MAX(mag.y, max[i][1]);
            max[i][2] = MAX(mag.z, max[i][2]);

            // calculate offsets
            offset[i][0] = -(max[i][0] + min[i][0]) / 2;
            offset[i][1] = -(max[i][1] + min[i][1]) / 2;
            offset[i][2] = -(max[i][2] + min[i][2]) / 2;

            // display all to user
            hal.console->printf("Heading: %.2f (%3d,%3d,%3d)",
                                ToDeg(heading),
                                (int)mag.x,
                                (int)mag.y,
                                (int)mag.z);

            // display offsets
            hal.console->printf(" offsets(%.2f, %.2f, %.2f)",
                                offset[i][0], offset[i][1], offset[i][2]);

            hal.console->printf(" t=%u", (unsigned)read_time);

            hal.console->println();
        }
    } else {
        hal.scheduler->delay(1);
    }
}

AP_HAL_MAIN();
