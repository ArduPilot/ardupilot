/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       Example of APM_Compass library (HMC5843 sensor).
 *       Code by Jordi MuÒoz and Jose Julio. DIYDrones.com
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <AP_SerialManager/AP_SerialManager.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_BoardConfig board_config;

class DummyVehicle {
public:
    AP_AHRS ahrs;  // Need since https://github.com/ArduPilot/ardupilot/pull/10890
    AP_Baro baro; // Compass tries to set magnetic model based on location.
#if AP_COMPASS_EXTERNALAHRS_ENABLED
    AP_ExternalAHRS eAHRS;
#endif  // AP_COMPASS_EXTERNALAHRS_ENABLED
};

static DummyVehicle vehicle;
// create compass object
static Compass compass;
static AP_SerialManager serial_manager;

uint32_t timer;

// to be called only once on boot for initializing objects
static void setup()
{
    hal.console->printf("Compass library test\n");

    board_config.init();
    vehicle.ahrs.init();
    compass.init();
    hal.console->printf("init done - %u compasses detected\n", compass.get_count());

    // set offsets to account for surrounding interference
    compass.set_and_save_offsets(0, Vector3f(0, 0, 0));
    // set local difference between magnetic north and true north
    compass.set_declination(ToRad(0.0f));

    hal.scheduler->delay(1000);
    timer = AP_HAL::micros();
}

// loop
static void loop()
{
    static const uint8_t compass_count = compass.get_count();
    static float min[COMPASS_MAX_INSTANCES][3];
    static float max[COMPASS_MAX_INSTANCES][3];
    static float offset[COMPASS_MAX_INSTANCES][3];

    // run read() at 10Hz
    if ((AP_HAL::micros() - timer) > 100000L) {
        timer = AP_HAL::micros();
        compass.read();
        const uint32_t read_time = AP_HAL::micros() - timer;

        for (uint8_t i = 0; i < compass_count; i++) {
            float heading;

            hal.console->printf("Compass #%u: ", i);

            if (!compass.healthy()) {
                hal.console->printf("not healthy\n");
                continue;
            }

            Matrix3f dcm_matrix;
            // use roll = 0, pitch = 0 for this example
            dcm_matrix.from_euler(0, 0, 0);
            heading = compass.calculate_heading(dcm_matrix, i);

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
            hal.console->printf("Heading: %.2f (%3d, %3d, %3d)",
                                (double)ToDeg(heading),
                                (int)mag.x,
                                (int)mag.y,
                                (int)mag.z);

            // display offsets
            hal.console->printf(" offsets(%.2f, %.2f, %.2f)",
                                (double)offset[i][0],
                                (double)offset[i][1],
                                (double)offset[i][2]);

            hal.console->printf(" t=%u", (unsigned)read_time);

            hal.console->printf("\n");
        }
    } else {

        // if stipulated time has not passed between two distinct readings, delay the program for a millisecond
        hal.scheduler->delay(1);
    }
}

AP_HAL_MAIN();
