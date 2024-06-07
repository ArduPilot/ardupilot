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
  generic Baro driver test
 */

#include <AP_Baro/AP_Baro.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>


const AP_HAL::HAL &hal = AP_HAL::get_HAL();

// create barometer object
static AP_Baro barometer;
#if HAL_EXTERNAL_AHRS_ENABLED
 static AP_ExternalAHRS eAHRS;
#endif // HAL_EXTERNAL_AHRS_ENABLED

static uint32_t timer;
static AP_BoardConfig board_config;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
SITL::SIM sitl;
#endif

void setup();
void loop();

// to be called only once on boot for initializing objects
void setup()
{
    hal.console->printf("Barometer library test\n");

    board_config.init();

    hal.scheduler->delay(1000);

    // initialize the barometer
    barometer.init();
    barometer.calibrate();

    // set up timer to count time in microseconds
    timer = AP_HAL::micros();
}

// loop
void loop()
{
    // terminate program if console fails to initialize
    if (!hal.console->is_initialized()) {
        return;
    }

    // run update() at 10Hz
    if ((AP_HAL::micros() - timer) > 100 * 1000UL) {
        timer = AP_HAL::micros();
        barometer.update();

        //calculate time taken for barometer readings to update
        uint32_t read_time = AP_HAL::micros() - timer;
        if (!barometer.healthy()) {
            hal.console->printf("not healthy\n");
            return;
        }

        //output barometer readings to console
        hal.console->printf(" Pressure: %.2f Pa\n"
                            " Temperature: %.2f degC\n"
                            " Relative Altitude: %.2f m\n"
                            " climb=%.2f m/s\n"
                            " Read + update time: %u usec\n"
                            "\n",
                            (double)barometer.get_pressure(),
                            (double)barometer.get_temperature(),
                            (double)barometer.get_altitude(),
                            (double)barometer.get_climb_rate(),
                            (unsigned)read_time);
    } else {
        // if stipulated time has not passed between two distinct readings, delay the program for a millisecond
        hal.scheduler->delay(1);
    }
}

const struct AP_Param::GroupInfo        GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};
GCS_Dummy _gcs;


AP_HAL_MAIN();
