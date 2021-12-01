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
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Compass/CompassCalibrator.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

class CompassCalibratorAccess : public CompassCalibrator{
public:
    Rotation auto_rotation_index_test(uint8_t n) { return auto_rotation_index(n);}
    bool right_angle_rotation_test(Rotation r) { return right_angle_rotation(r);}

};


CompassCalibratorAccess cal;

/*
 *  rotation tests
 */
void setup(void)
{
    hal.console->begin(115200);
    hal.console->printf("\n\ncalibration index test\n\n");

    for (uint8_t n=0; n < ROTATION_MAX; n++) {
        Rotation r = cal.auto_rotation_index_test(n);
        hal.console->printf("index: %i got %i%s\n",n,r,cal.right_angle_rotation_test(Rotation(n)) ? ", R" : "");
    }

    hal.console->printf("rotation unit tests done\n\n");
}

void loop(void) {}

AP_HAL_MAIN();
