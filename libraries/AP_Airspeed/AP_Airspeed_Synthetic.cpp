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
 *   synthetic airspeed driver, based on GPS data and pressure altitude
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_AHRS/AP_AHRS.h>

#include "AP_Airspeed.h"
#include "AP_Airspeed_Synthetic.h"

extern const AP_HAL::HAL &hal;

AP_Airspeed_Synthetic::AP_Airspeed_Synthetic(AP_Airspeed &_frontend, uint8_t _instance) :
    AP_Airspeed_Backend(_frontend, _instance)
{
}

bool AP_Airspeed_Synthetic::init()
{
    set_use_zero_offset();
    set_skip_cal();
    set_offset(0);
    return true;
}

// read the airspeed sensor
bool AP_Airspeed_Synthetic::get_differential_pressure(float &pressure)
{
    const AP_GPS &gps = AP::gps();
    AP_AHRS &ahrs = AP::ahrs();
    Vector3f gnd_speed;
    if (!ahrs.get_velocity_NED(gnd_speed) ||
        gps.status() < AP_GPS::GPS_OK_FIX_3D) {
        return false;
    }
    float wind_direction_from=0, wind_speed_mps=0;

    get_wind(wind_direction_from, wind_speed_mps);

    // calculate wind vector
    Vector3f wind(cosf(radians(wind_direction_from)), sinf(radians(wind_direction_from)), 0);
    wind *= wind_speed_mps;

    set_use_zero_offset();
    Vector3f aspeed3d = gnd_speed / ahrs.get_EAS2TAS();

    // add wind in ground frame
    aspeed3d += wind;

    // rotate to body frame
    const Matrix3f &rot = ahrs.get_rotation_body_to_ned();
    aspeed3d = rot.mul_transpose(aspeed3d);
    float airspeed = MAX(aspeed3d.x, 0);
    const float ratio = get_airspeed_ratio();
    pressure = sq(airspeed) / ratio;
    return true;
}
