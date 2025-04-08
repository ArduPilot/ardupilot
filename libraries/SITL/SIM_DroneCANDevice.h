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
  base class for CAN simulated devices
*/

#pragma once

#include <stdint.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/vectorN.h>

#if AP_TEST_DRONECAN_DRIVERS
namespace SITL {

class DroneCANDevice {
public:
    void update(void);

private:
    // barometer delay buffer variables
    struct readings_baro {
        uint32_t time;
        float data;
    };
    uint8_t _store_index;
    uint32_t _last_store_time;
    static const uint8_t _buffer_length = 50;
    VectorN<readings_baro, _buffer_length> _buffer;
      float _last_altitude;
    void update_baro(void);

    void update_airspeed(void);
    void update_compass(void);
    void update_rangefinder(void);
    void _setup_eliptical_correcion(uint8_t i);
    uint64_t _baro_last_update_us;
    uint64_t _airspeed_last_update_us;
    uint64_t _compass_last_update_us;
    uint64_t _rangefinder_last_update_us;
    Matrix3f _eliptical_corr;
    Vector3f _last_dia;
    Vector3f _last_odi;
};

}
#endif // AP_TEST_DRONECAN_DRIVERS