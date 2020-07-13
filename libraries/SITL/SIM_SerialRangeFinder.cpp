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
  Base class for serial rangefinders
*/

#include "SIM_SerialRangeFinder.h"

using namespace SITL;

uint16_t SerialRangeFinder::calculate_range_cm(float range_value) const
{
    // swiped from sitl_rangefinder.cpp - we should unify them at some stage

    const SITL *sitl = AP::sitl();

    float altitude = range_value;
    if (is_equal(range_value, -1.0f)) {  // Use SITL altitude as reading by default
        altitude = AP::sitl()->height_agl;
    }

    // sensor position offset in body frame
    const Vector3f relPosSensorBF = sitl->rngfnd_pos_offset;

    // adjust altitude for position of the sensor on the vehicle if position offset is non-zero
    if (!relPosSensorBF.is_zero()) {
        // get a rotation matrix following DCM conventions (body to earth)
        Matrix3f rotmat;
        sitl->state.quaternion.rotation_matrix(rotmat);
        // rotate the offset into earth frame
        const Vector3f relPosSensorEF = rotmat * relPosSensorBF;
        // correct the altitude at the sensor
        altitude -= relPosSensorEF.z;
    }

    // If the attidude is non reversed for SITL OR we are using rangefinder from external simulator,
    // We adjust the reading with noise, glitch and scaler as the reading is on analog port.
    if ((fabs(sitl->state.rollDeg) < 90.0 && fabs(sitl->state.pitchDeg) < 90.0) || !is_equal(range_value, -1.0f)) {
        if (is_equal(range_value, -1.0f)) {  // disable for external reading that already handle this
            // adjust for apparent altitude with roll
            altitude /= cosf(radians(sitl->state.rollDeg)) * cosf(radians(sitl->state.pitchDeg));
        }
        // Add some noise on reading
        altitude += sitl->sonar_noise * rand_float();
    }

    return altitude * 100.0f;
}

void SerialRangeFinder::update(float range)
{
    // just send a chunk of data at 5Hz:
    const uint32_t now = AP_HAL::millis();
    if (now - last_sent_ms < reading_interval_ms()) {
        return;
    }
    last_sent_ms = now;

    uint8_t data[255];
    const uint32_t packetlen = packet_for_alt(calculate_range_cm(range),
                                              data,
                                              ARRAY_SIZE(data));

    write_to_autopilot((char*)data, packetlen);
}
