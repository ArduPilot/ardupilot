/*
  SITL handling

  This simulates a rangefinder

 */

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"
#include "SITL_State.h"
#include <SITL/SITL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

using namespace HALSITL;

/*
  setup the rangefinder with new input
 */
void SITL_State::_update_rangefinder(float range_value)
{
    float altitude = range_value;
    if (is_equal(range_value, -1.0f)) {  // Use SITL altitude as reading by default
        altitude = _sitl->height_agl;
    }

    // sensor position offset in body frame
    const Vector3f relPosSensorBF = _sitl->rngfnd_pos_offset;

    // adjust altitude for position of the sensor on the vehicle if position offset is non-zero
    if (!relPosSensorBF.is_zero()) {
        // get a rotation matrix following DCM conventions (body to earth)
        Matrix3f rotmat;
        _sitl->state.quaternion.rotation_matrix(rotmat);
        // rotate the offset into earth frame
        const Vector3f relPosSensorEF = rotmat * relPosSensorBF;
        // correct the altitude at the sensor
        altitude -= relPosSensorEF.z;
    }

    float voltage = 5.0f;  // Start the reading at max value = 5V
    // If the attidude is non reversed for SITL OR we are using rangefinder from external simulator,
    // We adjust the reading with noise, glitch and scaler as the reading is on analog port.
    if ((fabs(_sitl->state.rollDeg) < 90.0 && fabs(_sitl->state.pitchDeg) < 90.0) || !is_equal(range_value, -1.0f)) {
        if (is_equal(range_value, -1.0f)) {  // disable for external reading that already handle this
            // adjust for apparent altitude with roll
            altitude /= cosf(radians(_sitl->state.rollDeg)) * cosf(radians(_sitl->state.pitchDeg));
        }
        // Add some noise on reading
        altitude += _sitl->sonar_noise * rand_float();

        // Altitude in in m, scaler in meters/volt
        voltage = altitude / _sitl->sonar_scale;
        // constrain to 0-5V
        voltage = constrain_float(voltage, 0.0f, 5.0f);

        // Use glitch defines as the probablility between 0-1 that any given sonar sample will read as max distance
        if (!is_zero(_sitl->sonar_glitch) && _sitl->sonar_glitch >= (rand_float() + 1.0f) / 2.0f) {
            voltage = 5.0f;
        }
    }

    sonar_pin_value = 1023 * (voltage / 5.0f);
}

#endif
