/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_Progmem.h>
#include "Compass.h"

const AP_Param::GroupInfo Compass::var_info[] PROGMEM = {
    // index 0 was used for the old orientation matrix

    // @Param: OFS_X
    // @DisplayName: Compass offsets on the X axis
    // @Description: Offset to be added to the compass x-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Increment: 1

    // @Param: OFS_Y
    // @DisplayName: Compass offsets on the Y axis
    // @Description: Offset to be added to the compass y-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Increment: 1

    // @Param: OFS_Z
    // @DisplayName: Compass offsets on the Z axis
    // @Description: Offset to be added to the compass z-axis values to compensate for metal in the frame
    // @Range: -400 400
    // @Increment: 1
    AP_GROUPINFO("OFS",    1, Compass, _offset, 0),

    // @Param: DEC
    // @DisplayName: Compass declination
    // @Description: An angle to compensate between the true north and magnetic north
    // @Range: -3.142 3.142
    // @Units: Radians
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("DEC",    2, Compass, _declination, 0),

    // @Param: LEARN
    // @DisplayName: Learn compass offsets automatically
    // @Description: Enable or disable the automatic learning of compass offsets
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("LEARN",  3, Compass, _learn, 1), // true if learning calibration

    // @Param: USE
    // @DisplayName: Use compass for yaw
    // @Description: Enable or disable the use of the compass (instead of the GPS) for determining heading
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("USE",    4, Compass, _use_for_yaw, 1), // true if used for DCM yaw

#if !defined( __AVR_ATmega1280__ )
    // @Param: AUTODEC
    // @DisplayName: Auto Declination
    // @Description: Enable or disable the automatic calculation of the declination based on gps location
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("AUTODEC",5, Compass, _auto_declination, 1),
#endif
    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
Compass::Compass(void) :
    product_id(AP_COMPASS_TYPE_UNKNOWN),
    _orientation(ROTATION_NONE),
    _null_init_done(false)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// Default init method, just returns success.
//
bool
Compass::init()
{
    return true;
}

void
Compass::set_orientation(enum Rotation rotation)
{
    _orientation = rotation;
}

void
Compass::set_offsets(const Vector3f &offsets)
{
    _offset.set(offsets);
}

void
Compass::save_offsets()
{
    _offset.save();
}

Vector3f &
Compass::get_offsets()
{
    return _offset;
}

void
Compass::set_initial_location(int32_t latitude, int32_t longitude)
{
    // if automatic declination is configured, then compute
    // the declination based on the initial GPS fix
#if !defined( __AVR_ATmega1280__ )
    if (_auto_declination) {
        // Set the declination based on the lat/lng from GPS
        _declination.set(radians(
                AP_Declination::get_declination(
                    (float)latitude / 10000000,
                    (float)longitude / 10000000)));
    }
#endif
}

void
Compass::set_declination(float radians)
{
    _declination.set_and_save(radians);
}

float
Compass::get_declination()
{
    return _declination.get();
}


float
Compass::calculate_heading(float roll, float pitch)
{
//  Note - This function implementation is deprecated
//  The alternate implementation of this function using the dcm matrix is preferred
    float headX;
    float headY;
    float cos_roll;
    float sin_roll;
    float cos_pitch;
    float sin_pitch;
    float heading;

    cos_roll = cos(roll);
    sin_roll = sin(roll);
    cos_pitch = cos(pitch);
    sin_pitch = sin(pitch);

    // Tilt compensated magnetic field X component:
    headX = mag_x*cos_pitch + mag_y*sin_roll*sin_pitch + mag_z*cos_roll*sin_pitch;
    // Tilt compensated magnetic field Y component:
    headY = mag_y*cos_roll - mag_z*sin_roll;
    // magnetic heading
    heading = atan2(-headY,headX);

    // Declination correction (if supplied)
    if( fabs(_declination) > 0.0 )
    {
        heading = heading + _declination;
        if (heading > M_PI)    // Angle normalization (-180 deg, 180 deg)
            heading -= (2.0 * M_PI);
        else if (heading < -M_PI)
            heading += (2.0 * M_PI);
    }

    return heading;
}


float
Compass::calculate_heading(const Matrix3f &dcm_matrix)
{
    float headX;
    float headY;
    float cos_pitch = safe_sqrt(1-(dcm_matrix.c.x*dcm_matrix.c.x));
    float heading;

    // sin(pitch) = - dcm_matrix(3,1)
    // cos(pitch)*sin(roll) = - dcm_matrix(3,2)
    // cos(pitch)*cos(roll) = - dcm_matrix(3,3)

    if (cos_pitch == 0.0) {
        // we are pointing straight up or down so don't update our
        // heading using the compass. Wait for the next iteration when
        // we hopefully will have valid values again.
        return 0;
    }

    // Tilt compensated magnetic field X component:
    headX = mag_x*cos_pitch - mag_y*dcm_matrix.c.y*dcm_matrix.c.x/cos_pitch - mag_z*dcm_matrix.c.z*dcm_matrix.c.x/cos_pitch;
    // Tilt compensated magnetic field Y component:
    headY = mag_y*dcm_matrix.c.z/cos_pitch - mag_z*dcm_matrix.c.y/cos_pitch;
    // magnetic heading
    // 6/4/11 - added constrain to keep bad values from ruining DCM Yaw - Jason S.
    heading = constrain(atan2(-headY,headX), -3.15, 3.15);

    // Declination correction (if supplied)
    if( fabs(_declination) > 0.0 )
    {
        heading = heading + _declination;
        if (heading > M_PI)    // Angle normalization (-180 deg, 180 deg)
            heading -= (2.0 * M_PI);
        else if (heading < -M_PI)
            heading += (2.0 * M_PI);
    }

    return heading;
}


/*
 *  this offset nulling algorithm is inspired by this paper from Bill Premerlani
 *
 *  http://gentlenav.googlecode.com/files/MagnetometerOffsetNullingRevisited.pdf
 *
 *  The base algorithm works well, but is quite sensitive to
 *  noise. After long discussions with Bill, the following changes were
 *  made:
 *
 *   1) we keep a history buffer that effectively divides the mag
 *      vectors into a set of N streams. The algorithm is run on the
 *      streams separately
 *
 *   2) within each stream we only calculate a change when the mag
 *      vector has changed by a significant amount.
 *
 *  This gives us the property that we learn quickly if there is no
 *  noise, but still learn correctly (and slowly) in the face of lots of
 *  noise.
 */
void
Compass::null_offsets(void)
{
    if (_learn == 0) {
        // auto-calibration is disabled
        return;
    }

    // this gain is set so we converge on the offsets in about 5
    // minutes with a 10Hz compass
    const float gain = 0.01;
    const float max_change = 10.0;
    const float min_diff = 50.0;
    Vector3f ofs;

    ofs = _offset.get();

    if (!_null_init_done) {
        // first time through
        _null_init_done = true;
        for (uint8_t i=0; i<_mag_history_size; i++) {
            // fill the history buffer with the current mag vector,
            // with the offset removed
            _mag_history[i] = Vector3i((mag_x+0.5) - ofs.x, (mag_y+0.5) - ofs.y, (mag_z+0.5) - ofs.z);
        }
        _mag_history_index = 0;
        return;
    }

    Vector3f b1, b2, diff;
    float length;

    // get a past element
    b1 = Vector3f(_mag_history[_mag_history_index].x,
                  _mag_history[_mag_history_index].y,
                  _mag_history[_mag_history_index].z);
    // the history buffer doesn't have the offsets
    b1 += ofs;

    // get the current vector
    b2 = Vector3f(mag_x, mag_y, mag_z);

    // calculate the delta for this sample
    diff = b2 - b1;
    length = diff.length();
    if (length < min_diff) {
        // the mag vector hasn't changed enough - we don't get
        // enough information from this vector to use it.
        // Note that we don't put the current vector into the mag
        // history here. We want to wait for a larger rotation to
        // build up before calculating an offset change, as accuracy
        // of the offset change is highly dependent on the size of the
        // rotation.
        _mag_history_index = (_mag_history_index + 1) % _mag_history_size;
        return;
    }

    // put the vector in the history
    _mag_history[_mag_history_index] = Vector3i((mag_x+0.5) - ofs.x, (mag_y+0.5) - ofs.y, (mag_z+0.5) - ofs.z);
    _mag_history_index = (_mag_history_index + 1) % _mag_history_size;

    // equation 6 of Bills paper
    diff = diff * (gain * (b2.length() - b1.length()) / length);

    // limit the change from any one reading. This is to prevent
    // single crazy readings from throwing off the offsets for a long
    // time
    length = diff.length();
    if (length > max_change) {
        diff *= max_change / length;
    }

    // set the new offsets
    _offset.set(_offset.get() - diff);
}
