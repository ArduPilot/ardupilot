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

    // @Param: MOTCT
    // @DisplayName: Motor interference compensation type
    // @Description: Set motor interference compensation type to disabled, throttle or current.  Do not change manually.
    // @Values: 0:Disabled,1:Use Throttle,2:Use Current
    // @Increment: 1
    AP_GROUPINFO("MOTCT",    6, Compass, _motor_comp_type, AP_COMPASS_MOT_COMP_DISABLED),

    // @Param: MOT_X
    // @DisplayName: Motor interference compensation for body frame X axis
    // @Description: Multiplied by the current throttle and added to the compass's x-axis values to compensate for motor interference
    // @Range: -1000 1000
    // @Units: Offset per Amp or at Full Throttle
    // @Increment: 1

    // @Param: MOT_Y
    // @DisplayName: Motor interference compensation for body frame Y axis
    // @Description: Multiplied by the current throttle and added to the compass's y-axis values to compensate for motor interference
    // @Range: -1000 1000
    // @Units: Offset per Amp or at Full Throttle
    // @Increment: 1

    // @Param: MOT_Z
    // @DisplayName: Motor interference compensation for body frame Z axis
    // @Description: Multiplied by the current throttle and added to the compass's z-axis values to compensate for motor interference
    // @Range: -1000 1000
    // @Units: Offset per Amp or at Full Throttle
    // @Increment: 1
    AP_GROUPINFO("MOT",    7, Compass, _motor_compensation, 0),

    // @Param: ORIENT
    // @DisplayName: Compass orientation
    // @Description: The orientation of the compass relative to the autopilot board. This will default to the right value for each board type, but can be changed if you have an external compass. See the documentation for your external compass for the right value. The correct orientation should give the X axis forward, the Y axis to the right and the Z axis down. So if your aircraft is pointing west it should show a position value for the Y axis, and a value close to zero for the X axis. NOTE: This orientation is combined with any AHRS_ORIENTATION setting. 
    // @Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180,9:Roll180Yaw45,10:Roll180Yaw90,11:Roll180Yaw135,12:Pitch180,13:Roll180Yaw225,14:Roll180Yaw270,15:Roll180Yaw315,16:Roll90,17:Roll90Yaw45,18:Roll90Yaw135,19:Roll270,20:Roll270Yaw45,21:Roll270Yaw90,22:Roll270Yaw136,23:Pitch90,24:Pitch270
    AP_GROUPINFO("ORIENT", 8, Compass, _orientation, ROTATION_NONE),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
Compass::Compass(void) :
    product_id(AP_COMPASS_TYPE_UNKNOWN),
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
Compass::set_offsets(const Vector3f &offsets)
{
    _offset.set(offsets);
}

void
Compass::save_offsets()
{
    _offset.save();
}

const Vector3f &
Compass::get_offsets() const
{
    return _offset;
}

void
Compass::set_motor_compensation(const Vector3f &motor_comp_factor)
{
    _motor_compensation.set(motor_comp_factor);
}

void
Compass::save_motor_compensation()
{
    _motor_comp_type.save();
    _motor_compensation.save();
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
Compass::set_declination(float radians, bool save_to_eeprom)
{
    if (save_to_eeprom) {
        _declination.set_and_save(radians);
    }else{
        _declination.set(radians);
    }
}

float
Compass::get_declination() const
{
    return _declination.get();
}


float
Compass::calculate_heading(const Matrix3f &dcm_matrix) const
{
    // Tilt compensated magnetic field Y component:
    float headY = mag_y * dcm_matrix.c.z - mag_z * dcm_matrix.c.y;

    // Tilt compensated magnetic field X component:
    float headX = mag_x + dcm_matrix.c.x * (headY - mag_x * dcm_matrix.c.x);

    // magnetic heading
    // 6/4/11 - added constrain to keep bad values from ruining DCM Yaw - Jason S.
    float heading = constrain_float(atan2f(-headY,headX), -3.15f, 3.15f);

    // Declination correction (if supplied)
    if( fabsf(_declination) > 0.0f )
    {
        heading = heading + _declination;
        if (heading > PI)    // Angle normalization (-180 deg, 180 deg)
            heading -= (2.0f * PI);
        else if (heading < -PI)
            heading += (2.0f * PI);
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
            _mag_history[i] = Vector3i((mag_x+0.5f) - ofs.x, (mag_y+0.5f) - ofs.y, (mag_z+0.5f) - ofs.z);
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
    _mag_history[_mag_history_index] = Vector3i((mag_x+0.5f) - ofs.x, (mag_y+0.5f) - ofs.y, (mag_z+0.5f) - ofs.z);
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
