/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "Compass.h"

const AP_Param::GroupInfo Compass::var_info[] PROGMEM = {
    // index 0 was used for the old orientation matrix
    AP_GROUPINFO("OFS",    1, Compass, _offset),
    AP_GROUPINFO("DEC",    2, Compass, _declination),
    AP_GROUPINFO("LEARN",  3, Compass, _learn), // true if learning calibration
    AP_GROUPINFO("USE",    4, Compass, _use_for_yaw), // true if used for DCM yaw
    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
Compass::Compass(void) :
	product_id(AP_COMPASS_TYPE_UNKNOWN),
    _declination		(0.0),
    _learn(1),
    _use_for_yaw(1),
    _null_enable(false),
    _null_init_done(false),
    _orientation(ROTATION_NONE)
{
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

bool
Compass::set_initial_location(long latitude, long longitude, bool force)
{
	// If the user has choosen to use auto-declination regardless of the planner value
	// OR
	// If the declination failed to load from the EEPROM (ie. not set by user)
	if(force || !_declination.load())
	{
		// Set the declination based on the lat/lng from GPS
		_declination.set(radians(AP_Declination::get_declination((float)latitude / 10000000, (float)longitude / 10000000)));

		// Reset null offsets
		null_offsets_disable();
		null_offsets_enable();
		return true;
	}
	return false;
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


void
Compass::calculate(float roll, float pitch)
{
//  Note - This function implementation is deprecated
//  The alternate implementation of this function using the dcm matrix is preferred
    float headX;
    float headY;
    float cos_roll;
    float sin_roll;
    float cos_pitch;
    float sin_pitch;
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

    // Optimization for external DCM use. Calculate normalized components
    heading_x = cos(heading);
    heading_y = sin(heading);
}


void
Compass::calculate(const Matrix3f &dcm_matrix)
{
    float headX;
    float headY;
    float cos_pitch = safe_sqrt(1-(dcm_matrix.c.x*dcm_matrix.c.x));
	// sin(pitch) = - dcm_matrix(3,1)
	// cos(pitch)*sin(roll) = - dcm_matrix(3,2)
	// cos(pitch)*cos(roll) = - dcm_matrix(3,3)

    if (cos_pitch == 0.0) {
        // we are pointing straight up or down so don't update our
        // heading using the compass. Wait for the next iteration when
        // we hopefully will have valid values again.
        return;
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

    // Optimization for external DCM use. Calculate normalized components
    heading_x = cos(heading);
    heading_y = sin(heading);

#if 0
    if (isnan(heading_x) || isnan(heading_y)) {
        Serial.printf("COMPASS: c.x %f c.y %f c.z %f cos_pitch %f mag_x %d mag_y %d mag_z %d headX %f headY %f heading %f heading_x %f heading_y %f\n",
                      dcm_matrix.c.x,
                      dcm_matrix.c.y,
                      dcm_matrix.c.x,
                      cos_pitch,
                      (int)mag_x, (int)mag_y, (int)mag_z,
                      headX, headY,
                      heading,
                      heading_x, heading_y);
    }
#endif
}


/*
  this offset nulling algorithm is based on a paper from
   Bill Premerlani

  http://gentlenav.googlecode.com/files/MagnetometerOffsetNullingRevisited.pdf
 */
void
Compass::null_offsets(void)
{
    if (_null_enable == false || _learn == 0) {
        // auto-calibration is disabled
        return;
    }

    Vector3f mag_body_new = Vector3f(mag_x,mag_y,mag_z);

    // this gain is set so we converge on the offsets in about 5
    // minutes with a 10Hz compass
    const float gain = 0.5;
    const float max_change = 2.0;

    if (!_null_init_done) {
        // first time through
        _null_init_done = true;
        _mag_body_last = mag_body_new;
        return;
    }

    Vector3f delta, diff;
    float diff_length, delta_length;

    diff = mag_body_new - _mag_body_last;
    diff_length = diff.length();
    if (diff_length == 0.0) {
        // the mag vector hasn't changed - we don't get any
        // information from this
        return;
    }

    // equation 6 of Bills paper
    delta = diff * (mag_body_new.length() - _mag_body_last.length()) / diff_length;

    // limit the change from any one reading. This is to prevent
    // single crazy readings from throwing off the offsets for a long
    // time
    delta_length = delta.length();
    if (delta_length > max_change) {
        delta *= max_change / delta_length;
    }

    // set the new offsets
    _offset.set(_offset.get() - (delta * gain));

    // remember the last mag vector
    _mag_body_last = mag_body_new;
}


void
Compass::null_offsets_enable(void)
{
	_null_init_done = false;
	_null_enable = true;
}

void
Compass::null_offsets_disable(void)
{
	_null_init_done = false;
	_null_enable = false;
}
