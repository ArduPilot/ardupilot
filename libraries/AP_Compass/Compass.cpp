/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "Compass.h"

const AP_Param::GroupInfo Compass::var_info[] PROGMEM = {
    AP_GROUPINFO("ORIENT", 0, Compass, _orientation_matrix),
    AP_GROUPINFO("OFS",    1, Compass, _offset),
    AP_GROUPINFO("DEC",    2, Compass, _declination),
    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
Compass::Compass(void) :
    _declination		(0.0),
    _null_init_done(false),
    _null_enable(false),
	product_id(AP_COMPASS_TYPE_UNKNOWN)
{
    // Default the orientation matrix to none - will be overridden at group load time
    // if an orientation has previously been saved.
    _orientation_matrix.set(ROTATION_NONE);
}

//_group

// Default init method, just returns success.
//
bool
Compass::init()
{
    return true;
}

void
Compass::set_orientation(const Matrix3f &rotation_matrix)
{
    _orientation_matrix.set_and_save(rotation_matrix);
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
    float cos_pitch = sqrt(1-(dcm_matrix.c.x*dcm_matrix.c.x));
	// sin(pitch) = - dcm_matrix(3,1)
	// cos(pitch)*sin(roll) = - dcm_matrix(3,2)
	// cos(pitch)*cos(roll) = - dcm_matrix(3,3)

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
}

void
Compass::null_offsets(const Matrix3f &dcm_matrix)
{
    // Update our estimate of the offsets in the magnetometer
    Vector3f    calc(0.0, 0.0, 0.0);        // XXX should be safe to remove explicit init here as the default ctor should do the right thing
    Matrix3f    dcm_new_from_last;
    float       weight;

    Vector3f mag_body_new = Vector3f(mag_x,mag_y,mag_z);
    
    if(_null_enable == false) return;

    if(_null_init_done) {
        dcm_new_from_last = dcm_matrix.transposed() * _last_dcm_matrix;      // Note 11/20/2010: transpose() is not working, transposed() is.

        weight = 3.0 - fabs(dcm_new_from_last.a.x) - fabs(dcm_new_from_last.b.y) - fabs(dcm_new_from_last.c.z);
        if (weight > .001) {
            calc = mag_body_new + _mag_body_last;                // Eq 11 from Bill P's paper
            calc -= dcm_new_from_last * _mag_body_last;
            calc -= dcm_new_from_last.transposed() * mag_body_new;
            if(weight > 0.5) weight = 0.5;
            calc = calc * (weight);
            _offset.set(_offset.get() - calc);
        }
    } else {
        _null_init_done = true;
    }
    _mag_body_last = mag_body_new - calc;
    _last_dcm_matrix = dcm_matrix;

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
