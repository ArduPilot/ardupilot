/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef Compass_h
#define Compass_h

#include <inttypes.h>
#include "../AP_Common/AP_Common.h"
#include "../AP_Math/AP_Math.h"

// compass product id
#define AP_COMPASS_TYPE_UNKNOWN  0x00
#define AP_COMPASS_TYPE_HIL      0x01
#define AP_COMPASS_TYPE_HMC5843  0x02
#define AP_COMPASS_TYPE_HMC5883L 0x03

// standard rotation matrices
#define ROTATION_NONE               Matrix3f(1, 0, 0, 0, 1, 0, 0 ,0, 1)
#define ROTATION_YAW_45             Matrix3f(0.70710678, -0.70710678, 0, 0.70710678, 0.70710678, 0, 0, 0, 1)
#define ROTATION_YAW_90             Matrix3f(0, -1, 0, 1, 0, 0, 0, 0, 1)
#define ROTATION_YAW_135            Matrix3f(-0.70710678, -0.70710678, 0, 0.70710678, -0.70710678, 0, 0, 0, 1)
#define ROTATION_YAW_180            Matrix3f(-1, 0, 0, 0, -1, 0, 0, 0, 1)
#define ROTATION_YAW_225            Matrix3f(-0.70710678, 0.70710678, 0, -0.70710678, -0.70710678, 0, 0, 0, 1)
#define ROTATION_YAW_270            Matrix3f(0, 1, 0, -1, 0, 0, 0, 0, 1)
#define ROTATION_YAW_315            Matrix3f(0.70710678, 0.70710678, 0, -0.70710678, 0.70710678, 0, 0, 0, 1)
#define ROTATION_ROLL_180           Matrix3f(1, 0, 0, 0, -1, 0, 0, 0, -1)
#define ROTATION_ROLL_180_YAW_45    Matrix3f(0.70710678, 0.70710678, 0, 0.70710678, -0.70710678, 0, 0, 0, -1)
#define ROTATION_ROLL_180_YAW_90    Matrix3f(0, 1, 0, 1, 0, 0, 0, 0, -1)
#define ROTATION_ROLL_180_YAW_135   Matrix3f(-0.70710678, 0.70710678, 0, 0.70710678, 0.70710678, 0, 0, 0, -1)
#define ROTATION_PITCH_180          Matrix3f(-1, 0, 0, 0, 1, 0, 0, 0, -1)
#define ROTATION_ROLL_180_YAW_225   Matrix3f(-0.70710678, -0.70710678, 0, -0.70710678, 0.70710678, 0, 0, 0, -1)
#define ROTATION_ROLL_180_YAW_270   Matrix3f(0, -1, 0, -1, 0, 0, 0, 0, -1)
#define ROTATION_ROLL_180_YAW_315   Matrix3f(0.70710678, -0.70710678, 0, -0.70710678, -0.70710678, 0, 0, 0, -1)

class Compass
{
public:
	int				product_id;     /// product id
	int             mag_x;          ///< magnetic field strength along the X axis
	int             mag_y;          ///< magnetic field strength along the Y axis
	int             mag_z;          ///< magnetic field strength along the Z axis
	float           heading;        ///< compass heading in radians
	float           heading_x;      ///< compass vector X magnitude
	float           heading_y;      ///< compass vector Y magnitude
	unsigned long   last_update;    ///< millis() time of last update
	bool			healthy;        ///< true if last read OK

	/// Constructor
	///
	/// @param  key         Storage key used for configuration data.
	///
	Compass();

	/// Initialize the compass device.
	///
	/// @returns    True if the compass was initialized OK, false if it was not
	///             found or is not functioning.
	///
	virtual bool init();
	
	/// Read the compass and update the mag_ variables.
	///
	virtual bool read(void) = 0;

	/// Calculate the tilt-compensated heading_ variables.
	///
	/// @param  roll                The current airframe roll angle.
	/// @param  pitch               The current airframe pitch angle.
	///
	virtual void calculate(float roll, float pitch);

	/// Calculate the tilt-compensated heading_ variables.
	///
	/// @param dcm_matrix			The current orientation rotation matrix
	///
	virtual void calculate(const Matrix3f &dcm_matrix);

	/// Set the compass orientation matrix, used to correct for
	/// various compass mounting positions.
	///
	/// @param  rotation_matrix     Rotation matrix to transform magnetometer readings
	///                             to the body frame.
	///
	virtual void set_orientation(const Matrix3f &rotation_matrix);

	/// Sets the compass offset x/y/z values.
	///
	/// @param  offsets             Offsets to the raw mag_ values.
	///
	virtual void set_offsets(const Vector3f &offsets);

	/// Saves the current compass offset x/y/z values.
	///
	/// This should be invoked periodically to save the offset values maintained by
	/// ::null_offsets.
	///
	virtual void save_offsets();

	/// Returns the current offset values
	///
	/// @returns                    The current compass offsets.
	///
	virtual Vector3f &get_offsets();

	/// Program new offset values.
	///
	/// @param  x                   Offset to the raw mag_x value.
	/// @param  y                   Offset to the raw mag_y value.
	/// @param  z                   Offset to the raw mag_z value.
	///
	void set_offsets(int x, int y, int z) { set_offsets(Vector3f(x, y, z)); }

	/// Perform automatic offset updates using the results of the DCM matrix.
	///
	/// @param  dcm_matrix          The DCM result matrix.
	///
	void null_offsets(const Matrix3f &dcm_matrix);


	/// Enable/Start automatic offset updates 
	///
	void null_offsets_enable(void);


	/// Disable/Stop automatic offset updates
	///
	void null_offsets_disable(void);


	/// Sets the local magnetic field declination.
	///
	/// @param  radians             Local field declination.
	///
	virtual void set_declination(float radians);
	float get_declination();

    static const struct AP_Param::GroupInfo var_info[];

protected:
	AP_Matrix3f         _orientation_matrix;
	AP_Vector3f         _offset;
	AP_Float            _declination;

	bool                _null_enable;        	///< enabled flag for offset nulling
	bool                _null_init_done;        ///< first-time-around flag used by offset nulling
	Matrix3f            _last_dcm_matrix;       ///< previous DCM matrix used by offset nulling
	Vector3f            _mag_body_last;         ///< ?? used by offset nulling
};
#endif
