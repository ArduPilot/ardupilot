/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef Compass_h
#define Compass_h

#include <inttypes.h>
#include "../AP_Common/AP_Common.h"
#include "../AP_Math/AP_Math.h"
#include "../AP_Declination/AP_Declination.h" // ArduPilot Mega Declination Helper Library

// compass product id
#define AP_COMPASS_TYPE_UNKNOWN  0x00
#define AP_COMPASS_TYPE_HIL      0x01
#define AP_COMPASS_TYPE_HMC5843  0x02
#define AP_COMPASS_TYPE_HMC5883L 0x03

class Compass
{
public:
    int16_t product_id;                         /// product id
    int16_t mag_x;                      ///< magnetic field strength along the X axis
    int16_t mag_y;                      ///< magnetic field strength along the Y axis
    int16_t mag_z;                      ///< magnetic field strength along the Z axis
    uint32_t last_update;               ///< micros() time of last update
    bool healthy;                               ///< true if last read OK

    /// Constructor
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

    /// use spare CPU cycles to accumulate values from the compass if
    /// possible
    virtual void accumulate(void) = 0;

    /// Calculate the tilt-compensated heading_ variables.
    ///
    /// @param  roll                The current airframe roll angle.
    /// @param  pitch               The current airframe pitch angle.
    ///
    /// @returns heading in radians
    ///
    virtual float calculate_heading(float roll, float pitch);

    /// Calculate the tilt-compensated heading_ variables.
    ///
    /// @param dcm_matrix			The current orientation rotation matrix
    ///
    /// @returns heading in radians
    ///
    virtual float calculate_heading(const Matrix3f &dcm_matrix);

    /// Set the compass orientation matrix, used to correct for
    /// various compass mounting positions.
    ///
    /// @param  rotation_matrix     Rotation matrix to transform magnetometer readings
    ///                             to the body frame.
    ///
    virtual void set_orientation(enum Rotation rotation);

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

    /// Sets the initial location used to get declination
    ///
    /// @param  latitude             GPS Latitude.
    /// @param  longitude            GPS Longitude.
    ///
    void set_initial_location(int32_t latitude, int32_t longitude);

    /// Program new offset values.
    ///
    /// @param  x                   Offset to the raw mag_x value.
    /// @param  y                   Offset to the raw mag_y value.
    /// @param  z                   Offset to the raw mag_z value.
    ///
    void set_offsets(int x, int y, int z) {
        set_offsets(Vector3f(x, y, z));
    }

    /// Perform automatic offset updates
    ///
    void null_offsets(void);

    /// return true if the compass should be used for yaw calculations
    bool use_for_yaw(void) {
        return healthy && _use_for_yaw;
    }

    /// Sets the local magnetic field declination.
    ///
    /// @param  radians             Local field declination.
    ///
    virtual void set_declination(float radians);
    float get_declination();

    static const struct AP_Param::GroupInfo var_info[];

protected:
    enum Rotation _orientation;
    AP_Vector3f _offset;
    AP_Float _declination;
    AP_Int8 _learn;                             ///<enable calibration learning
    AP_Int8 _use_for_yaw;                       ///<enable use for yaw calculation
    AP_Int8 _auto_declination;                  ///<enable automatic declination code

    bool _null_init_done;                           ///< first-time-around flag used by offset nulling

    ///< used by offset correction
    static const uint8_t _mag_history_size = 20;
    uint8_t _mag_history_index;
    Vector3i _mag_history[_mag_history_size];
};
#endif
