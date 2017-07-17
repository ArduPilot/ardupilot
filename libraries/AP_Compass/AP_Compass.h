#pragma once

#include <inttypes.h>

#include <AP_Common/AP_Common.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#include "CompassCalibrator.h"
#include "AP_Compass_Backend.h"

// motor compensation types (for use with motor_comp_enabled)
#define AP_COMPASS_MOT_COMP_DISABLED    0x00
#define AP_COMPASS_MOT_COMP_THROTTLE    0x01
#define AP_COMPASS_MOT_COMP_CURRENT     0x02

// setup default mag orientation for some board types
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX && CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
# define MAG_BOARD_ORIENTATION ROTATION_ROLL_180
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX && CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
# define MAG_BOARD_ORIENTATION ROTATION_YAW_90
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX && (CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBRAIN2 || \
      CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXFMINI)
# define MAG_BOARD_ORIENTATION ROTATION_YAW_270
#else
# define MAG_BOARD_ORIENTATION ROTATION_NONE
#endif

// define default compass calibration fitness and consistency checks
#define AP_COMPASS_CALIBRATION_FITNESS_DEFAULT 16.0f
#define AP_COMPASS_MAX_XYZ_ANG_DIFF radians(90.0f)
#define AP_COMPASS_MAX_XY_ANG_DIFF radians(60.0f)
#define AP_COMPASS_MAX_XY_LENGTH_DIFF 200.0f

/**
   maximum number of compass instances available on this platform. If more
   than 1 then redundant sensors may be available
 */
#define COMPASS_MAX_INSTANCES 3
#define COMPASS_MAX_BACKEND   3

class Compass
{
friend class AP_Compass_Backend;
public:
    /// Constructor
    ///
    Compass();

    /// Initialize the compass device.
    ///
    /// @returns    True if the compass was initialized OK, false if it was not
    ///             found or is not functioning.
    ///
    bool init();

    /// Read the compass and update the mag_ variables.
    ///
    bool read();

    /// use spare CPU cycles to accumulate values from the compass if
    /// possible (this method should also be implemented in the backends)
    void accumulate();

    /// Calculate the tilt-compensated heading_ variables.
    ///
    /// @param dcm_matrix			The current orientation rotation matrix
    ///
    /// @returns heading in radians
    ///
    float calculate_heading(const Matrix3f &dcm_matrix) const {
        return calculate_heading(dcm_matrix, get_primary());
    }
    float calculate_heading(const Matrix3f &dcm_matrix, uint8_t i) const;

    /// Sets offset x/y/z values.
    ///
    /// @param  i                   compass instance
    /// @param  offsets             Offsets to the raw mag_ values in milligauss.
    ///
    void set_offsets(uint8_t i, const Vector3f &offsets);

    /// Sets and saves the compass offset x/y/z values.
    ///
    /// @param  i                   compass instance
    /// @param  offsets             Offsets to the raw mag_ values in milligauss.
    ///
    void set_and_save_offsets(uint8_t i, const Vector3f &offsets);
    void set_and_save_diagonals(uint8_t i, const Vector3f &diagonals);
    void set_and_save_offdiagonals(uint8_t i, const Vector3f &diagonals);

    /// Saves the current offset x/y/z values for one or all compasses
    ///
    /// @param  i                   compass instance
    ///
    /// This should be invoked periodically to save the offset values maintained by
    /// ::learn_offsets.
    ///
    void save_offsets(uint8_t i);
    void save_offsets(void);

    // return the number of compass instances
    uint8_t get_count(void) const { return _compass_count; }

    /// Return the current field as a Vector3f in milligauss
    const Vector3f &get_field(uint8_t i) const { return _state[i].field; }
    const Vector3f &get_field(void) const { return get_field(get_primary()); }

    // compass calibrator interface
    void compass_cal_update();

    void start_calibration_all(bool retry=false, bool autosave=false, float delay_sec=0.0f, bool autoreboot = false);

    void cancel_calibration_all();

    bool compass_cal_requires_reboot() { return _cal_complete_requires_reboot; }
    bool is_calibrating() const;

    /*
      handle an incoming MAG_CAL command
    */
    MAV_RESULT handle_mag_cal_command(const mavlink_command_long_t &packet);

    void send_mag_cal_progress(mavlink_channel_t chan);
    void send_mag_cal_report(mavlink_channel_t chan);

    // check if the compasses are pointing in the same direction
    bool consistent() const;

    /// Return the health of a compass
    bool healthy(uint8_t i) const { return _state[i].healthy; }
    bool healthy(void) const { return healthy(get_primary()); }
    uint8_t get_healthy_mask() const;

    /// Returns the current offset values
    ///
    /// @returns                    The current compass offsets in milligauss.
    ///
    const Vector3f &get_offsets(uint8_t i) const { return _state[i].offset; }
    const Vector3f &get_offsets(void) const { return get_offsets(get_primary()); }

    /// Sets the initial location used to get declination
    ///
    /// @param  latitude             GPS Latitude.
    /// @param  longitude            GPS Longitude.
    ///
    void set_initial_location(int32_t latitude, int32_t longitude);

    /// Program new offset values.
    ///
    /// @param  i                   compass instance
    /// @param  x                   Offset to the raw mag_x value in milligauss.
    /// @param  y                   Offset to the raw mag_y value in milligauss.
    /// @param  z                   Offset to the raw mag_z value in milligauss.
    ///
    void set_and_save_offsets(uint8_t i, int x, int y, int z) {
        set_and_save_offsets(i, Vector3f(x, y, z));
    }

    // learn offsets accessor
    bool learn_offsets_enabled() const { return _learn; }

    /// Perform automatic offset updates
    ///
    void learn_offsets(void);

    /// return true if the compass should be used for yaw calculations
    bool use_for_yaw(uint8_t i) const;
    bool use_for_yaw(void) const;

    /// Sets the local magnetic field declination.
    ///
    /// @param  radians             Local field declination.
    /// @param save_to_eeprom       true to save to eeprom (false saves only to memory)
    ///
    void set_declination(float radians, bool save_to_eeprom = true);
    float get_declination() const;

    // set overall board orientation
    void set_board_orientation(enum Rotation orientation) {
        _board_orientation = orientation;
    }

    /// Set the motor compensation type
    ///
    /// @param  comp_type           0 = disabled, 1 = enabled use throttle, 2 = enabled use current
    ///
    void motor_compensation_type(const uint8_t comp_type);

    /// get the motor compensation value.
    uint8_t get_motor_compensation_type() const {
        return _motor_comp_type;
    }

    /// Set the motor compensation factor x/y/z values.
    ///
    /// @param  i                   instance of compass
    /// @param  offsets             Offsets multiplied by the throttle value and added to the raw mag_ values.
    ///
    void set_motor_compensation(uint8_t i, const Vector3f &motor_comp_factor);

    /// get motor compensation factors as a vector
    const Vector3f& get_motor_compensation(uint8_t i) const { return _state[i].motor_compensation; }
    const Vector3f& get_motor_compensation(void) const { return get_motor_compensation(get_primary()); }

    /// Saves the current motor compensation x/y/z values.
    ///
    /// This should be invoked periodically to save the offset values calculated by the motor compensation auto learning
    ///
    void save_motor_compensation();

    /// Returns the current motor compensation offset values
    ///
    /// @returns                    The current compass offsets in milligauss.
    ///
    const Vector3f &get_motor_offsets(uint8_t i) const { return _state[i].motor_offset; }
    const Vector3f &get_motor_offsets(void) const { return get_motor_offsets(get_primary()); }

    /// Set the throttle as a percentage from 0.0 to 1.0
    /// @param thr_pct              throttle expressed as a percentage from 0 to 1.0
    void set_throttle(float thr_pct) {
        if (_motor_comp_type == AP_COMPASS_MOT_COMP_THROTTLE) {
            _thr_or_curr = thr_pct;
        }
    }

    /// Set the current used by system in amps
    /// @param amps                 current flowing to the motors expressed in amps
    void set_current(float amps) {
        if (_motor_comp_type == AP_COMPASS_MOT_COMP_CURRENT) {
            _thr_or_curr = amps;
        }
    }

    /// Returns True if the compasses have been configured (i.e. offsets saved)
    ///
    /// @returns                    True if compass has been configured
    ///
    bool configured(uint8_t i);
    bool configured(void);

    /// Returns the instance of the primary compass
    ///
    /// @returns                    the instance number of the primary compass
    ///
    uint8_t get_primary(void) const { return _primary; }

    // HIL methods
    void        setHIL(uint8_t instance, float roll, float pitch, float yaw);
    void        setHIL(uint8_t instance, const Vector3f &mag, uint32_t last_update_usec);
    const Vector3f&   getHIL(uint8_t instance) const;
    void        _setup_earth_field();

    // enable HIL mode
    void        set_hil_mode(void) { _hil_mode = true; }

    // return last update time in microseconds
    uint32_t last_update_usec(void) const { return _state[get_primary()].last_update_usec; }
    uint32_t last_update_usec(uint8_t i) const { return _state[i].last_update_usec; }

    uint32_t last_update_ms(void) const { return _state[get_primary()].last_update_ms; }
    uint32_t last_update_ms(uint8_t i) const { return _state[i].last_update_ms; }

    static const struct AP_Param::GroupInfo var_info[];

    // HIL variables
    struct {
        Vector3f Bearth;
        float last_declination;
        bool healthy[COMPASS_MAX_INSTANCES];
        Vector3f field[COMPASS_MAX_INSTANCES];
    } _hil;

    enum LearnType {
        LEARN_NONE=0,
        LEARN_INTERNAL=1,
        LEARN_EKF=2
    };

    // return the chosen learning type
    enum LearnType get_learn_type(void) const {
        return (enum LearnType)_learn.get();
    }

    // return maximum allowed compass offsets
    uint16_t get_offsets_max(void) const {
        return (uint16_t)_offset_max.get();
    }

private:
    /// Register a new compas driver, allocating an instance number
    ///
    /// @return number of compass instances
    uint8_t register_compass(void);

    // load backend drivers
    bool _add_backend(AP_Compass_Backend *backend, const char *name, bool external);
    void _detect_backends(void);

    // compass cal
    bool _accept_calibration(uint8_t i);
    bool _accept_calibration_mask(uint8_t mask);
    void _cancel_calibration(uint8_t i);
    void _cancel_calibration_mask(uint8_t mask);
    uint8_t _get_cal_mask() const;
    bool _start_calibration(uint8_t i, bool retry=false, float delay_sec=0.0f);
    bool _start_calibration_mask(uint8_t mask, bool retry=false, bool autosave=false, float delay_sec=0.0f, bool autoreboot=false);
    bool _auto_reboot() { return _compass_cal_autoreboot; }


    //keep track of which calibrators have been saved
    bool _cal_saved[COMPASS_MAX_INSTANCES];
    bool _cal_autosave;

    //autoreboot after compass calibration
    bool _compass_cal_autoreboot;
    bool _cal_complete_requires_reboot;
    bool _cal_has_run;

    // backend objects
    AP_Compass_Backend *_backends[COMPASS_MAX_BACKEND];
    uint8_t     _backend_count;

    // number of registered compasses.
    uint8_t     _compass_count;

    // settable parameters
    AP_Int8 _learn;

    // board orientation from AHRS
    enum Rotation _board_orientation;

    // primary instance
    AP_Int8     _primary;

    // declination in radians
    AP_Float    _declination;

    // enable automatic declination code
    AP_Int8     _auto_declination;

    // first-time-around flag used by offset nulling
    bool        _null_init_done;

    // used by offset correction
    static const uint8_t _mag_history_size = 20;

    // motor compensation type
    // 0 = disabled, 1 = enabled for throttle, 2 = enabled for current
    AP_Int8     _motor_comp_type;

    // throttle expressed as a percentage from 0 ~ 1.0 or current expressed in amps
    float       _thr_or_curr;

    struct mag_state {
        AP_Int8     external;
        bool        healthy;
        AP_Int8     orientation;
        AP_Vector3f offset;
        AP_Vector3f diagonals;
        AP_Vector3f offdiagonals;

        // device id detected at init.
        // saved to eeprom when offsets are saved allowing ram &
        // eeprom values to be compared as consistency check
        AP_Int32    dev_id;

        AP_Int8     use_for_yaw;

        uint8_t     mag_history_index;
        Vector3i    mag_history[_mag_history_size];

        // factors multiplied by throttle and added to compass outputs
        AP_Vector3f motor_compensation;

        // latest compensation added to compass
        Vector3f    motor_offset;

        // corrected magnetic field strength
        Vector3f    field;

        // when we last got data
        uint32_t    last_update_ms;
        uint32_t    last_update_usec;

        // board specific orientation
        enum Rotation rotation;
    } _state[COMPASS_MAX_INSTANCES];

    AP_Int16 _offset_max;

    CompassCalibrator _calibrator[COMPASS_MAX_INSTANCES];

    // if we want HIL only
    bool _hil_mode:1;

    AP_Float _calibration_threshold;
};
