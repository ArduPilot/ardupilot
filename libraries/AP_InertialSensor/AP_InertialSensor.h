#pragma once

#include "AP_InertialSensor_config.h"

// Gyro and Accelerometer calibration criteria
#define AP_INERTIAL_SENSOR_ACCEL_TOT_MAX_OFFSET_CHANGE  4.0f
#define AP_INERTIAL_SENSOR_ACCEL_MAX_OFFSET             250.0f
#define AP_INERTIAL_SENSOR_ACCEL_VIBE_FLOOR_FILT_HZ     5.0f    // accel vibration floor filter hz
#define AP_INERTIAL_SENSOR_ACCEL_VIBE_FILT_HZ           2.0f    // accel vibration filter hz
#define AP_INERTIAL_SENSOR_ACCEL_PEAK_DETECT_TIMEOUT_MS 500     // peak-hold detector timeout

#include <AP_HAL/AP_HAL_Boards.h>

#include <stdint.h>

#include <AP_AccelCal/AP_AccelCal.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_Math/AP_Math.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <Filter/LowPassFilter.h>
#include <Filter/HarmonicNotchFilter.h>
#include <AP_SerialManager/AP_SerialManager_config.h>
#include "AP_InertialSensor_Params.h"
#include "AP_InertialSensor_tempcal.h"

#ifndef AP_SIM_INS_ENABLED
#define AP_SIM_INS_ENABLED AP_SIM_ENABLED
#endif

#ifndef AP_SIM_INS_FILE_ENABLED
#define AP_SIM_INS_FILE_ENABLED AP_SIM_ENABLED
#endif

class AP_InertialSensor_Backend;
class AuxiliaryBus;
class AP_AHRS;

/*
  forward declare AP_Logger class. We can't include logger.h
  because of mutual dependencies
 */
class AP_Logger;

/* AP_InertialSensor is an abstraction for gyro and accel measurements
 * which are correctly aligned to the body axes and scaled to SI units.
 *
 * Gauss-Newton accel calibration routines borrowed from Rolfe Schmidt
 * blog post describing the method: http://chionophilous.wordpress.com/2011/10/24/accelerometer-calibration-iv-1-implementing-gauss-newton-on-an-atmega/
 * original sketch available at http://rolfeschmidt.com/mathtools/skimetrics/adxl_gn_calibration.pde
 */
class AP_InertialSensor : AP_AccelCal_Client
{
    friend class AP_InertialSensor_Backend;

public:
    AP_InertialSensor();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_InertialSensor);

    static AP_InertialSensor *get_singleton();

    enum Gyro_Calibration_Timing {
        GYRO_CAL_NEVER = 0,
        GYRO_CAL_STARTUP_ONLY = 1
    };

    /// Perform startup initialisation.
    ///
    /// Called to initialise the state of the IMU.
    ///
    /// Gyros will be calibrated unless INS_GYRO_CAL is zero
    ///
    /// @param style	The initialisation startup style.
    ///
    void init(uint16_t sample_rate_hz);

    // get accel/gyro instance numbers that a backend will get when they register
    bool get_accel_instance(uint8_t &instance) const;
    bool get_gyro_instance(uint8_t &instance) const;

    /// Register a new gyro/accel driver, allocating an instance
    /// number
    bool register_gyro(uint8_t &instance, uint16_t raw_sample_rate_hz, uint32_t id);
    bool register_accel(uint8_t &instance, uint16_t raw_sample_rate_hz, uint32_t id);

    // a function called by the main thread at the main loop rate:
    void periodic();

    /// calibrating - returns true if the gyros or accels are currently being calibrated
    bool calibrating() const;

    /// calibrating - returns true if a temperature calibration is running
    bool temperature_cal_running() const;
    
    /// Perform cold-start initialisation for just the gyros.
    ///
    /// @note This should not be called unless ::init has previously
    ///       been called, as ::init may perform other work
    ///
    void init_gyro(void);

    // get startup messages to output to the GCS
    bool get_output_banner(uint8_t instance_id, char* banner, uint8_t banner_len);

    /// Fetch the current gyro values
    ///
    /// @returns	vector of rotational rates in radians/sec
    ///
    const Vector3f     &get_gyro(uint8_t i) const { return _gyro[i]; }
    const Vector3f     &get_gyro(void) const { return get_gyro(_first_usable_gyro); }

    // set gyro offsets in radians/sec
    const Vector3f &get_gyro_offsets(uint8_t i) const { return _gyro_offset(i); }
    const Vector3f &get_gyro_offsets(void) const { return get_gyro_offsets(_first_usable_gyro); }

    //get delta angle if available
    bool get_delta_angle(uint8_t i, Vector3f &delta_angle, float &delta_angle_dt) const;
    bool get_delta_angle(Vector3f &delta_angle, float &delta_angle_dt) const {
        return get_delta_angle(_first_usable_gyro, delta_angle, delta_angle_dt);
    }

    //get delta velocity if available
    bool get_delta_velocity(uint8_t i, Vector3f &delta_velocity, float &delta_velocity_dt) const;
    bool get_delta_velocity(Vector3f &delta_velocity, float &delta_velocity_dt) const {
        return get_delta_velocity(_first_usable_accel, delta_velocity, delta_velocity_dt);
    }

    /// Fetch the current accelerometer values
    ///
    /// @returns	vector of current accelerations in m/s/s
    ///
    const Vector3f     &get_accel(uint8_t i) const { return _accel[i]; }
    const Vector3f     &get_accel(void) const { return get_accel(_first_usable_accel); }

    // multi-device interface
    bool get_gyro_health(uint8_t instance) const { return (instance<_gyro_count) ? _gyro_healthy[instance] : false; }
    bool get_gyro_health(void) const { return get_gyro_health(_first_usable_gyro); }
    bool get_gyro_health_all(void) const;
    bool gyros_consistent(uint8_t threshold) const;
    uint8_t get_gyro_count(void) const { return MIN(INS_MAX_INSTANCES, _gyro_count); }
    bool gyro_calibrated_ok(uint8_t instance) const { return _gyro_cal_ok[instance]; }
    bool gyro_calibrated_ok_all() const;
    bool use_gyro(uint8_t instance) const;
    Gyro_Calibration_Timing gyro_calibration_timing();

    bool get_accel_health(uint8_t instance) const { return (instance<_accel_count) ? _accel_healthy[instance] : false; }
    bool get_accel_health(void) const { return get_accel_health(_first_usable_accel); }
    bool get_accel_health_all(void) const;
    bool accels_consistent(float accel_error_threshold) const;
    uint8_t get_accel_count(void) const { return MIN(INS_MAX_INSTANCES, _accel_count); }
    bool accel_calibrated_ok_all() const;
    bool use_accel(uint8_t instance) const;

    // get observed sensor rates, including any internal sampling multiplier
    uint16_t get_gyro_rate_hz(uint8_t instance) const { return uint16_t(_gyro_raw_sample_rates[instance] * _gyro_over_sampling[instance]); }
    uint16_t get_accel_rate_hz(uint8_t instance) const { return uint16_t(_accel_raw_sample_rates[instance] * _accel_over_sampling[instance]); }

    // FFT support access
#if HAL_GYROFFT_ENABLED
    const Vector3f& get_gyro_for_fft(void) const { return _gyro_for_fft[_first_usable_gyro]; }
    FloatBuffer&  get_raw_gyro_window(uint8_t instance, uint8_t axis) { return _gyro_window[instance][axis]; }
    FloatBuffer&  get_raw_gyro_window(uint8_t axis) { return get_raw_gyro_window(_first_usable_gyro, axis); }
    uint16_t get_raw_gyro_rate_hz() const { return get_raw_gyro_rate_hz(_first_usable_gyro); }
    uint16_t get_raw_gyro_rate_hz(uint8_t instance) const { return _gyro_raw_sample_rates[_first_usable_gyro]; }
#if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
    bool has_fft_notch() const;
#endif
#endif
    bool set_gyro_window_size(uint16_t size);
    // get accel offsets in m/s/s
    const Vector3f &get_accel_offsets(uint8_t i) const { return _accel_offset(i); }
    const Vector3f &get_accel_offsets(void) const { return get_accel_offsets(_first_usable_accel); }

    // get accel scale
    const Vector3f &get_accel_scale(uint8_t i) const { return _accel_scale(i); }
    const Vector3f &get_accel_scale(void) const { return get_accel_scale(_first_usable_accel); }

    // return a 3D vector defining the position offset of the IMU accelerometer in metres relative to the body frame origin
    const Vector3f &get_imu_pos_offset(uint8_t instance) const {
        return _accel_pos(instance);
    }
    const Vector3f &get_imu_pos_offset(void) const {
        return _accel_pos(_first_usable_accel);
    }

    // return the temperature if supported. Zero is returned if no
    // temperature is available
    float get_temperature(uint8_t instance) const { return _temperature[instance]; }

    /* get_delta_time returns the time period in seconds
     * overwhich the sensor data was collected
     */
    float get_delta_time() const { return MIN(_delta_time, _loop_delta_t_max); }

    // return the maximum gyro drift rate in radians/s/s. This
    // depends on what gyro chips are being used
    float get_gyro_drift_rate(void) const { return ToRad(0.5f/60); }

    // update gyro and accel values from accumulated samples
    void update(void) __RAMFUNC__;

    // wait for a sample to be available
    void wait_for_sample(void) __RAMFUNC__;

    // class level parameters
    static const struct AP_Param::GroupInfo var_info[];
#if INS_AUX_INSTANCES
    AP_InertialSensor_Params params[INS_AUX_INSTANCES];
#endif

    // set overall board orientation
    void set_board_orientation(enum Rotation orientation) {
        _board_orientation = orientation;
    }

    // return the selected loop rate at which samples are made avilable
    uint16_t get_loop_rate_hz(void) const { return _loop_rate; }

    // return the main loop delta_t in seconds
    float get_loop_delta_t(void) const { return _loop_delta_t; }

    bool healthy(void) const { return get_gyro_health() && get_accel_health(); }

    uint8_t get_first_usable_accel(void) const { return _first_usable_accel; }
    uint8_t get_first_usable_gyro(void) const { return _first_usable_gyro; }

    // get the gyro filter rate in Hz
    uint16_t get_gyro_filter_hz(void) const { return _gyro_filter_cutoff; }

    // get the accel filter rate in Hz
    uint16_t get_accel_filter_hz(void) const { return _accel_filter_cutoff; }

#if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
    // setup the notch for throttle based tracking
    bool setup_throttle_gyro_harmonic_notch(float center_freq_hz, float lower_freq_hz, float ref, uint8_t harmonics);

    // write out harmonic notch log messages
    void write_notch_log_messages() const;
#endif

    // indicate which bit in LOG_BITMASK indicates raw logging enabled
    void set_log_raw_bit(uint32_t log_raw_bit) { _log_raw_bit = log_raw_bit; }

    // Logging Functions
    void Write_IMU() const;
    void Write_Vibration() const;

    // calculate vibration levels and check for accelerometer clipping (called by a backends)
    void calc_vibration_and_clipping(uint8_t instance, const Vector3f &accel, float dt);

    // retrieve latest calculated vibration levels
    Vector3f get_vibration_levels() const { return get_vibration_levels(_first_usable_accel); }
    Vector3f get_vibration_levels(uint8_t instance) const;

    // retrieve and clear accelerometer clipping count
    uint32_t get_accel_clip_count(uint8_t instance) const;

    // check for vibration movement. True when all axis show nearly zero movement
    bool is_still();

    AuxiliaryBus *get_auxiliary_bus(int16_t backend_id) { return get_auxiliary_bus(backend_id, 0); }
    AuxiliaryBus *get_auxiliary_bus(int16_t backend_id, uint8_t instance);

    void detect_backends(void);

    // accel peak hold detector
    void set_accel_peak_hold(uint8_t instance, const Vector3f &accel);
    float get_accel_peak_hold_neg_x() const { return _peak_hold_state.accel_peak_hold_neg_x; }

    //Returns accel calibrator interface object pointer
    AP_AccelCal* get_acal() const { return _acal; }

    // Returns body fixed accelerometer level data averaged during accel calibration's first step
    bool get_fixed_mount_accel_cal_sample(uint8_t sample_num, Vector3f& ret) const;

    // Returns primary accelerometer level data averaged during accel calibration's first step
    bool get_first_usable_accel_cal_sample_avg(uint8_t sample_num, Vector3f& ret) const;

    // Returns newly calculated trim values if calculated
    bool get_new_trim(Vector3f &trim_rad);

    // notify IMUs of the new primaries
    void set_primary_gyro(uint8_t instance);
    void set_primary_accel(uint8_t instance);

#if HAL_INS_ACCELCAL_ENABLED
    // initialise and register accel calibrator
    // called during the startup of accel cal
    void acal_init();

    // update accel calibrator
    void acal_update();
#endif

#if HAL_GCS_ENABLED
    bool calibrate_gyros();

    MAV_RESULT calibrate_trim();

    // simple accel calibration
    MAV_RESULT simple_accel_cal();
private:
    uint32_t last_accel_cal_ms;
public:
#endif

    bool accel_cal_requires_reboot() const { return _accel_cal_requires_reboot; }

    // return time in microseconds of last update() call
    uint32_t get_last_update_usec(void) const { return _last_update_usec; }

    // for killing an IMU for testing purposes
    void kill_imu(uint8_t imu_idx, bool kill_it);

#if AP_SERIALMANAGER_IMUOUT_ENABLED
    // optional UART for sending IMU data to an external process
    void set_imu_out_uart(AP_HAL::UARTDriver *uart);
    void send_uart_data(void);

    struct {
        uint16_t counter;
        AP_HAL::UARTDriver *imu_out_uart;
    } uart;
#endif // AP_SERIALMANAGER_IMUOUT_ENABLED

    enum IMU_SENSOR_TYPE {
        IMU_SENSOR_TYPE_ACCEL = 0,
        IMU_SENSOR_TYPE_GYRO = 1,
    };

#if AP_INERTIALSENSOR_BATCHSAMPLER_ENABLED
    class BatchSampler {
    public:
        BatchSampler(const AP_InertialSensor &imu) :
            type(IMU_SENSOR_TYPE_ACCEL),
            _imu(imu) {
            AP_Param::setup_object_defaults(this, var_info);
        };

        void init();
        void sample(uint8_t instance, IMU_SENSOR_TYPE _type, uint64_t sample_us, const Vector3f &sample) __RAMFUNC__;

        // a function called by the main thread at the main loop rate:
        void periodic();

        bool doing_sensor_rate_logging() const { return _doing_sensor_rate_logging; }
        bool doing_post_filter_logging() const {
            return (_doing_post_filter_logging && (post_filter || !_doing_sensor_rate_logging))
                || (_doing_pre_post_filter_logging && post_filter);
        }

        // Getters for arming check
        bool is_initialised() const { return initialised; }
        bool enabled() const { return _sensor_mask > 0; }

        // class level parameters
        static const struct AP_Param::GroupInfo var_info[];
    

        // Parameters
        AP_Int16 _required_count;
        uint16_t _real_required_count;
        AP_Int8 _sensor_mask;
        AP_Int8 _batch_options_mask;

        // Parameters controlling pushing data to AP_Logger:
        // Each DF message is ~ 108 bytes in size, so we use about 1kB/s of
        // logging bandwidth with a 100ms interval.  If we are taking
        // 1024 samples then we need to send 32 packets, so it will
        // take ~3 seconds to push a complete batch to the log.  If
        // you are running a on an FMU with three IMUs then you
        // will loop back around to the first sensor after about
        // twenty seconds.
        AP_Int16 samples_per_msg;
        AP_Int8 push_interval_ms;

        // end Parameters

    private:

        enum batch_opt_t {
            BATCH_OPT_SENSOR_RATE = (1<<0),
            BATCH_OPT_POST_FILTER = (1<<1),
            BATCH_OPT_PRE_POST_FILTER = (1<<2),
        };

        void rotate_to_next_sensor();
        void update_doing_sensor_rate_logging();

        bool should_log(uint8_t instance, IMU_SENSOR_TYPE type) __RAMFUNC__;
        void push_data_to_log();

        // Logging functions
        bool Write_ISBH(const float sample_rate_hz) const;
        bool Write_ISBD() const;

        bool has_option(batch_opt_t option) const { return _batch_options_mask & uint16_t(option); }

        uint64_t measurement_started_us;

        bool initialised;
        bool isbh_sent;
        bool _doing_sensor_rate_logging;
        bool _doing_post_filter_logging;
        bool _doing_pre_post_filter_logging;
        uint8_t instance; // instance we are sending data for
        bool post_filter; // whether we are sending post-filter data
        AP_InertialSensor::IMU_SENSOR_TYPE type;
        uint16_t isb_seqnum;
        int16_t *data_x;
        int16_t *data_y;
        int16_t *data_z;
        uint16_t data_write_offset; // units: samples
        uint16_t data_read_offset; // units: samples
        uint32_t last_sent_ms;

        // all samples are multiplied by this
        uint16_t multiplier; // initialised as part of init()

        const AP_InertialSensor &_imu;
    };
    BatchSampler batchsampler{*this};
#endif

#if HAL_EXTERNAL_AHRS_ENABLED
    // handle external AHRS data
    void handle_external(const AP_ExternalAHRS::ins_data_message_t &pkt);
#endif

#if HAL_INS_TEMPERATURE_CAL_ENABLE
    /*
      get a string representation of parameters that should be made
      persistent across changes of firmware type
     */
    void get_persistent_params(ExpandingString &str) const;
#endif

    // force save of current calibration as valid
    void force_save_calibration(void);

#if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
    // structure per harmonic notch filter. This is public to allow for
    // easy iteration
    class HarmonicNotch {
    public:
        HarmonicNotchFilterParams params;
        HarmonicNotchFilterVector3f filter[INS_MAX_INSTANCES];

        uint8_t num_dynamic_notches;

        // the current center frequency for the notch
        float calculated_notch_freq_hz[INS_MAX_NOTCHES];
        uint8_t num_calculated_notch_frequencies;

        // runtime update of notch parameters
        void update_params(uint8_t instance, bool converging, float gyro_rate);

        // Update the harmonic notch frequencies
        void update_freq_hz(float scaled_freq);
        void update_frequencies_hz(uint8_t num_freqs, const float scaled_freq[]);

        // enable/disable the notch
        void set_inactive(bool _inactive) {
            inactive = _inactive;
        }

        bool is_inactive(void) const {
            return inactive;
        }

    private:
        // support for updating harmonic filter at runtime
        float last_center_freq_hz[INS_MAX_INSTANCES];
        float last_bandwidth_hz[INS_MAX_INSTANCES];
        float last_attenuation_dB[INS_MAX_INSTANCES];
        bool inactive;
    } harmonic_notches[HAL_INS_NUM_HARMONIC_NOTCH_FILTERS];
#endif  // AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED

private:
    // load backend drivers
    bool _add_backend(AP_InertialSensor_Backend *backend);
    void _start_backends();
    AP_InertialSensor_Backend *_find_backend(int16_t backend_id, uint8_t instance);

    // gyro initialisation
    void _init_gyro();

    // Calibration routines borrowed from Rolfe Schmidt
    // blog post describing the method: http://chionophilous.wordpress.com/2011/10/24/accelerometer-calibration-iv-1-implementing-gauss-newton-on-an-atmega/
    // original sketch available at http://rolfeschmidt.com/mathtools/skimetrics/adxl_gn_calibration.pde

    bool _calculate_trim(const Vector3f &accel_sample, Vector3f &trim_rad);

    // save gyro calibration values to eeprom
    void _save_gyro_calibration();

    // Logging function
    void Write_IMU_instance(const uint64_t time_us, const uint8_t imu_instance) const;
    
    // backend objects
    AP_InertialSensor_Backend *_backends[INS_MAX_BACKENDS];

    // number of gyros and accel drivers. Note that most backends
    // provide both accel and gyro data, so will increment both
    // counters on initialisation
    uint8_t _gyro_count;
    uint8_t _accel_count;
    uint8_t _backend_count;

    // the selected loop rate at which samples are made available
    uint16_t _loop_rate;
    float _loop_delta_t;
    float _loop_delta_t_max;

    // Most recent accelerometer reading
    Vector3f _accel[INS_MAX_INSTANCES];
    Vector3f _delta_velocity[INS_MAX_INSTANCES];
    float _delta_velocity_dt[INS_MAX_INSTANCES];
    bool _delta_velocity_valid[INS_MAX_INSTANCES];
    // delta velocity accumulator
    Vector3f _delta_velocity_acc[INS_MAX_INSTANCES];
    // time accumulator for delta velocity accumulator
    float _delta_velocity_acc_dt[INS_MAX_INSTANCES];

    // Low Pass filters for gyro and accel
    LowPassFilter2pVector3f _accel_filter[INS_MAX_INSTANCES];
    LowPassFilter2pVector3f _gyro_filter[INS_MAX_INSTANCES];
    Vector3f _accel_filtered[INS_MAX_INSTANCES];
    Vector3f _gyro_filtered[INS_MAX_INSTANCES];
#if HAL_GYROFFT_ENABLED
    // Thread-safe public version of _last_raw_gyro
    Vector3f _gyro_for_fft[INS_MAX_INSTANCES];
    Vector3f _last_gyro_for_fft[INS_MAX_INSTANCES];
    FloatBuffer _gyro_window[INS_MAX_INSTANCES][XYZ_AXIS_COUNT];
    uint16_t _gyro_window_size;
    // capture a gyro window after the filters
    LowPassFilter2pVector3f _post_filter_gyro_filter[INS_MAX_INSTANCES];
    bool _post_filter_fft;
    uint8_t _fft_window_phase;
#endif
    bool _new_accel_data[INS_MAX_INSTANCES];
    bool _new_gyro_data[INS_MAX_INSTANCES];

    // Most recent gyro reading
    Vector3f _gyro[INS_MAX_INSTANCES];
    Vector3f _delta_angle[INS_MAX_INSTANCES];
    float _delta_angle_dt[INS_MAX_INSTANCES];
    bool _delta_angle_valid[INS_MAX_INSTANCES];
    // time accumulator for delta angle accumulator
    float _delta_angle_acc_dt[INS_MAX_INSTANCES];
    Vector3f _delta_angle_acc[INS_MAX_INSTANCES];
    Vector3f _last_delta_angle[INS_MAX_INSTANCES];
    Vector3f _last_raw_gyro[INS_MAX_INSTANCES];

    // bitmask indicating if a sensor is doing sensor-rate sampling:
    uint8_t _accel_sensor_rate_sampling_enabled;
    uint8_t _gyro_sensor_rate_sampling_enabled;

    // multipliers for data supplied via sensor-rate logging:
    uint16_t _accel_raw_sampling_multiplier[INS_MAX_INSTANCES];
    uint16_t _gyro_raw_sampling_multiplier[INS_MAX_INSTANCES];

    // IDs to uniquely identify each sensor: shall remain
    // the same across reboots
    AP_Int32 _accel_id_old_param[INS_MAX_INSTANCES-INS_AUX_INSTANCES];
    AP_Int32 _gyro_id_old_param[INS_MAX_INSTANCES-INS_AUX_INSTANCES];

    // accelerometer scaling and offsets
    AP_Vector3f _accel_scale_old_param[INS_MAX_INSTANCES-INS_AUX_INSTANCES];
    AP_Vector3f _accel_offset_old_param[INS_MAX_INSTANCES-INS_AUX_INSTANCES];
    AP_Vector3f _gyro_offset_old_param[INS_MAX_INSTANCES-INS_AUX_INSTANCES];

    // accelerometer position offset in body frame
    AP_Vector3f _accel_pos_old_param[INS_MAX_INSTANCES-INS_AUX_INSTANCES];

    // Use Accessor methods to access above variables
#if INS_AUX_INSTANCES
    #define INS_PARAM_WRAPPER(var) \
        inline decltype(var##_old_param[0])& var(uint8_t i) { \
            if (i<(INS_MAX_INSTANCES-INS_AUX_INSTANCES)) { \
                return var##_old_param[i]; \
            } else { \
                return params[i-(INS_MAX_INSTANCES-INS_AUX_INSTANCES)].var; \
            } \
        } \
        inline decltype(var##_old_param[0])& var(uint8_t i) const { \
            return const_cast<AP_InertialSensor*>(this)->var(i); \
        }
#else
    #define INS_PARAM_WRAPPER(var) \
        inline decltype(var##_old_param[0])& var(uint8_t i) { \
            return var##_old_param[i]; \
        } \
        inline decltype(var##_old_param[0])& var(uint8_t i) const { \
            return const_cast<AP_InertialSensor*>(this)->var(i); \
        }
#endif

    // Accessor methods for old parameters
    INS_PARAM_WRAPPER(_accel_id);
    INS_PARAM_WRAPPER(_gyro_id);
    INS_PARAM_WRAPPER(_accel_scale);
    INS_PARAM_WRAPPER(_accel_offset);
    INS_PARAM_WRAPPER(_gyro_offset);
    INS_PARAM_WRAPPER(_accel_pos);

    // accelerometer max absolute offsets to be used for calibration
    float _accel_max_abs_offsets[INS_MAX_INSTANCES];

    // accelerometer and gyro raw sample rate in units of Hz
    float  _accel_raw_sample_rates[INS_MAX_INSTANCES];
    float  _gyro_raw_sample_rates[INS_MAX_INSTANCES];

    // how many sensors samples per notify to the backend
    uint8_t _accel_over_sampling[INS_MAX_INSTANCES];
    uint8_t _gyro_over_sampling[INS_MAX_INSTANCES];

    // last sample time in microseconds. Use for deltaT calculations
    // on non-FIFO sensors
    uint64_t _accel_last_sample_us[INS_MAX_INSTANCES];
    uint64_t _gyro_last_sample_us[INS_MAX_INSTANCES];

    // sample times for checking real sensor rate for FIFO sensors
    uint16_t _sample_accel_count[INS_MAX_INSTANCES];
    uint32_t _sample_accel_start_us[INS_MAX_INSTANCES];
    uint16_t _sample_gyro_count[INS_MAX_INSTANCES];
    uint32_t _sample_gyro_start_us[INS_MAX_INSTANCES];
    
    // temperatures for an instance if available
    float _temperature[INS_MAX_INSTANCES];

    // filtering frequency (0 means default)
    AP_Int16    _accel_filter_cutoff;
    AP_Int16    _gyro_filter_cutoff;
    AP_Int8     _gyro_cal_timing;

    // use for attitude, velocity, position estimates
    AP_Int8     _use_old_param[INS_MAX_INSTANCES - INS_AUX_INSTANCES];
    INS_PARAM_WRAPPER(_use);

    // control enable of fast sampling
    AP_Int8     _fast_sampling_mask;

    // control enable of fast sampling
    AP_Int8     _fast_sampling_rate;

    // control enable of detected sensors
    AP_Int8     _enable_mask;
    
    // board orientation from AHRS
    enum Rotation _board_orientation;

    // per-sensor orientation to allow for board type defaults at runtime
    enum Rotation _gyro_orientation[INS_MAX_INSTANCES];
    enum Rotation _accel_orientation[INS_MAX_INSTANCES];

    // calibrated_ok/id_ok flags
    bool _gyro_cal_ok[INS_MAX_INSTANCES];
    bool _accel_id_ok[INS_MAX_INSTANCES];

    // first usable gyro and accel
    uint8_t _first_usable_gyro;
    uint8_t _first_usable_accel;

    // primary accel and gyro
    uint8_t _primary_gyro;
    uint8_t _primary_accel;

    // mask of accels and gyros which we will be actively using
    // and this should wait for in wait_for_sample()
    uint8_t _gyro_wait_mask;
    uint8_t _accel_wait_mask;

    // bitmask bit which indicates if we should log raw accel and gyro data
    uint32_t _log_raw_bit;

    // has wait_for_sample() found a sample?
    bool _have_sample:1;

    bool _backends_detected:1;

    // are gyros or accels currently being calibrated
    bool _calibrating_accel;
    bool _calibrating_gyro;
    bool _trimming_accel;

    // the delta time in seconds for the last sample
    float _delta_time;

    // last time a wait_for_sample() returned a sample
    uint32_t _last_sample_usec;

    // target time for next wait_for_sample() return
    uint32_t _next_sample_usec;

    // time between samples in microseconds
    uint32_t _sample_period_usec;

    // last time update() completed
    uint32_t _last_update_usec;

    // health of gyros and accels
    bool _gyro_healthy[INS_MAX_INSTANCES];
    bool _accel_healthy[INS_MAX_INSTANCES];

    uint32_t _accel_error_count[INS_MAX_INSTANCES];
    uint32_t _gyro_error_count[INS_MAX_INSTANCES];

    // vibration and clipping
    uint32_t _accel_clip_count[INS_MAX_INSTANCES];
    LowPassFilterVector3f _accel_vibe_floor_filter[INS_VIBRATION_CHECK_INSTANCES];
    LowPassFilterVector3f _accel_vibe_filter[INS_VIBRATION_CHECK_INSTANCES];

    // peak hold detector state for primary accel
    struct PeakHoldState {
        float accel_peak_hold_neg_x;
        uint32_t accel_peak_hold_neg_x_age;
    } _peak_hold_state;

    // threshold for detecting stillness
    AP_Float _still_threshold;

    // Trim options
    AP_Int8 _acc_body_aligned;
    AP_Int8 _trim_option;

    static AP_InertialSensor *_singleton;
    AP_AccelCal* _acal;

    AccelCalibrator *_accel_calibrator;

    //save accelerometer bias and scale factors
    void _acal_save_calibrations() override;
    void _acal_event_failure() override;

    // Returns AccelCalibrator objects pointer for specified acceleromter
    AccelCalibrator* _acal_get_calibrator(uint8_t i) override { return i<get_accel_count()?&(_accel_calibrator[i]):nullptr; }

    Vector3f _trim_rad;
    bool _new_trim;

    bool _accel_cal_requires_reboot;

    // sensor error count at startup (used to ignore errors within 2 seconds of startup)
    uint32_t _accel_startup_error_count[INS_MAX_INSTANCES];
    uint32_t _gyro_startup_error_count[INS_MAX_INSTANCES];
    bool _startup_error_counts_set;
    uint32_t _startup_ms;

#if AP_INERTIALSENSOR_KILL_IMU_ENABLED
    uint8_t imu_kill_mask;
#endif

#if HAL_INS_TEMPERATURE_CAL_ENABLE
public:
    // instance number for logging
#if INS_AUX_INSTANCES
    uint8_t tcal_instance(const AP_InertialSensor_TCal &tc) const {
        for (uint8_t i=0; i<INS_MAX_INSTANCES - INS_AUX_INSTANCES; i++) {
            if (&tc == &tcal_old_param[i]) {
                return i;
            }
        }
        for (uint8_t i=0; i<INS_AUX_INSTANCES; i++) {
            if (&tc == &params[i].tcal) {
                return i + INS_MAX_INSTANCES;
            }
        }
        return 0;
    }
#else
    uint8_t tcal_instance(const AP_InertialSensor_TCal &tc) const {
        return &tc - &tcal(0);
    }
#endif
private:
    AP_InertialSensor_TCal tcal_old_param[INS_MAX_INSTANCES - INS_AUX_INSTANCES];

    enum class TCalOptions : uint8_t {
        PERSIST_TEMP_CAL = (1U<<0),
        PERSIST_ACCEL_CAL = (1U<<1),
    };

    // temperature that last calibration was run at
    AP_Float caltemp_accel_old_param[INS_MAX_INSTANCES - INS_AUX_INSTANCES];
    AP_Float caltemp_gyro_old_param[INS_MAX_INSTANCES - INS_AUX_INSTANCES];

    INS_PARAM_WRAPPER(caltemp_accel);
    INS_PARAM_WRAPPER(caltemp_gyro);
    INS_PARAM_WRAPPER(tcal);

    AP_Int32 tcal_options;
    bool tcal_learning;
#endif

    // Raw logging options bitmask and parameter
    enum class RAW_LOGGING_OPTION {
        PRIMARY_GYRO_ONLY   = (1U<<0),
        ALL_GYROS           = (1U<<1),
        POST_FILTER         = (1U<<2),
        PRE_AND_POST_FILTER = (1U<<3),
    };
    AP_Int16 raw_logging_options;
    bool raw_logging_option_set(RAW_LOGGING_OPTION option) const {
        return (raw_logging_options.get() & int32_t(option)) != 0;
    }
};

namespace AP {
    AP_InertialSensor &ins();
};
