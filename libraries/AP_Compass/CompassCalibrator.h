#include <AP_Math.h>

#define COMPASS_CAL_NUM_SPHERE_PARAMS 4
#define COMPASS_CAL_NUM_ELLIPSOID_PARAMS 6
#define COMPASS_CAL_NUM_SAMPLES 300

//RMS tolerance
#define COMPASS_CAL_DEFAULT_TOLERANCE 5.0f

enum compass_cal_status_t {
    COMPASS_CAL_NOT_STARTED=0,
    COMPASS_CAL_WAITING_TO_START=1,
    COMPASS_CAL_SAMPLING_STEP_ONE=2,
    COMPASS_CAL_SAMPLING_STEP_TWO=3,
    COMPASS_CAL_SUCCESS=4,
    COMPASS_CAL_FAILED=5
};

class CompassCalibrator {
public:
    CompassCalibrator();

    void start(bool retry=false, bool autosave=false, float delay=0.0f);
    void clear();

    void new_sample(const Vector3f &sample);
    void run_fit_chunk();

    void check_for_timeout();

    void set_tolerance(float tolerance) { _tolerance = tolerance; }

    void get_calibration(Vector3f &offsets, Vector3f &diagonals, Vector3f &offdiagonals);

    bool running() const;
    float get_completion_percent() const;
    enum compass_cal_status_t get_status() const { return _status; }
    float get_fitness() const { return sqrtf(_fitness); }
    bool get_autosave() const { return _autosave; }
    uint8_t get_attempt() const { return _attempt; }

private:
    union sphere_param_t {
        sphere_param_t(){};
        struct {
            float radius;
            Vector3f offset;
        } named;

        float array[COMPASS_CAL_NUM_SPHERE_PARAMS];
    };

    union ellipsoid_param_t {
        ellipsoid_param_t(){};
        struct {
            Vector3f diag;
            Vector3f offdiag;
        } named;

        float array[COMPASS_CAL_NUM_ELLIPSOID_PARAMS];
    };

    class CompassSample {
    public:
        Vector3f get() const;
        void set(const Vector3f &in);
    private:
        int16_t x;
        int16_t y;
        int16_t z;
    };

    bool fit_acceptable();

    bool _autosave;

    bool _retry;
    uint8_t _attempt;
    uint32_t _start_time_ms;
    float _delay_start_sec;

    float calc_residual(const Vector3f& sample, const sphere_param_t& sp, const ellipsoid_param_t& ep) const;
    float calc_mean_squared_residuals(const sphere_param_t& sp, const ellipsoid_param_t& ep) const;
    float calc_mean_squared_residuals() const;

    void calc_sphere_jacob(const Vector3f& sample, const sphere_param_t& sp, sphere_param_t& ret) const;
    void run_sphere_fit(uint8_t max_iterations=20);

    void calc_ellipsoid_jacob(const Vector3f& sample, const ellipsoid_param_t& sp, ellipsoid_param_t& ret) const;
    void run_ellipsoid_fit(uint8_t max_iterations=20);

    // returns true if sample should be added to buffer
    bool accept_sample(const Vector3f &sample);
    bool accept_sample(const CompassSample &sample);

    void thin_samples();

    bool set_status(compass_cal_status_t status);
    void reset_state();

    uint32_t _last_sample_ms;

    uint16_t _fit_step;

    enum compass_cal_status_t _status;

    CompassSample *_sample_buffer;
    uint16_t _samples_collected;
    uint16_t _samples_thinned;

    // mean squared residuals
    float _fitness;

    float _tolerance;

    sphere_param_t _sphere_param;
    ellipsoid_param_t _ellipsoid_param;

    // math helpers
    bool inverse6x6(const float m[],float invOut[]);
    float det6x6(const float m[]);
    bool inverse4x4(float m[],float invOut[]);
    bool inverse3x3(float m[], float invOut[]);
    uint16_t get_random();
};
