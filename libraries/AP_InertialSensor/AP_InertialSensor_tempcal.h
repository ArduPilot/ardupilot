#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/polyfit.h>
#include <Filter/LowPassFilter2p.h>

// AP_InertialSensor_TCal class is public for use by SITL
class AP_InertialSensor_TCal {
public:
    static const struct AP_Param::GroupInfo var_info[];
    void correct_accel(float temperature, float cal_temp, Vector3f &accel) const;
    void correct_gyro(float temperature, float cal_temp, Vector3f &accel) const;
    void sitl_apply_accel(float temperature, Vector3f &accel) const;
    void sitl_apply_gyro(float temperature, Vector3f &accel) const;

    void update_accel_learning(const Vector3f &accel);
    void update_gyro_learning(const Vector3f &accel);

    enum class Enable : uint8_t {
        Disabled = 0,
        Enabled = 1,
        LearnCalibration = 2,
    };

    // add samples for learning
    void update_accel_learning(const Vector3f &gyro, float temperature);
    void update_gyro_learning(const Vector3f &accel, float temperature);
    
    // class for online learning of calibration
    class Learn {
    public:
        Learn(AP_InertialSensor_TCal &_tcal, float _start_temp);

        // state for accel/gyro (accel first)
        struct LearnState {
            float last_temp;
            uint32_t last_sample_ms;
            Vector3f sum;
            uint32_t sum_count;
            LowPassFilter2p<float> temp_filter;
            // double precision is needed for good results when we
            // span a wide range of temperatures
            PolyFit<4, double, Vector3f> pfit;
        } state[2];

        void add_sample(const Vector3f &sample, float temperature, LearnState &state);
        void finish_calibration(float temperature);
        bool save_calibration(float temperature);
        void reset(float temperature);
        float start_temp;
        float start_tmax;
        uint32_t last_save_ms;

        AP_InertialSensor_TCal &tcal;
        uint8_t instance(void) const {
            return tcal.instance();
        }
        Vector3f accel_start;
    };

    AP_Enum<Enable> enable;

    // get persistent params for this instance
    void get_persistent_params(ExpandingString &str) const;

private:
    AP_Float temp_max;
    AP_Float temp_min;
    AP_Vector3f accel_coeff[3];
    AP_Vector3f gyro_coeff[3];
    Vector3f accel_tref;
    Vector3f gyro_tref;
    Learn *learn;

    void correct_sensor(float temperature, float cal_temp, const AP_Vector3f coeff[3], Vector3f &v) const;
    Vector3f polynomial_eval(float temperature, const AP_Vector3f coeff[3]) const;

    // get instance number
    uint8_t instance(void) const;
};
