/*
  CINS state estimator for AP_AHRS
 */

#pragma once

#include <AP_AHRS/AP_AHRS_config.h>

#if AP_AHRS_CINS_ENABLED

#include "AP_AHRS_Backend.h"

#include <AP_Math/AP_Math.h>
#include <AP_Math/LieGroups.h>
#include <AP_Common/Location.h>

#define CINS_MAX_HISTORY 100

class AP_AHRS_CINS : public AP_AHRS_Backend
{
public:
    AP_AHRS_CINS();
    const char *shortname() const override { return "CINS"; }

    void update() override;

    void get_results(Estimates &results) override;

    bool synthetic_airspeed_EAS(const Estimates &results, float &airspeed_ret, AirspeedEstimateType &type) override;

    // true once CINS has initialised its position from GPS and its
    // attitude from the accelerometer and compass, so that it is
    // propagating its state from the IMU:
    bool initialised(void) const {
        return done_pos_init && done_attitude_init;
    }

    // true while CINS is initialised and propagating from the IMU:
    bool healthy(void) const;

    // true while CINS is healthy and has recently fused a GPS position:
    bool fusing_gps(void) const;

    bool pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const override;

    // forget the gyro bias, used on new gyro calibration
    void reset_gyro_drift(void) override;

    // forget the IMU biases and re-initialise the attitude, used on
    // new IMU calibration
    void reset() override;

    // return a wind estimation vector, in m/s
    bool wind_estimate(Vector3f &wind) const { return false; }

    bool use_compass() override { return true; }

    void send_ekf_status_report(class GCS_MAVLINK &link) const {
    }

    void get_control_limits(float &ekfGndSpdLimit, float &ekfNavVelGainScaler, float &ekfNavVelGainScaler_Z) const {
        // lower gains in VTOL controllers when flying on CINS; these
        // are the values DCM uses:
        ekfGndSpdLimit = 50.0;
        ekfNavVelGainScaler = 0.5;
        ekfNavVelGainScaler_Z = 0.25;
    }

private:

    struct {
        AP_Float gpspos_att;
        AP_Float gpsvel_att;
        AP_Float gps_lag;
        AP_Float gps_pos;
        AP_Float gps_vel;
        AP_Float mag_att;
        AP_Float Q11;
        AP_Float Q22;
        AP_Float gps_pos_gyr_bias;
        AP_Float gps_vel_gyr_bias;
        AP_Float mag_gyr_bias;
        AP_Float sat_gyr_bias;
        AP_Float gps_pos_acc_bias;
        AP_Float gps_vel_acc_bias;
        AP_Float mag_acc_bias;
        AP_Float sat_acc_bias;
        AP_Float variance_lowpass;
    } gains;

    bool get_location(Location &loc) const
    {
        loc = state.origin;
        loc.offset(state.XHat.pos().x, state.XHat.pos().y);
        loc.alt -= state.XHat.pos().z * 100;
        return state.have_origin;
    }
    bool set_origin(const Location &loc) override;
    bool get_origin(Location &loc) const override {
        loc = state.origin;
        return state.have_origin;
    }

    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const;

public:
    // parameter block, registered as a subgroup of AP_AHRS (AHRS_CINS_*)
    static const struct AP_Param::GroupInfo var_info[];

private:
    void update_imu(const Vector3F &gyro_rads, const Vector3F &accel_mss, const ftype dt);
    void update_gps(const Vector3F &pos, const Vector3F &vel, const ftype gps_dt);
    ftype update_vector_measurement_cts(const Vector3F &measurement, const Vector3F& reference, const Vector2F &ref_base, const ftype& gain_R, const ftype& gain_V, const ftype& gain_gyr_bias, const ftype& gain_acc_bias, const ftype dt, ftype &variance, bool &accepted);
    void update_attitude_from_compass();
    bool init_attitude(const Vector3F &accel);
    void reset_position_to_gps(const Vector3F &pos, const Vector3F &vel);
    void update_baro_height();
    // one-time estimator state initialisation, run lazily on the first
    // update() (the backend interface has no init() entry point):
    void init(void);
    bool get_compass_yaw(ftype roll_rad, ftype pitch_rad, ftype &yaw_rad);
    bool get_compass_vector(Vector3F &mag_vec, Vector3F &mag_ref, ftype &dt);

#if HAL_LOGGING_ENABLED
    void Log_Write();
    uint32_t last_log_ms;
#endif

    void compute_bias_update_imu(const SIM23& Gamma);
    void saturate_bias(Vector3F& bias_correction, const Vector3F& current_bias, const ftype& saturation, const ftype& dt) const;

    struct {
        Location origin;
        bool have_origin;

        //XHat and ZHat for CINS
        Gal3F XHat; // State estimate containing attitude, velocity, position
        SIM23 ZHat; // Auxiliary state for estimator maths

        // Gyro Bias
        Vector3F gyr_bias;
        Vector3F gyr_bias_correction;
        struct {
            Matrix3F rot;
            Matrix3F vel;
            Matrix3F pos;
        } gyr_bias_gain_mat;

        // Accel Bias
        Vector3F acc_bias;
        Vector3F acc_bias_correction;
        struct {
            Matrix3F rot;
            Matrix3F vel;
            Matrix3F pos;
        } acc_bias_gain_mat;

        // Variance States
        struct {
            uint8_t steps_to_initialise;
            ftype velVar;
            ftype posVar;
            ftype magVar;
            ftype velTest;
            ftype posTest;
            ftype magTest;
        } variances;

    } state;

    // Variables to be kept on the heap
    struct {
        Matrix3F A11;
        Matrix3F A12;
        Matrix3F A13;
        Matrix3F A21;
        Matrix3F A22;
        Matrix3F A23;
        Matrix3F A31;
        Matrix3F A32;
        Matrix3F A33;
        Matrix3F M1Gyr;
        Matrix3F M2Gyr;
        Matrix3F M3Gyr;
        Matrix3F M1Acc;
        Matrix3F M2Acc;
        Matrix3F M3Acc;
        Matrix3F C11;
        Matrix3F C12;
        Matrix3F C13;
        Matrix3F K11;
        Matrix3F K21;
        Matrix3F K31;

        SIM23 Delta_SIM23;
        SIM23 ZInv;
        Matrix3F I3;
        Vector3F zero_vector;
        Vector3F gravity_vector;

        GL2 S_Gamma;
        Vector3F W_Gamma_1;
        Vector3F W_Gamma_2;
        SIM23 GammaInv;

        Vector3F Omega_Delta;
        Vector3F W_Delta1;
        Vector3F W_Delta2;
    } heapVars;

    // Delay variables required to handle delayed measurements.

    struct stamped_Gal3F {
        double timestamp;
        Gal3F gal3;
    };

    struct {
        double internal_time;
        ObjectBuffer<stamped_Gal3F> stamped_inputs{CINS_MAX_HISTORY};
        Gal3F pending_input; // inputs not yet pushed into stamped_inputs
        double last_push_time; // internal_time of the last push
        Vector3F gps_vel;
        Vector3F gps_pos;
        Gal3F YR; // Primary right delay matrix
        Gal3F YR_stepper; // right delay matrix to apply at new GPS measurement.
    } delayer;

    uint32_t last_gps_update_ms;
    uint32_t last_gps_fused_ms;   // last time a GPS position passed the innovation gate
    uint32_t last_imu_update_ms;
    bool done_attitude_init;
    bool done_pos_init;
    bool done_init;
    uint32_t last_mag_us;

    // CINS's height comes from GPS.  While it is not fusing GPS it
    // publishes the baro height instead, offset to match its own
    // height when it stopped fusing GPS:
    ftype baro_offset_D;      // position_D = baro_offset_D - baro altitude
    bool have_baro_offset;
    bool using_baro_height;
    uint16_t position_D_reset_count;
};

#endif // AP_AHRS_CINS_ENABLED

