/*
  CINS state estimator for AP_AHRS, developed by Mr Patrick Wiltshire and Dr Pieter Van Goor
 */

#include "AP_AHRS_config.h"

#if AP_AHRS_CINS_ENABLED

#include "AP_AHRS_CINS.h"
#include "AP_AHRS.h"

#include <AP_Compass/AP_Compass.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_HAL/Util.h>

#if !AP_COMPASS_ENABLED || !AP_GPS_ENABLED
#error "CINS requires compass and GPS support"
#endif

// gains tested for 5Hz GPS
#define CINS_GAIN_GPSPOS_ATT (1.0E-5)
#define CINS_GAIN_GPSVEL_ATT (1.0E-4)

// Gains for new CINS method
#define CINS_GAIN_GPSPOS_POS (10.0)
#define CINS_GAIN_GPSVEL_VEL (10.0)
#define CINS_GAIN_MAG (0.5)
#define CINS_GAIN_Q11 (0.001)
#define CINS_GAIN_Q22 (0.001)
// Gains for the Gyro and Accel Biases
#define CINS_GAIN_POS_GYR_BIAS (1E-7)
#define CINS_GAIN_VEL_GYR_BIAS (1E-6)
#define CINS_GAIN_MAG_GYR_BIAS (0.005)
#define CINS_SAT_GYR_BIAS (0.02) // 0.0873 rad/s is 5 deg/s
#define CINS_GAIN_POS_ACC_BIAS (1E-3)
#define CINS_GAIN_VEL_ACC_BIAS (1E-3)
#define CINS_GAIN_MAG_ACC_BIAS (0.005)
#define CINS_SAT_ACC_BIAS (0.5)
// Naive GPS Time Delay settings
#define CINS_GPS_DELAY (0.2) // seconds of delay
// Variance gain for health checking
#define CINS_VAR_LOWPASS (0.01) // low pass of measurement errors
#define CINS_VAR_SCALE (2.) // accept measurements with normalised variance smaller than this
// reject a GPS measurement whose squared error exceeds this multiple of
// the low-passed running variance (a ~5-sigma innovation gate); this
// stops jammed/grossly-wrong GPS from diverging the observer
#define CINS_GPS_INNOV_GATE (25.0)
// CINS is unhealthy once it has not propagated from the IMU, or has
// had no GPS data, for this long:
#define CINS_HEALTH_IMU_TIMEOUT_MS 500U
#define CINS_HEALTH_GPS_TIMEOUT_MS 5000U

/*
  CINS is very fast, but stack hungry, using Os keeps stack usage down
 */
#pragma GCC optimize("Os")

// table of user settable parameters
const AP_Param::GroupInfo AP_AHRS_CINS::var_info[] = {

    // @Param: GPATT
    // @DisplayName: CINS GPS position attitude gain
    // @Description: CINS GPS position attitude gain
    // @Range: 0.0001 0.001
    // @User: Advanced
    AP_GROUPINFO("GPATT", 1, AP_AHRS_CINS, gains.gpspos_att, CINS_GAIN_GPSPOS_ATT),

    // @Param: GVATT
    // @DisplayName: CINS GPS velocity attitude gain
    // @Description: CINS GPS velocity attitude gain
    // @Range: 0.0001 0.001
    // @User: Advanced
    AP_GROUPINFO("GVATT", 2, AP_AHRS_CINS, gains.gpsvel_att, CINS_GAIN_GPSVEL_ATT),

    // @Param: GPLAG
    // @DisplayName: CINS GPS lag
    // @Description: CINS GPS lag
    // @Range: 0.0 0.2
    // @User: Advanced
    // @Units: s
    AP_GROUPINFO("GPLAG", 3, AP_AHRS_CINS, gains.gps_lag, CINS_GPS_DELAY),

    // @Param: GPPOS
    // @DisplayName: CINS GPS position gain
    // @Description: CINS GPS position gain
    // @Range: 0.1 10.0
    // @User: Advanced
    AP_GROUPINFO("GPPOS", 4, AP_AHRS_CINS, gains.gps_pos, CINS_GAIN_GPSPOS_POS),

    // @Param: GPVEL
    // @DisplayName: CINS GPS velocity gain
    // @Description: CINS GPS velocity gain
    // @Range: 0.1 10.0
    // @User: Advanced
    AP_GROUPINFO("GPVEL", 5, AP_AHRS_CINS, gains.gps_vel, CINS_GAIN_GPSVEL_VEL),

    // @Param: MGATT
    // @DisplayName: CINS magnetometer attitude gain
    // @Description: CINS magnetometer (compass) attitude gain
    // @Range: 0.1 10.0
    // @User: Advanced
    AP_GROUPINFO("MGATT", 6, AP_AHRS_CINS, gains.mag_att, CINS_GAIN_MAG),

    // @Param: INTQV
    // @DisplayName: CINS internal velocity Q gain
    // @Description: CINS internal velocity Q gain
    // @Range: 0.0001 0.1
    // @User: Advanced
    AP_GROUPINFO("INTQV", 7, AP_AHRS_CINS, gains.Q11, CINS_GAIN_Q11),

    // @Param: INTQP
    // @DisplayName: CINS internal position Q gain
    // @Description: CINS internal position Q gain
    // @Range: 0.0001 0.1
    // @User: Advanced
    AP_GROUPINFO("INTQP", 8, AP_AHRS_CINS, gains.Q22, CINS_GAIN_Q22),

    // @Param: GPPGB
    // @DisplayName: CINS GPS position - gyro bias gain
    // @Description: CINS GPS position - gyro bias gain
    // @Range: 0.0000001 0.000001
    // @User: Advanced
    AP_GROUPINFO("GPPGB", 9, AP_AHRS_CINS, gains.gps_pos_gyr_bias, CINS_GAIN_POS_GYR_BIAS),

    // @Param: GPVGB
    // @DisplayName: CINS GPS velocity - gyro bias gain
    // @Description: CINS GPS velocity - gyro bias gain
    // @Range: 0.0000001 0.000001
    // @User: Advanced
    AP_GROUPINFO("GPVGB", 10, AP_AHRS_CINS, gains.gps_vel_gyr_bias, CINS_GAIN_VEL_GYR_BIAS),

    // @Param: MAGGB
    // @DisplayName: CINS magnetometer - gyro bias gain
    // @Description: CINS magnetometer (compass) gyro bias gain
    // @Range: 0.001 0.0001
    // @User: Advanced
    AP_GROUPINFO("MAGGB", 11, AP_AHRS_CINS, gains.mag_gyr_bias, CINS_GAIN_MAG_GYR_BIAS),

    // @Param: SATGB
    // @DisplayName: CINS gyro bias saturation
    // @Description: CINS gyro bias saturation
    // @Range: 0.05 0.1
    // @User: Advanced
    AP_GROUPINFO("SATGB", 12, AP_AHRS_CINS, gains.sat_gyr_bias, CINS_SAT_GYR_BIAS),


    // @Param: GPPAB
    // @DisplayName: CINS GPS position - accel. bias gain
    // @Description: CINS GPS position - accel. bias gain
    // @Range: 0.0000001 0.000001
    // @User: Advanced
    AP_GROUPINFO("GPPAB", 13, AP_AHRS_CINS, gains.gps_pos_acc_bias, CINS_GAIN_POS_ACC_BIAS),

    // @Param: GPVAB
    // @DisplayName: CINS GPS velocity - accel. bias gain
    // @Description: CINS GPS velocity - accel. bias gain
    // @Range: 0.0000001 0.000001
    // @User: Advanced
    AP_GROUPINFO("GPVAB", 14, AP_AHRS_CINS, gains.gps_vel_acc_bias, CINS_GAIN_VEL_ACC_BIAS),

    // @Param: MAGAB
    // @DisplayName: CINS magnetometer - accel. bias gain
    // @Description: CINS magnetometer (compass) accel. bias gain
    // @Range: 0.001 0.0001
    // @User: Advanced
    AP_GROUPINFO("MAGAB", 15, AP_AHRS_CINS, gains.mag_acc_bias, CINS_GAIN_MAG_ACC_BIAS),

    // @Param: SATAB
    // @DisplayName: CINS accel. bias saturation
    // @Description: CINS accel. bias saturation
    // @Range: 0.05 0.1
    // @User: Advanced
    AP_GROUPINFO("SATAB", 16, AP_AHRS_CINS, gains.sat_acc_bias, CINS_SAT_ACC_BIAS),

    // @Param: VARLP
    // @DisplayName: CINS variance low-pass gain
    // @Description: CINS variance low-pass gain used for health checks
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("VARLP", 17, AP_AHRS_CINS, gains.variance_lowpass, CINS_VAR_LOWPASS),

    // @Param: VARSC
    // @DisplayName: CINS variance scale
    // @Description: CINS variance scale used for health checks
    // @Range: 1.0 5.0
    // @User: Advanced
    AP_GROUPINFO("VARSC", 18, AP_AHRS_CINS, gains.variance_scale, CINS_VAR_SCALE),

    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

Vector3F computeRotationCorrection(const Vector3F& v1, const Vector3F& v2, const ftype& gain, const ftype& dt);

// constructor
AP_AHRS_CINS::AP_AHRS_CINS(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}


/*
  initialise the filter
 */
void AP_AHRS_CINS::init(void)
{
    //Initialise XHat and ZHat as stationary at the origin
    state.XHat = Gal3F::identity();
    state.ZHat = SIM23::identity();

    state.gyr_bias.zero();
    state.gyr_bias_gain_mat.rot.zero();
    state.gyr_bias_gain_mat.vel.zero();
    state.gyr_bias_gain_mat.pos.zero();

    state.acc_bias.zero();
    state.acc_bias_gain_mat.rot.zero();
    state.acc_bias_gain_mat.vel.zero();
    state.acc_bias_gain_mat.pos.zero();

    delayer.YR = Gal3F::identity();
    delayer.YR_stepper = Gal3F::identity();

    heapVars.I3.identity();
    heapVars.zero_vector.zero();
    heapVars.gravity_vector = Vector3F(0,0,GRAVITY_MSS);

    state.variances.velVar = 10.0;
    state.variances.posVar = 10.0;
    state.variances.magVar = 1.0;
    state.variances.steps_to_initialise = 100;
}
/*
  update function, called at loop rate
 */
void AP_AHRS_CINS::update(void)
{
    if (!done_init) {
        init();
        done_init = true;
    }

    if (!done_yaw_init) {
        done_yaw_init = init_yaw();
    }

    // read the sensor libraries directly, as DCM does.  CINS has no
    // Replay support, and AP_DAL's sensor data is only refreshed when
    // EKF2 or EKF3 start a frame, so it would go stale with both
    // disabled:
    const auto &ins = AP::ins();

    // Get delta angle and convert to gyro rad/s
    const uint8_t gyro_index = ins.get_first_usable_gyro();
    Vector3f delta_angle;
    float dangle_dt;
    if (!ins.get_delta_angle(gyro_index, delta_angle, dangle_dt) || dangle_dt <= 0) {
        // can't update, no delta angle
        return;
    }
    // turn delta angle into a gyro in radians/sec
    const Vector3F gyro = (delta_angle / dangle_dt).toftype();

    // get delta velocity and convert to accel m/s/s
    const uint8_t accel_index = ins.get_first_usable_accel();
    Vector3f delta_velocity;
    float dvel_dt;
    if (!ins.get_delta_velocity(accel_index, delta_velocity, dvel_dt) || dvel_dt <= 0) {
        // can't update, no delta velocity
        return;
    }
    // turn delta velocity into a accel vector
    const Vector3F accel = (delta_velocity / dvel_dt).toftype();

    if (done_yaw_init && state.have_origin) {
        update_imu(gyro, accel, dangle_dt);
        last_imu_update_ms = AP_HAL::millis();
    }

    // see if we have new GPS data
    const auto &gps = AP::gps();
    if (gps.status() >= AP_GPS_FixType::FIX_3D) {
        const uint32_t last_gps_fix_ms = gps.last_message_time_ms(gps.primary_sensor());
        if (last_gps_update_ms != last_gps_fix_ms) {
            // don't allow for large gain if we lose and regain GPS
            const float gps_dt = MIN((last_gps_fix_ms - last_gps_update_ms)*0.001, 1);
            last_gps_update_ms = last_gps_fix_ms;
            const auto &loc = gps.location();
            if (!state.have_origin) {
                state.have_origin = true;
                state.origin = loc;
            }
            const auto & vel = gps.velocity();
            const Vector3d pos = state.origin.get_distance_NED_double(loc);

            update_gps(pos.toftype(), vel.toftype(), gps_dt);
        }
    }

    update_attitude_from_compass();

#if HAL_LOGGING_ENABLED
    Log_Write();
#endif
}

#if HAL_LOGGING_ENABLED
/*
  write the CINS log messages.  Like DCM, these are written whether
  or not CINS is the selected backend, at 10Hz
 */
void AP_AHRS_CINS::Log_Write()
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_log_ms < 100) {
        return;
    }
    last_log_ms = now_ms;

    // @LoggerMessage: CINS
    // @Description: CINS state
    // @Field: TimeUS: Time since system startup
    // @Field: I: instance
    // @Field: Roll: euler roll
    // @Field: Pitch: euler pitch
    // @Field: Yaw: euler yaw
    // @Field: VN: velocity north
    // @Field: VE: velocity east
    // @Field: VD: velocity down
    // @Field: PN: position north
    // @Field: PE: position east
    // @Field: PD: position down
    // @Field: Lat: latitude
    // @Field: Lon: longitude
    // @Field: Alt: altitude AMSL
    ftype roll_rad, pitch_rad, yaw_rad;
    state.XHat.rot().to_euler(&roll_rad, &pitch_rad, &yaw_rad);
    Location loc;
    UNUSED_RESULT(get_location(loc));

    AP::logger().WriteStreaming("CINS", "TimeUS,I,Roll,Pitch,Yaw,VN,VE,VD,PN,PE,PD,Lat,Lon,Alt",
                                "s#dddnnnmmmDUm",
                                "F-000000000GG0",
                                "QBfffffffffLLf",
                                AP_HAL::micros64(),
                                0,
                                degrees(roll_rad),
                                degrees(pitch_rad),
                                wrap_360(degrees(yaw_rad)),
                                state.XHat.vel().x,
                                state.XHat.vel().y,
                                state.XHat.vel().z,
                                state.XHat.pos().x,
                                state.XHat.pos().y,
                                state.XHat.pos().z,
                                loc.lat,
                                loc.lng,
                                loc.alt*0.01);

    // @LoggerMessage: CINW
    // @Description: CINS wind estimate
    // @Field: TimeUS: Time since system startup
    // @Field: I: instance
    // @Field: VWN: wind estimate north
    // @Field: VWE: wind estimate east
    // @Field: VWD: wind estimate down
    AP::logger().WriteStreaming("CINW", "TimeUS,I,VWN,VWE,VWD",
                                "s#nnn",
                                "F-000",
                                "QBfff",
                                AP_HAL::micros64(),
                                0,
                                _wind.x,
                                _wind.y,
                                _wind.z);

    // @LoggerMessage: CIN2
    // @Description: Extra CINS states
    // @Field: TimeUS: Time since system startup
    // @Field: I: instance
    // @Field: GX: Gyro Bias X
    // @Field: GY: Gyro Bias Y
    // @Field: GZ: Gyro Bias Z
    // @Field: AX: Accel Bias X
    // @Field: AY: Accel Bias Y
    // @Field: AZ: Accel Bias Z
    // @Field: AVN: auxiliary velocity north
    // @Field: AVE: auxiliary velocity east
    // @Field: AVD: auxiliary velocity down
    // @Field: APN: auxiliary position north
    // @Field: APE: auxiliary position east
    // @Field: APD: auxiliary position down
    AP::logger().WriteStreaming("CIN2", "TimeUS,I,GX,GY,GZ,AX,AY,AZ,AVN,AVE,AVD,APN,APE,APD",
                                "s#kkkooonnnmmm",
                                "F-000000000000",
                                "QBffffffffffff",
                                AP_HAL::micros64(),
                                0,
                                degrees(state.gyr_bias.x),
                                degrees(state.gyr_bias.y),
                                degrees(state.gyr_bias.z),
                                state.acc_bias.x,
                                state.acc_bias.y,
                                state.acc_bias.z,
                                state.ZHat.W1().x,
                                state.ZHat.W1().y,
                                state.ZHat.W1().z,
                                state.ZHat.W2().x,
                                state.ZHat.W2().y,
                                state.ZHat.W2().z);
}
#endif  // HAL_LOGGING_ENABLED

void AP_AHRS_CINS::get_results(AP_AHRS_Backend::Estimates &results)
{
    // not using a specific sensor:
    results.primary_gyro = AP::ins().get_first_usable_gyro();
    results.primary_accel = AP::ins().get_first_usable_accel();

#if AP_AIRSPEED_ENABLED
    // CINS does not fuse airspeed; use the primary sensor:
    results.active_airspeed_index = primary_airspeed_index();
#endif  // AP_AIRSPEED_ENABLED

    //
    // attitude
    //
    // publish the vehicle-body (trimmed) attitude so AHRS_TRIM / a board
    // mounting offset is honoured, matching DCM's _body_dcm_matrix.  This
    // also feeds the trimmed fuselage axis into estimate_wind() below, so the
    // wind estimate is invariant to the board mounting offset.
    results.dcm_matrix = (state.XHat.rot() * AP::ahrs().get_rotation_vehicle_body_to_autopilot_body().toftype()).tofloat();
    results.quaternion.from_rotation_matrix(results.dcm_matrix);
    results.dcm_matrix.to_euler(&results.roll_rad, &results.pitch_rad, &results.yaw_rad);
    results.attitude_valid = healthy();

    //
    // angular rates and accels
    //
    // CINS's biases follow the measurement - truth convention, so the
    // drift added to the gyro is the negated bias:
    const auto &ins = AP::ins();
    results.gyro_drift = -state.gyr_bias.tofloat();
    results.gyro_estimate = ins.get_gyro(results.primary_gyro) + results.gyro_drift;
    results.accel_bias = state.acc_bias.tofloat();
    // XHat's rotation takes the autopilot body frame to NED:
    results.accel_ef = (state.XHat.rot() * (ins.get_accel(results.primary_accel) - results.accel_bias).toftype()).tofloat();

    //
    // velocities
    //
    results.velocity_NED = state.XHat.vel().tofloat();
    results.velocity_NED_valid = healthy();

    // ground velocity estimate in meters/second, in North/East order
    // note: velocity_NE is significantly different to results.velocity_NED.xy()!
    results.velocity_NE = results.velocity_NED.xy();

    results.vert_pos_rate_D = results.velocity_NED.z;
    results.vert_pos_rate_D_valid = healthy();

    results.location_valid = get_location(results.location) && healthy();

    // origin-relative position: XHat's position is NED from our origin
    // (see get_origin()):
    if (healthy()) {
        const Vector3F &pos = state.XHat.pos();
        results.position_NE = Vector2p{postype_t(pos.x), postype_t(pos.y)};
        results.position_NE_valid = true;
        results.position_D = pos.z;
        results.position_D_valid = true;
    }

    /*
     * Sensor-related information
     */
    // true if the estimator will use GPS data in creating its
    // estimate when the data is good:
    results.configured_to_use_gps = true;
    // true if GPS is configured as the horizontal position source
    // for this estimator.  Used to decide whether GPS will set
    // the navigation origin.
    results.configured_to_use_gps_for_pos_XY = true;

    // overall backend health/initialisation, consumed by the frontend
    // now that AP_AHRS_Backend no longer exposes a healthy() virtual:
    results.healthy = healthy();
    results.initialised = initialised();

    // publish a nav filter status so the frontend selects CINS (the
    // fixed-wing/ground fallback logic demotes a backend to DCM when
    // it reports no valid nav outputs).  Once we have converged (have
    // an origin) we provide the standard attitude/velocity/position
    // outputs from our own GPS-aided solution:
    memset(&results.filter_status, 0, sizeof(results.filter_status));
    if (healthy()) {
        results.filter_status.flags.attitude = true;
        results.filter_status.flags.horiz_vel = true;
        results.filter_status.flags.vert_vel = true;
        results.filter_status.flags.horiz_pos_rel = true;
        results.filter_status.flags.horiz_pos_abs = true;
        results.filter_status.flags.vert_pos = true;
        results.filter_status.flags.pred_horiz_pos_rel = true;
        results.filter_status.flags.pred_horiz_pos_abs = true;
        results.filter_status.flags.initalized = true;
        results.filter_status.flags.using_gps = true;
    }
    results.filter_status_valid = true;

    get_control_limits(results.control_ground_speed_limit_ms,
                       results.control_gain_scaler_XY,
                       results.control_gain_scaler_Z);

    // drive the wind-triangle estimator from our own velocity and
    // fuselage forward direction, and publish the resulting synthetic
    // wind.  estimate_wind() self-gates on wind estimation being
    // enabled (Plane only) and rate-limits internally;
    // synthetic_airspeed_EAS() derives a synthetic airspeed from this wind
    // and our nav velocity:
    if (results.velocity_NED_valid) {
        estimate_wind(results.velocity_NED, results.dcm_matrix.colx());
    }
    results.wind = _wind;
    results.wind_valid = (AP::ahrs().get_wind_estimation_enabled() &&
                          _have_wind_estimate &&
                          results.healthy);

    // publish this backend's airspeed estimate from the results above:
    fill_airspeed_estimate(results);
}

// derive a synthetic airspeed the way DCM does, from our own nav velocity
// and wind estimate: the forward (body X) component of the air-relative
// velocity, constrained by the ground speed and AHRS_WIND_MAX.  Without a
// wind estimate, ask the frontend to fall back to the DCM synthetic:
bool AP_AHRS_CINS::synthetic_airspeed_EAS(const Estimates &results, float &airspeed_ret, AirspeedEstimateType &type)
{
    if (!results.wind_valid || !results.velocity_NED_valid) {
        type = AirspeedEstimateType::DCM_FALLBACK;
        return false;
    }
    type = AirspeedEstimateType::CINS_SYNTHETIC;
    // take the positive component in the X direction; this mimics a pitot
    // tube:
    const Vector3f airspeed = results.dcm_matrix.mul_transpose(results.velocity_NED - results.wind);
    airspeed_ret = MAX(airspeed.x, 0) * get_TAS2EAS();
    constrain_airspeed_EAS_by_ground_speed(airspeed_ret);
    return true;
}

bool AP_AHRS_CINS::healthy(void) const
{
    if (!initialised()) {
        return false;
    }
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_imu_update_ms > CINS_HEALTH_IMU_TIMEOUT_MS) {
        return false;
    }
    if (now_ms - last_gps_update_ms > CINS_HEALTH_GPS_TIMEOUT_MS) {
        return false;
    }
    return true;
}

bool AP_AHRS_CINS::pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const
{
    if (!state.have_origin) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Waiting for GPS");
        return false;
    }
    if (!done_yaw_init) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Waiting for compass yaw");
        return false;
    }
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Not healthy");
        return false;
    }
    return true;
}

/*
  update on new GPS sample
 */
void AP_AHRS_CINS::update_gps(const Vector3F &pos, const Vector3F &vel, const ftype gps_dt)
{
    // Update the delayer variables. The gps vel and pos are used at every time step through the delay structure.
    delayer.YR = delayer.YR_stepper.inverse() * delayer.YR;
    delayer.YR_stepper = Gal3F::identity();
    delayer.gps_vel = vel;
    delayer.gps_pos = pos;
}


/*
  update from IMU data
 */
void AP_AHRS_CINS::update_imu(const Vector3F &gyro_rads, const Vector3F &accel_mss, const ftype dt)
{
    //Integrate Dynamics using the Matrix exponential

    // Update the bias gain matrices
    // In mathematical terms, \dot{M} = B = - Ad_{Z^{-1} \hat{X}} [I_3, 0_3; 0_3, I_3; 0_3, 0_3].
    // Here, we split B into its parts and add them to the parts of the bias gain matrix.
    if (done_yaw_init) {
        const SIM23 XInv_Z = SIM23(state.XHat.inverse()) * state.ZHat;
        state.gyr_bias_gain_mat.rot += -state.XHat.rot() * dt;
        state.gyr_bias_gain_mat.vel += Matrix3F::skew_symmetric(XInv_Z.W1()) * XInv_Z.R().transposed() * dt;
        state.gyr_bias_gain_mat.pos += Matrix3F::skew_symmetric(XInv_Z.W2()) * XInv_Z.R().transposed() * dt;

        // state.acc_bias_gain_mat.rot is unchanged
        state.acc_bias_gain_mat.vel += -state.XHat.rot() * state.ZHat.A().a11() * dt;
        state.acc_bias_gain_mat.pos += -state.XHat.rot() * state.ZHat.A().a12() * dt;
    }

    const Gal3F leftMat = Gal3F::exponential(heapVars.zero_vector, heapVars.zero_vector, heapVars.gravity_vector*dt, -dt);
    const Gal3F rightMat = Gal3F::exponential((gyro_rads-state.gyr_bias)*dt, heapVars.zero_vector, (accel_mss-state.acc_bias)*dt, dt);
    //Update XHat (Observer Dynamics)
    state.XHat = leftMat * state.XHat * rightMat;

    //Update ZHat (Auxilary Dynamics)
    heapVars.GammaInv = SIM23::identity();
    const GL2 S_Gamma = 0.5 * state.ZHat.A().transposed() * GL2(gains.Q11.get(), 0., 0., gains.Q22.get()) * state.ZHat.A();
    heapVars.GammaInv.A() = GL2::identity() - dt * S_Gamma;

    // Update the bias gain with Gamma
    compute_bias_update_imu(heapVars.GammaInv.inverse());

    state.ZHat = SIM23(leftMat) * state.ZHat * heapVars.GammaInv;

    // Update the delay matrices and buffer
    delayer.YR = delayer.YR * rightMat;
    delayer.internal_time += dt;
    delayer.stamped_inputs.push({delayer.internal_time, rightMat});

    struct stamped_Gal3F back;
    while (delayer.stamped_inputs.peek(back) &&
           back.timestamp < delayer.internal_time - gains.gps_lag) {
        delayer.YR_stepper = delayer.YR_stepper * back.gal3;
        delayer.stamped_inputs.pop();
    }

    // Only fuse the (delayed) GPS while it is fresh: continuing to fuse
    // a frozen GPS position while dead-reckoning drives the observer to
    // diverge (and faults under SITL's floating-point-exception
    // trapping).  The XHat/ZHat propagation and delay buffer above keep
    // dead-reckoning until GPS returns.
    if (last_gps_update_ms == 0 ||
        AP_HAL::millis() - last_gps_update_ms >= 1000U) {
        return;
    }

    // Update using delayed GPS
    const ftype& current_lag = delayer.YR.tau();
    const Vector3F& left_delay_vel = -heapVars.gravity_vector*current_lag;
    const Vector3F& left_delay_pos = -heapVars.gravity_vector*0.5*current_lag*current_lag;

    const Gal3& YRInv = delayer.YR.inverse();
    const Vector2F& ref_vel = Vector2F(1., 0.);
    const Vector3F& mu0_vel = YRInv.vel();
    const Vector3F mu_vel = delayer.gps_vel - left_delay_vel * ref_vel.x - left_delay_pos * ref_vel.y;
    const ftype velErrorSq = update_vector_measurement_cts(mu_vel, mu0_vel, ref_vel, gains.gpsvel_att.get(), gains.gps_vel.get(), gains.gps_vel_gyr_bias.get(), gains.gps_vel_acc_bias.get(), dt, state.variances.velVar);
    state.variances.velTest = velErrorSq / (state.variances.velVar * gains.variance_scale);


    const Vector3F& mu0_pos = YRInv.pos();
    const Vector2F& ref_pos = Vector2F(-current_lag, 1.);
    const Vector3F mu_pos = delayer.gps_pos - left_delay_vel * ref_pos.x - left_delay_pos * ref_pos.y;
    const ftype posErrorSq = update_vector_measurement_cts(mu_pos, mu0_pos, ref_pos, gains.gpspos_att.get(), gains.gps_pos.get(), gains.gps_pos_gyr_bias.get(), gains.gps_pos_acc_bias.get(), dt, state.variances.posVar);
    state.variances.posTest = posErrorSq / (state.variances.posVar * gains.variance_scale);

    // The CINS algorithm does not have an internal variance state like an EKF.
    // Instead, we compute the variance by applying a low-pass to the squared measurement errors.
    // If a new squared measurement error arrives that is much larger than this low-passed value, it indicates something is wrong.
    // This only works after initialising for a number of steps first.
    if (state.variances.steps_to_initialise > 0) {
        --state.variances.steps_to_initialise;
    }

}

ftype AP_AHRS_CINS::update_vector_measurement_cts(const Vector3F &measurement, const Vector3F& reference, const Vector2F &ref_base, const ftype& gain_R, const ftype& gain_V, const ftype& gain_gyr_bias, const ftype& gain_acc_bias, const ftype dt, ftype &variance)
{
    // Compute and apply an update for an arbitrary vector-type measurement.
    // The measurement is of the form $\mu = R \mu_0 + V C$, where
    // \mu is the measured value
    // \mu_0 is a reference vector (e.g. zero for GPS and 'north' for magnetometer)
    // C is a the reference vector 'base' (e.g. zero for magnetometer and (1,0) or (0,1) for GPS vel. or pos.)
    // The return value is the squared measurement error
    // The update equations are then drawn from Lemma 5.3 in
    // van Goor, Pieter, et al. "Constructive synchronous observer design for inertial navigation with delayed GNSS measurements." European Journal of Control (2024): 101047.

    // Set up some computation variables
    const Vector3F& muHat = state.XHat.rot() * reference + state.XHat.vel()*ref_base.x + state.XHat.pos()*ref_base.y;

    // innovation gating: skip the correction (and the running-variance
    // update) for a measurement whose squared error exceeds
    // CINS_GPS_INNOV_GATE times the low-passed running variance, once the
    // variance estimate has settled.  Leaving the observer state and the
    // variance untouched means a jammed or grossly-wrong GPS fix can
    // neither diverge the observer nor inflate the variance (which would
    // re-open the gate); accepted measurements keep the variance tracking
    // so a run of rejections cannot lock the gate closed.
    const ftype measurementErrorSq = sq((measurement - muHat).length());
    if (state.variances.steps_to_initialise == 0 &&
        measurementErrorSq > variance * CINS_GPS_INNOV_GATE) {
        return measurementErrorSq;
    }
    variance = gains.variance_lowpass * measurementErrorSq + (1.0 - gains.variance_lowpass) * variance;

    heapVars.ZInv = state.ZHat.inverse();
    const Vector2F& gains_AInvC = heapVars.ZInv.A() * ref_base;
    const GL2& CCT = GL2(ref_base.x*ref_base.x, ref_base.x*ref_base.y, ref_base.x*ref_base.y, ref_base.y*ref_base.y);
    const Vector3F& mu_Z = state.ZHat.W1()*gains_AInvC.x + state.ZHat.W2()*gains_AInvC.y;

    // Compute correction terms
    heapVars.S_Gamma = -0.5 * (gain_V) * heapVars.ZInv.A() * CCT * heapVars.ZInv.A().transposed();
    heapVars.W_Gamma_1 = - (measurement - mu_Z) * gains_AInvC.x * (gain_R+gain_V);
    heapVars.W_Gamma_2 = - (measurement - mu_Z) * gains_AInvC.y * (gain_R+gain_V);
    heapVars.GammaInv = SIM23(heapVars.I3, -heapVars.W_Gamma_1*dt, -heapVars.W_Gamma_2*dt, GL2::identity() - dt*heapVars.S_Gamma);

    heapVars.Omega_Delta = computeRotationCorrection((muHat - mu_Z), -(measurement - mu_Z), 4.*gain_R, dt);
    heapVars.W_Delta1 = (measurement - muHat) * gains_AInvC.x * (gain_R + gain_V);
    heapVars.W_Delta2 = (measurement - muHat) * gains_AInvC.y * (gain_R + gain_V);

    // Compute the bias correction

    // Construct the components of the 9x9 Adjoint matrix $\Ad_Z$
    // heapVars.AdZ_11 = state.ZHat.R();
    // heapVars.AdZ_12.zero(); // Zero matrix
    // heapVars.AdZ_13.zero(); // Zero matrix
    // heapVars.AdZ_21 = - Matrix3F::skew_symmetric(heapVars.ZInv.W1());
    // heapVars.AdZ_22 = state.ZHat.R() * heapVars.ZInv.A().a11();
    // heapVars.AdZ_23 = state.ZHat.R() * heapVars.ZInv.A().a21();
    // heapVars.AdZ_31 = - Matrix3F::skew_symmetric(heapVars.ZInv.W2());
    // heapVars.AdZ_32 = state.ZHat.R() * heapVars.ZInv.A().a12();
    // heapVars.AdZ_33 = state.ZHat.R() * heapVars.ZInv.A().a22();

    // Linearised measurement matrix $\mu - \hat{\mu} = Dh DR_{\hat{X}} \Ad_Z \varepsilon$.
    // const Matrix3F& pre_C11 = - Matrix3F::skew_symmetric(muHat);
    // const Matrix3F& pre_C12 = heapVars.I3*ref_base.x;
    // const Matrix3F& pre_C13 = heapVars.I3*ref_base.y;

    // const Matrix3F& C11 = pre_C11 * AdZ_11 + pre_C12 * AdZ_21 + pre_C13 * AdZ_31;
    // const Matrix3F& C12 = pre_C11 * AdZ_12 + pre_C12 * AdZ_22 + pre_C13 * AdZ_32;
    // const Matrix3F& C13 = pre_C11 * AdZ_13 + pre_C12 * AdZ_23 + pre_C13 * AdZ_33;
    heapVars.C11 = - Matrix3F::skew_symmetric(muHat) * state.ZHat.R();
    heapVars.C11 += -  Matrix3F::skew_symmetric(heapVars.ZInv.W1()) * ref_base.x;
    heapVars.C11 += - Matrix3F::skew_symmetric(heapVars.ZInv.W2()) * ref_base.y;
    heapVars.C12 = state.ZHat.R() * heapVars.ZInv.A().a11() * ref_base.x;
    heapVars.C12 += state.ZHat.R() * heapVars.ZInv.A().a12() * ref_base.y;
    heapVars.C13 = state.ZHat.R() * heapVars.ZInv.A().a21() * ref_base.x;
    heapVars.C13 += state.ZHat.R() * heapVars.ZInv.A().a22() * ref_base.y;

    // Gain matrix $\Delta = K (\mu - \hat{\mu})$.
    heapVars.K11 = Matrix3F::skew_symmetric(muHat - mu_Z) * 4. * gain_R * dt;
    heapVars.K21 = heapVars.I3 * gains_AInvC.x * (gain_R + gain_V) * dt;
    heapVars.K31 = heapVars.I3 * gains_AInvC.y * (gain_R + gain_V) * dt;

    // Compute the bias correction using the C matrices.
    // The correction to bias is given by the formula delta_b = k_b (MC)^\top (y-\hat{y})
    heapVars.M1Gyr = state.gyr_bias_gain_mat.rot;
    heapVars.M2Gyr = state.gyr_bias_gain_mat.vel;
    heapVars.M3Gyr = state.gyr_bias_gain_mat.pos;

    heapVars.M1Acc = state.acc_bias_gain_mat.rot;
    heapVars.M2Acc = state.acc_bias_gain_mat.vel;
    heapVars.M3Acc = state.acc_bias_gain_mat.pos;

    state.gyr_bias_correction = (heapVars.C11*heapVars.M1Gyr + heapVars.C12*heapVars.M2Gyr + heapVars.C13*heapVars.M3Gyr).mul_transpose(measurement - muHat) * gain_gyr_bias;
    state.acc_bias_correction = (heapVars.C11*heapVars.M1Acc + heapVars.C12*heapVars.M2Acc + heapVars.C13*heapVars.M3Acc).mul_transpose(measurement - muHat) * gain_acc_bias;
    saturate_bias(state.gyr_bias_correction, state.gyr_bias, gains.sat_gyr_bias.get(), dt);
    saturate_bias(state.acc_bias_correction, state.acc_bias, gains.sat_acc_bias.get(), dt);

    // Compute the bias gain matrix updates
    heapVars.A11 = heapVars.I3;
    heapVars.A12.zero(); // Zero matrix
    heapVars.A13.zero(); // Zero matrix
    heapVars.A21 = - Matrix3F::skew_symmetric(heapVars.GammaInv.W1());
    heapVars.A22 = heapVars.I3*heapVars.GammaInv.A().a11();
    heapVars.A23 = heapVars.I3*heapVars.GammaInv.A().a21();
    heapVars.A31 = - Matrix3F::skew_symmetric(heapVars.GammaInv.W2());
    heapVars.A32 = heapVars.I3*heapVars.GammaInv.A().a12();
    heapVars.A33 = heapVars.I3*heapVars.GammaInv.A().a22();

    state.gyr_bias_gain_mat.rot = (heapVars.A11 - heapVars.K11 * heapVars.C11) * heapVars.M1Gyr;
    state.gyr_bias_gain_mat.vel = (heapVars.A21 - heapVars.K21 * heapVars.C11) * heapVars.M1Gyr;
    state.gyr_bias_gain_mat.pos = (heapVars.A31 - heapVars.K31 * heapVars.C11) * heapVars.M1Gyr;
    state.acc_bias_gain_mat.rot = (heapVars.A11 - heapVars.K11 * heapVars.C11) * heapVars.M1Acc;
    state.acc_bias_gain_mat.vel = (heapVars.A21 - heapVars.K21 * heapVars.C11) * heapVars.M1Acc;
    state.acc_bias_gain_mat.pos = (heapVars.A31 - heapVars.K31 * heapVars.C11) * heapVars.M1Acc;
    state.gyr_bias_gain_mat.rot += (heapVars.A12 - heapVars.K11 * heapVars.C12) * heapVars.M2Gyr;
    state.gyr_bias_gain_mat.vel += (heapVars.A22 - heapVars.K21 * heapVars.C12) * heapVars.M2Gyr;
    state.gyr_bias_gain_mat.pos += (heapVars.A32 - heapVars.K31 * heapVars.C12) * heapVars.M2Gyr;
    state.acc_bias_gain_mat.rot += (heapVars.A12 - heapVars.K11 * heapVars.C12) * heapVars.M2Acc;
    state.acc_bias_gain_mat.vel += (heapVars.A22 - heapVars.K21 * heapVars.C12) * heapVars.M2Acc;
    state.acc_bias_gain_mat.pos += (heapVars.A32 - heapVars.K31 * heapVars.C12) * heapVars.M2Acc;
    state.gyr_bias_gain_mat.rot += (heapVars.A13 - heapVars.K11 * heapVars.C13) * heapVars.M3Gyr;
    state.gyr_bias_gain_mat.vel += (heapVars.A23 - heapVars.K21 * heapVars.C13) * heapVars.M3Gyr;
    state.gyr_bias_gain_mat.pos += (heapVars.A33 - heapVars.K31 * heapVars.C13) * heapVars.M3Gyr;
    state.acc_bias_gain_mat.rot += (heapVars.A13 - heapVars.K11 * heapVars.C13) * heapVars.M3Acc;
    state.acc_bias_gain_mat.vel += (heapVars.A23 - heapVars.K21 * heapVars.C13) * heapVars.M3Acc;
    state.acc_bias_gain_mat.pos += (heapVars.A33 - heapVars.K31 * heapVars.C13) * heapVars.M3Acc;

    // Apply the state and bias updates
    state.gyr_bias += state.gyr_bias_correction * dt;
    state.acc_bias += state.acc_bias_correction * dt;
    heapVars.Omega_Delta += state.gyr_bias_gain_mat.rot * state.gyr_bias_correction + state.acc_bias_gain_mat.rot * state.acc_bias_correction;
    heapVars.W_Delta1 += state.gyr_bias_gain_mat.pos * state.gyr_bias_correction + state.acc_bias_gain_mat.pos * state.acc_bias_correction;
    heapVars.W_Delta2 += state.gyr_bias_gain_mat.vel * state.gyr_bias_correction + state.acc_bias_gain_mat.vel * state.acc_bias_correction;

    // Construct the correction term Delta
    heapVars.Delta_SIM23 = SIM23(Matrix3F::from_angular_velocity(heapVars.Omega_Delta*dt), heapVars.W_Delta1*dt, heapVars.W_Delta2*dt, GL2::identity());
    heapVars.Delta_SIM23 = state.ZHat * heapVars.Delta_SIM23 * heapVars.ZInv;

    // Update the states using the correction terms
    state.XHat = Gal3F(heapVars.Delta_SIM23.R(), heapVars.Delta_SIM23.W2(), heapVars.Delta_SIM23.W1(), 0.) * state.XHat;
    state.ZHat = state.ZHat * heapVars.GammaInv;

    return measurementErrorSq;
}

/*
  initialise yaw from compass, if available
 */
bool AP_AHRS_CINS::init_yaw(void)
{
    ftype mag_yaw, dt;
    if (!get_compass_yaw(mag_yaw, dt)) {
        return false;
    }
    ftype roll_rad, pitch_rad, yaw_rad;
    state.XHat.rot().to_euler(&roll_rad, &pitch_rad, &yaw_rad);
    state.XHat.rot().from_euler(roll_rad, pitch_rad, mag_yaw);

    return true;
}

/*
  get yaw from compass
 */
bool AP_AHRS_CINS::get_compass_yaw(ftype &yaw_rad, ftype &dt)
{
    const auto &compass = AP::compass();
    if (compass.get_num_enabled() == 0) {
        return false;
    }
    const uint8_t mag_idx = compass.get_first_usable();
    if (!compass.healthy(mag_idx)) {
        return false;
    }
    if (!state.have_origin) {
        return false;
    }
    const auto &field = compass.get_field(mag_idx);
    const uint32_t last_us = compass.last_update_usec(mag_idx);
    if (last_us == last_mag_us) {
        // no new data
        return false;
    }
    dt = (last_us - last_mag_us) * 1.0e-6;
    last_mag_us = last_us;

    float declination_rad = compass.get_declination();
    if (is_zero(declination_rad)) {
        // the compass learns its declination from the AHRS location,
        // which CINS does not publish until its yaw is initialised, so
        // use the declination at our own origin:
        declination_rad = radians(AP_Declination::get_declination(state.origin.lat * 1.0e-7f,
                                                                  state.origin.lng * 1.0e-7f));
    }

    // const float cos_pitch_sq = 1.0f-(state.XHat.rot().c.x*state.XHat.rot().c.x);
    // const float headY = field.y * state.XHat.rot().c.z - field.z * state.XHat.rot().c.y;

    // // Tilt compensated magnetic field X component:
    // const float headX = field.x * cos_pitch_sq - state.XHat.rot().c.x * (field.y * state.XHat.rot().c.y + field.z * state.XHat.rot().c.z);

    // return magnetic yaw

    yaw_rad = wrap_PI(-atan2f(field.y,field.x) + declination_rad);

    return true;
}


bool AP_AHRS_CINS::get_compass_vector(Vector3F &mag_vec, Vector3F &mag_ref, ftype &dt)
{
    const auto &compass = AP::compass();
    if (compass.get_num_enabled() == 0) {
        return false;
    }
    const uint8_t mag_idx = compass.get_first_usable();
    if (!compass.healthy(mag_idx)) {
        return false;
    }
    if (!state.have_origin) {
        return false;
    }
    mag_vec = compass.get_field(mag_idx).toftype();
    const uint32_t last_us = compass.last_update_usec(mag_idx);
    if (last_us == last_mag_us) {
        // no new data
        return false;
    }
    dt = (last_us - last_mag_us) * 1.0e-6;
    last_mag_us = last_us;

    Location loc;
    if (!get_location(loc)) {
        return false;
    }
    mag_ref = AP_Declination::get_earth_field_ga(loc).toftype();
    if (mag_ref.is_zero()) {
        // wait for declination
        return false;
    }

    return true;
}

void AP_AHRS_CINS::update_attitude_from_compass()
{
    // Fuse the compass to constrain yaw. NOTE: with the current gains this is
    // largely ineffectual at holding yaw during a GPS blackout - the correction
    // is too weak to arrest gyro-bias-driven yaw drift, so heading remains
    // effectively observable only through GPS.
    ftype dt;
    Vector3F mag_vec, mag_ref;
    if (!get_compass_vector(mag_vec, mag_ref, dt)) {
        return;
    }
    mag_vec *= 1.e-3; // Convert mag measurement from milliGauss to Gauss


    Vector2F zero_2;
    const ftype magErrorSq = update_vector_measurement_cts(mag_ref, mag_vec, zero_2, gains.mag_att.get(), 0.0, gains.mag_gyr_bias.get(), gains.mag_acc_bias.get(), dt, state.variances.magVar);
    state.variances.magTest = magErrorSq / (state.variances.magVar * gains.variance_scale.get());
}

void AP_AHRS_CINS::compute_bias_update_imu(const SIM23& Gamma)
{
    // Compute the bias update for IMU inputs

    heapVars.GammaInv = Gamma.inverse();
    heapVars.A11 = Gamma.R();
    heapVars.A12.zero(); // Zero matrix
    heapVars.A13.zero(); // Zero matrix
    heapVars.A21 = - Matrix3F::skew_symmetric(heapVars.GammaInv.W1()) * Gamma.R();
    heapVars.A22 = Gamma.R() * heapVars.GammaInv.A().a11();
    heapVars.A23 = Gamma.R() * heapVars.GammaInv.A().a21();
    heapVars.A31 = - Matrix3F::skew_symmetric(heapVars.GammaInv.W2()) * Gamma.R();
    heapVars.A32 = Gamma.R() * heapVars.GammaInv.A().a12();
    heapVars.A33 = Gamma.R() * heapVars.GammaInv.A().a22();

    // Implement M(t+) = A M(t)
    heapVars.M1Gyr = state.gyr_bias_gain_mat.rot;
    heapVars.M2Gyr = state.gyr_bias_gain_mat.vel;
    heapVars.M3Gyr = state.gyr_bias_gain_mat.pos;
    state.gyr_bias_gain_mat.rot = heapVars.A11 * heapVars.M1Gyr;
    state.gyr_bias_gain_mat.rot += heapVars.A12 * heapVars.M2Gyr;
    state.gyr_bias_gain_mat.rot += heapVars.A13 * heapVars.M3Gyr;
    state.gyr_bias_gain_mat.vel = heapVars.A21 * heapVars.M1Gyr;
    state.gyr_bias_gain_mat.vel += heapVars.A22 * heapVars.M2Gyr;
    state.gyr_bias_gain_mat.vel += heapVars.A23 * heapVars.M3Gyr;
    state.gyr_bias_gain_mat.pos = heapVars.A31 * heapVars.M1Gyr;
    state.gyr_bias_gain_mat.pos += heapVars.A32 * heapVars.M2Gyr;
    state.gyr_bias_gain_mat.pos += heapVars.A33 * heapVars.M3Gyr;

    heapVars.M1Acc = state.acc_bias_gain_mat.rot;
    heapVars.M2Acc = state.acc_bias_gain_mat.vel;
    heapVars.M3Acc = state.acc_bias_gain_mat.pos;
    state.acc_bias_gain_mat.rot = heapVars.A11 * heapVars.M1Acc;
    state.acc_bias_gain_mat.rot += heapVars.A12 * heapVars.M2Acc;
    state.acc_bias_gain_mat.rot += heapVars.A13 * heapVars.M3Acc;
    state.acc_bias_gain_mat.vel = heapVars.A21 * heapVars.M1Acc;
    state.acc_bias_gain_mat.vel += heapVars.A22 * heapVars.M2Acc;
    state.acc_bias_gain_mat.vel += heapVars.A23 * heapVars.M3Acc;
    state.acc_bias_gain_mat.pos = heapVars.A31 * heapVars.M1Acc;
    state.acc_bias_gain_mat.pos += heapVars.A32 * heapVars.M2Acc;
    state.acc_bias_gain_mat.pos += heapVars.A33 * heapVars.M3Acc;
}


void AP_AHRS_CINS::saturate_bias(Vector3F& bias_correction, const Vector3F& current_bias, const ftype& saturation, const ftype& dt) const
{
    // Ensure that no part of the bias exceeds the saturation limit
    if (abs(bias_correction.x) > 1E-8) {
        bias_correction.x *= MIN(1., (saturation - abs(current_bias.x)) / (dt * abs(bias_correction.x)));
    }
    if (abs(bias_correction.y) > 1E-8) {
        bias_correction.y *= MIN(1., (saturation - abs(current_bias.y)) / (dt * abs(bias_correction.y)));
    }
    if (abs(bias_correction.z) > 1E-8) {
        bias_correction.z *= MIN(1., (saturation - abs(current_bias.z)) / (dt * abs(bias_correction.z)));
    }
}


Vector3F computeRotationCorrection(const Vector3F& v1, const Vector3F& v2, const ftype& gain, const ftype& dt)
{
    // The correction is given by Omega = - skew(v1) * v2 * gain, where v2 = R v0 for some const. v0.
    // It is applied as dR/dt = skew(Omega) R.
    // The key point is that Omega changes R which changes Omega.
    // We use a first order expansion to capture this effect here.

    const Vector3F omega1 = - Matrix3F::skew_symmetric(v1) * v2 * gain;
    const Vector3F omega2 = - Matrix3F::skew_symmetric(v1) * Matrix3F::skew_symmetric(omega1) * v2 * gain;
    const Vector3F omega3 = - Matrix3F::skew_symmetric(v1) * Matrix3F::skew_symmetric(omega2) * v2 * gain
                            - Matrix3F::skew_symmetric(v1) * Matrix3F::skew_symmetric(omega1) * Matrix3F::skew_symmetric(omega1) * v2 * gain;
    return omega1 + omega2*dt + omega3*0.5*dt*dt;
}

bool AP_AHRS_CINS::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    hgtVar = 0.;
    tasVar = 0.;
    if (state.variances.steps_to_initialise > 0) {
        velVar = 0.;
        posVar = 0.;
        magVar.zero();
        return false;
    }

    velVar = state.variances.velTest;
    posVar = state.variances.posTest;
    magVar.x = state.variances.magTest;
    magVar.y = state.variances.magTest;
    magVar.z = state.variances.magTest;

    return true;
}

#endif // AP_AHRS_CINS_ENABLED
