/*
  CINS state estimator for AP_AHRS, developed by Mr Patrick Wiltshire and Dr Pieter Van Goor
 */

#include "AP_AHRS_config.h"

#if AP_AHRS_CINS_ENABLED

#include "AP_AHRS_CINS.h"
#include "AP_AHRS.h"

#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
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
// reject a GPS measurement whose squared error exceeds this multiple of
// the low-passed running variance (a ~5-sigma innovation gate); this
// stops jammed/grossly-wrong GPS from diverging the observer
#define CINS_GPS_INNOV_GATE (25.0)
// the compass gate is ~3-sigma, like EKF3's default: at 5-sigma on the
// error vector's length, a heading error of any size passes wherever
// the horizontal field is weak
#define CINS_MAG_INNOV_GATE (9.0)
// the running variance the gate and the published test ratios use is
// at least the squared error expected from the sensor's noise, so a
// variance learnt while stationary doesn't reject normal manoeuvres.
// These are per-axis standard deviations, used when the GPS reports a
// smaller accuracy or none, and EKF3's default measurement noises:
#define CINS_GPS_VEL_NOISE_MIN 0.3 // m/s
#define CINS_GPS_POS_NOISE_MIN 0.5 // m
#define CINS_MAG_NOISE_MIN 0.05 // Gauss
// CINS is unhealthy once it has not propagated from the IMU for this
// long:
#define CINS_HEALTH_IMU_TIMEOUT_MS 500U
// CINS stops claiming a horizontal position once it has not fused a GPS
// position for this long:
#define CINS_GPS_FUSION_TIMEOUT_MS 5000U

// with a healthy baro, CINS publishes the baro height once it has not
// fused a GPS position for this long:
#define CINS_GPS_HEIGHT_TIMEOUT_MS 1000U
// once CINS has not fused a GPS position for this long, it resets its
// position and velocity to the next fix, as GPS which disagrees with
// its estimate by more than the innovation gate is otherwise rejected
// for ever:
#define CINS_GPS_RESET_TIMEOUT_MS 10000U
// longest interval between compass samples fused as one step:
#define CINS_MAG_MAX_DT 0.2

/*
  CINS is very fast, but stack hungry, using Os keeps stack usage down
 */
#pragma GCC optimize("Os")

// table of user settable parameters
const AP_Param::GroupInfo AP_AHRS_CINS::var_info[] = {

    // @Param: GPATT
    // @DisplayName: CINS GPS position attitude gain
    // @Description: CINS GPS position attitude gain
    // @Range: 0.000001 0.0001
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
    // @Range: 0.0005 0.05
    // @User: Advanced
    AP_GROUPINFO("MAGGB", 11, AP_AHRS_CINS, gains.mag_gyr_bias, CINS_GAIN_MAG_GYR_BIAS),

    // @Param: SATGB
    // @DisplayName: CINS gyro bias saturation
    // @Description: CINS gyro bias saturation
    // @Range: 0.01 0.1
    // @User: Advanced
    AP_GROUPINFO("SATGB", 12, AP_AHRS_CINS, gains.sat_gyr_bias, CINS_SAT_GYR_BIAS),


    // @Param: GPPAB
    // @DisplayName: CINS GPS position - accel. bias gain
    // @Description: CINS GPS position - accel. bias gain
    // @Range: 0.0001 0.01
    // @User: Advanced
    AP_GROUPINFO("GPPAB", 13, AP_AHRS_CINS, gains.gps_pos_acc_bias, CINS_GAIN_POS_ACC_BIAS),

    // @Param: GPVAB
    // @DisplayName: CINS GPS velocity - accel. bias gain
    // @Description: CINS GPS velocity - accel. bias gain
    // @Range: 0.0001 0.01
    // @User: Advanced
    AP_GROUPINFO("GPVAB", 14, AP_AHRS_CINS, gains.gps_vel_acc_bias, CINS_GAIN_VEL_ACC_BIAS),

    // @Param: MAGAB
    // @DisplayName: CINS magnetometer - accel. bias gain
    // @Description: CINS magnetometer (compass) accel. bias gain
    // @Range: 0.0005 0.05
    // @User: Advanced
    AP_GROUPINFO("MAGAB", 15, AP_AHRS_CINS, gains.mag_acc_bias, CINS_GAIN_MAG_ACC_BIAS),

    // @Param: SATAB
    // @DisplayName: CINS accel. bias saturation
    // @Description: CINS accel. bias saturation
    // @Range: 0.1 1.0
    // @User: Advanced
    AP_GROUPINFO("SATAB", 16, AP_AHRS_CINS, gains.sat_acc_bias, CINS_SAT_ACC_BIAS),

    // @Param: VARLP
    // @DisplayName: CINS variance low-pass gain
    // @Description: CINS variance low-pass gain used for health checks
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("VARLP", 17, AP_AHRS_CINS, gains.variance_lowpass, CINS_VAR_LOWPASS),

    // 18 was VARSC

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
    delayer.pending_input = Gal3F::identity();

    heapVars.I3.identity();
    heapVars.zero_vector.zero();
    heapVars.gravity_vector = Vector3F(0,0,GRAVITY_MSS);

    state.variances.velVar = 10.0;
    state.variances.posVar = 10.0;
    state.variances.magVar = 1.0;
    state.variances.steps_to_initialise = 100;
}
/*
  start the observer from a GPS position and velocity, keeping the
  attitude and the learnt biases.  The auxiliary state, the bias gain
  matrices, the GPS delay buffer and the measurement variances restart
  as they do at start-up, so the innovation gate is open until the
  variances have been relearnt.
 */
void AP_AHRS_CINS::reset_position_to_gps(const Vector3d &pos, const Vector3F &vel, ResetEvent event)
{
    if (had_position) {
        // tell the position controllers our position has stepped,
        // including when it is restored after a divergence:
        position_NE_reset_count++;
        position_D_reset_count++;
        log_reset(event, (pos - position_NED()).toftype());
    }
    had_position = true;
    // restart the observer's frame at this position:
    frame_offset_NED = pos;
    state.XHat = Gal3F(state.XHat.rot(), Vector3F{}, vel, state.XHat.tau());
    state.ZHat = SIM23::identity();

    state.gyr_bias_gain_mat.rot.zero();
    state.gyr_bias_gain_mat.vel.zero();
    state.gyr_bias_gain_mat.pos.zero();
    state.acc_bias_gain_mat.rot.zero();
    state.acc_bias_gain_mat.vel.zero();
    state.acc_bias_gain_mat.pos.zero();

    delayer.YR = Gal3F::identity();
    delayer.YR_stepper = Gal3F::identity();
    delayer.pending_input = Gal3F::identity();
    delayer.stamped_inputs.clear();
    delayer.last_push_time = delayer.internal_time;

    state.variances.velVar = 10.0;
    state.variances.posVar = 10.0;
    state.variances.steps_to_initialise = 100;

    last_gps_fused_ms = AP_HAL::millis();
}

void AP_AHRS_CINS::reset_gyro_drift(void)
{
    state.gyr_bias.zero();
    state.gyr_bias_correction.zero();
    state.gyr_bias_gain_mat.rot.zero();
    state.gyr_bias_gain_mat.vel.zero();
    state.gyr_bias_gain_mat.pos.zero();
}

void AP_AHRS_CINS::reset(void)
{
    reset_gyro_drift();
    state.acc_bias.zero();
    state.acc_bias_correction.zero();
    state.acc_bias_gain_mat.rot.zero();
    state.acc_bias_gain_mat.vel.zero();
    state.acc_bias_gain_mat.pos.zero();
    // re-initialise the attitude from the accelerometer and compass on
    // the next update.  The origin, position and velocity are kept.
    done_attitude_init = false;
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

    if (!done_attitude_init) {
        done_attitude_init = init_attitude(accel);
        if (done_attitude_init) {
            if (had_attitude) {
                // tell the attitude controllers our attitude has stepped:
                attitude_reset_count++;
            }
            had_attitude = true;
        }
    }

    const uint32_t last_gps_fused_before_ms = last_gps_fused_ms;

    if (initialised()) {
        update_imu(gyro, accel, dangle_dt);
        last_imu_update_ms = AP_HAL::millis();
        check_divergence();
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
                // no common origin has been set into us; use this fix
                state.have_origin = true;
                state.origin = loc;
                origin_is_own = true;
            }
            const auto & vel = gps.velocity();
            const Vector3d pos = state.origin.get_distance_NED_double(loc);

            if (!done_pos_init) {
                // start from the first fix, which need not be at the
                // origin when the origin is a common one:
                reset_position_to_gps(pos, vel.toftype(), ResetEvent::GPS);
                done_pos_init = true;
            } else if (initialised() &&
                       AP_HAL::millis() - last_gps_fused_ms > CINS_GPS_RESET_TIMEOUT_MS) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CINS: reset position to GPS");
                reset_position_to_gps(pos, vel.toftype(), ResetEvent::GPS);
            }

            update_gps((pos - frame_offset_NED).toftype(), vel.toftype(), gps_dt);
            check_divergence();
        }
    }

    update_attitude_from_compass();
    check_divergence();

    update_baro_height(last_gps_fused_ms != last_gps_fused_before_ms);

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
                                position_NED().x,
                                position_NED().y,
                                position_NED().z,
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

/*
  CINS has no height source other than GPS, so without GPS fusion its
  height and vertical velocity come from the IMU alone and drift
  without bound.  Once it stops fusing GPS, publish the baro height
  instead, offset to match CINS's own height at the last fused GPS
  sample
 */
void AP_AHRS_CINS::update_baro_height(bool gps_fused)
{
    const auto &baro = AP::baro();
    const bool baro_healthy = baro.healthy();
    if (baro_healthy && gps_fused) {
        // anchor to our height only when a GPS sample has been fused
        // in this update, so IMU drift since then doesn't stay in the
        // baro height:
        baro_offset_D = position_NED().z + baro.get_altitude();
        have_baro_offset = true;
    }

    // switch to the baro as soon as GPS samples stop being fused.
    // The offset is kept through a baro dropout, so the baro height
    // can be used again when the baro recovers.  Without a baro,
    // CINS's own height is kept for as long as its horizontal
    // position:
    HeightSource source = HeightSource::NONE;
    if (healthy() && AP_HAL::millis() - last_gps_fused_ms <= CINS_GPS_HEIGHT_TIMEOUT_MS) {
        source = HeightSource::CINS;
    } else if (healthy() && baro_healthy && have_baro_offset) {
        source = HeightSource::BARO;
    } else if (fusing_gps()) {
        source = HeightSource::CINS;
    }
    if (source != height_source) {
        // the published height may step when its source changes:
        position_D_reset_count++;
        ftype old_D, new_D;
        const bool have_step = published_height_D(height_source, old_D) &&
                               published_height_D(source, new_D);
        height_source = source;
        log_reset(ResetEvent::HEIGHT_SOURCE, Vector3F{0, 0, have_step ? new_D - old_D : 0});
    }
}

// true if the estimator state is finite and physically plausible:
bool AP_AHRS_CINS::state_is_sane() const
{
    const Vector3F &pos = state.XHat.pos();
    const Vector3F &vel = state.XHat.vel();
    if (!attitude_is_sane() ||
        pos.is_nan() || pos.is_inf() || vel.is_nan() || vel.is_inf()) {
        return false;
    }
    return pos.length() < CINS_MAX_POSITION_M &&
        position_NED().length() < CINS_MAX_POSITION_M &&
        vel.length() < CINS_MAX_VELOCITY_MS;
}

// true if the attitude and the biases are finite:
bool AP_AHRS_CINS::attitude_is_sane() const
{
    const Matrix3F &rot = state.XHat.rot();
    return !(rot.a.is_nan() || rot.a.is_inf() ||
             rot.b.is_nan() || rot.b.is_inf() ||
             rot.c.is_nan() || rot.c.is_inf() ||
             state.gyr_bias.is_nan() || state.gyr_bias.is_inf() ||
             state.acc_bias.is_nan() || state.acc_bias.is_inf());
}

/*
  restart the estimator if it has diverged, rather than publish its
  state or let it overflow.  The origin is kept.  If only the
  position or velocity diverged, the attitude and biases are kept and
  the position restarts from the next GPS fix; otherwise the attitude
  is initialised again from the sensors too
 */
void AP_AHRS_CINS::check_divergence()
{
    if (!initialised() || state_is_sane()) {
        return;
    }
    position_NE_reset_count++;
    position_D_reset_count++;
    done_pos_init = false;
    if (attitude_is_sane()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CINS: position diverged, resetting");
        state.XHat = Gal3F(state.XHat.rot(), Vector3F{}, Vector3F{}, state.XHat.tau());
        log_reset(ResetEvent::DIVERGED, Vector3F{});
        return;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CINS: diverged, reinitialising");
    init();
    state.gyr_bias_correction.zero();
    state.acc_bias_correction.zero();
    delayer.stamped_inputs.clear();
    delayer.last_push_time = delayer.internal_time;
    done_attitude_init = false;
    log_reset(ResetEvent::ATTITUDE_DIVERGED, Vector3F{});
}

// the down position published for a height source:
bool AP_AHRS_CINS::published_height_D(HeightSource source, ftype &posD) const
{
    switch (source) {
    case HeightSource::CINS:
        posD = position_NED().z;
        return true;
    case HeightSource::BARO:
        posD = baro_offset_D - AP::baro().get_altitude();
        return true;
    case HeightSource::NONE:
        break;
    }
    return false;
}

/*
  log a reset of the position, or a change in the published height's
  source.  step is the change in the NED position
 */
void AP_AHRS_CINS::log_reset(ResetEvent event, const Vector3F &step) const
{
#if HAL_LOGGING_ENABLED
    // @LoggerMessage: CINR
    // @Description: CINS position resets and height source changes
    // @Field: TimeUS: Time since system startup
    // @Field: I: instance
    // @Field: Ev: event: 0=reset to GPS, 1=common origin adopted, 2=height source changed, 3=position or velocity diverged and restarted, 4=attitude diverged and everything restarted
    // @Field: HS: height source after the event: 0=none, 1=CINS, 2=baro
    // @Field: NEC: NE position reset count
    // @Field: DC: down position reset count
    // @Field: DN: position step north
    // @Field: DE: position step east
    // @Field: DD: position step down
    AP::logger().Write("CINR", "TimeUS,I,Ev,HS,NEC,DC,DN,DE,DD",
                       "s#----mmm",
                       "F-----000",
                       "QBBBHHfff",
                       AP_HAL::micros64(),
                       0,
                       uint8_t(event),
                       uint8_t(height_source),
                       position_NE_reset_count,
                       position_D_reset_count,
                       float(step.x),
                       float(step.y),
                       float(step.z));
#endif
}

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
    // re-initialising the attitude resets the yaw as well:
    results.attitude_reset_count = attitude_reset_count;
    results.yaw_reset_count = attitude_reset_count;

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
    // without GPS the velocity comes from integrating the
    // accelerometers and drifts, so only claim it while fusing GPS:
    results.velocity_NED = state.XHat.vel().tofloat();
    results.velocity_NED_valid = fusing_gps();

    // ground velocity estimate in meters/second, in North/East order
    // note: velocity_NE is significantly different to results.velocity_NED.xy()!
    results.velocity_NE = results.velocity_NED.xy();

    if (height_source == HeightSource::BARO) {
        results.vert_pos_rate_D = -AP::baro().get_climb_rate();
        results.vert_pos_rate_D_valid = true;
        // the published height is the baro's, so pair it with the
        // baro climb rate rather than the IMU's vertical velocity in
        // the velocity vector the position controllers prefer:
        results.velocity_NED.z = results.vert_pos_rate_D;
    } else {
        results.vert_pos_rate_D = results.velocity_NED.z;
        results.vert_pos_rate_D_valid = (height_source == HeightSource::CINS);
    }

    // without GPS the horizontal position is unconstrained, so only
    // claim one while fusing GPS:
    results.location_valid = get_location(results.location) && fusing_gps();

    // CINS takes a common origin from the other backends.  It only
    // offers its own when it is the configured backend: its origin is
    // its first fix, without the GPS quality checks the EKFs make
    // before setting theirs, so it must not become the origin of an
    // EKF which has been chosen instead:
    results.provides_common_origin = (AP::ahrs().configured_ekf_type() == AP_AHRS::EKFType::CINS);

    // origin-relative position (see get_origin()):
    const Vector3d pos = position_NED();
    if (fusing_gps()) {
        results.position_NE = Vector2p{postype_t(pos.x), postype_t(pos.y)};
        results.position_NE_valid = true;
    }
    results.position_NE_reset_count = position_NE_reset_count;
    if (height_source == HeightSource::BARO) {
        results.position_D = baro_offset_D - AP::baro().get_altitude();
        results.position_D_valid = true;
    } else if (height_source == HeightSource::CINS) {
        results.position_D = pos.z;
        results.position_D_valid = true;
    }
    results.position_D_reset_count = position_D_reset_count;
    // the location's altitude follows the published height, so it is
    // the baro's too while that is in use:
    if (results.location_valid && height_source == HeightSource::BARO) {
        results.location.alt = state.origin.alt - int32_t(results.position_D * 100);
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
    // it reports no valid nav outputs).  While propagating from the
    // IMU we provide attitude, and height from the baro; velocity,
    // position and use of GPS only while fusing GPS:
    memset(&results.filter_status, 0, sizeof(results.filter_status));
    if (healthy()) {
        results.filter_status.flags.attitude = true;
        results.filter_status.flags.initalized = true;
    }
    if (results.vert_pos_rate_D_valid) {
        results.filter_status.flags.vert_vel = true;
    }
    if (results.position_D_valid) {
        results.filter_status.flags.vert_pos = true;
    }
    if (fusing_gps()) {
        results.filter_status.flags.horiz_vel = true;
        results.filter_status.flags.horiz_pos_rel = true;
        results.filter_status.flags.horiz_pos_abs = true;
        results.filter_status.flags.pred_horiz_pos_rel = true;
        results.filter_status.flags.pred_horiz_pos_abs = true;
        results.filter_status.flags.using_gps = true;
    }
    results.filter_status_valid = true;

    get_control_limits(results.control_ground_speed_limit_ms,
                       results.control_gain_scaler_XY,
                       results.control_gain_scaler_Z);

    // innovation test ratios, used by the vehicles' EKF failsafes:
    results.variances_valid = get_variances(results.velVar, results.posVar, results.hgtVar, results.magVar, results.tasVar);

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
    return true;
}

bool AP_AHRS_CINS::fusing_gps(void) const
{
    return healthy() &&
        AP_HAL::millis() - last_gps_fused_ms <= CINS_GPS_FUSION_TIMEOUT_MS;
}

bool AP_AHRS_CINS::set_origin(const Location &loc)
{
    // take a common origin from another backend if we have no
    // origin.  When CINS is not the configured backend its own origin,
    // taken from its first fix, only stands in until the configured
    // backend chooses one, so a later common origin replaces it;
    // otherwise switching to CINS would change the navigation frame:
    if (state.have_origin &&
        (!origin_is_own || AP::ahrs().configured_ekf_type() == AP_AHRS::EKFType::CINS)) {
        return false;
    }
    const bool had_origin = state.have_origin;
    // our old origin, NED from the new one:
    const Vector3d origin_shift = loc.get_distance_NED_double(state.origin);
    state.origin = loc;
    state.have_origin = true;
    origin_is_own = false;
    if (had_origin && done_pos_init) {
        // keep our location, now relative to the new origin.  The
        // observer's own frame is unchanged:
        frame_offset_NED += origin_shift;
        baro_offset_D += origin_shift.z;
        position_NE_reset_count++;
        position_D_reset_count++;
        log_reset(ResetEvent::ORIGIN, origin_shift.toftype());
    }
    return true;
}

bool AP_AHRS_CINS::pre_arm_check(bool requires_position, char *failure_msg, uint8_t failure_msg_len) const
{
    if (!done_pos_init) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Waiting for GPS");
        return false;
    }
    if (!done_attitude_init) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Waiting for compass yaw");
        return false;
    }
    if (AP_HAL::millis() - last_gps_fused_ms > CINS_GPS_FUSION_TIMEOUT_MS) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Not fusing GPS");
        return false;
    }
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Not healthy");
        return false;
    }
    if (state.variances.steps_to_initialise > 0) {
        // until then CINS publishes no variances, so the vehicles'
        // variance checks cannot run:
        hal.util->snprintf(failure_msg, failure_msg_len, "Waiting for variances");
        return false;
    }
    // a test ratio above one means CINS is currently rejecting that
    // sensor's measurements at its innovation gate:
    if (state.variances.velTest > 1 || state.variances.posTest > 1) {
        hal.util->snprintf(failure_msg, failure_msg_len, "GPS inconsistent");
        return false;
    }
    if (state.variances.magTest > 1) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Compass inconsistent");
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

    const auto &gps = AP::gps();
    float vel_acc = 0, hacc = 0, vacc = 0;
    UNUSED_RESULT(gps.speed_accuracy(vel_acc));
    UNUSED_RESULT(gps.horizontal_accuracy(hacc));
    UNUSED_RESULT(gps.vertical_accuracy(vacc));
    vel_acc = MAX(vel_acc, CINS_GPS_VEL_NOISE_MIN);
    hacc = MAX(hacc, CINS_GPS_POS_NOISE_MIN);
    vacc = MAX(vacc, CINS_GPS_POS_NOISE_MIN);
    delayer.gps_vel_min_var = 3 * sq(vel_acc);
    delayer.gps_pos_min_var = 2 * sq(hacc) + sq(vacc);
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
    if (done_attitude_init) {
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

    // the buffer must hold the inputs spanning the GPS lag.  Push no
    // more often than lets it do that, combining consecutive inputs
    // into one entry when the loop rate is too high to keep one entry
    // per input (above about 500Hz at the default lag):
    delayer.pending_input = delayer.pending_input * rightMat;
    const double min_push_interval = gains.gps_lag / (CINS_MAX_HISTORY - 2);
    if (delayer.internal_time - delayer.last_push_time >= min_push_interval) {
        if (delayer.stamped_inputs.space() == 0) {
            // should not happen given the push interval; if it does,
            // apply the oldest input early rather than lose it
            struct stamped_Gal3F oldest;
            if (delayer.stamped_inputs.pop(oldest)) {
                delayer.YR_stepper = delayer.YR_stepper * oldest.gal3;
            }
        }
        delayer.stamped_inputs.push({delayer.internal_time, delayer.pending_input});
        delayer.pending_input = Gal3F::identity();
        delayer.last_push_time = delayer.internal_time;
    }

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
    bool vel_accepted;
    state.variances.velTest = update_vector_measurement_cts(mu_vel, mu0_vel, ref_vel, gains.gpsvel_att.get(), gains.gps_vel.get(), gains.gps_vel_gyr_bias.get(), gains.gps_vel_acc_bias.get(), dt, state.variances.velVar, delayer.gps_vel_min_var, CINS_GPS_INNOV_GATE, vel_accepted);


    const Vector3F& mu0_pos = YRInv.pos();
    const Vector2F& ref_pos = Vector2F(-current_lag, 1.);
    const Vector3F mu_pos = delayer.gps_pos - left_delay_vel * ref_pos.x - left_delay_pos * ref_pos.y;
    bool pos_accepted;
    state.variances.posTest = update_vector_measurement_cts(mu_pos, mu0_pos, ref_pos, gains.gpspos_att.get(), gains.gps_pos.get(), gains.gps_pos_gyr_bias.get(), gains.gps_pos_acc_bias.get(), dt, state.variances.posVar, delayer.gps_pos_min_var, CINS_GPS_INNOV_GATE, pos_accepted);

    if (pos_accepted) {
        last_gps_fused_ms = AP_HAL::millis();
    }

    // The CINS algorithm does not have an internal variance state like an EKF.
    // Instead, we compute the variance by applying a low-pass to the squared measurement errors.
    // If a new squared measurement error arrives that is much larger than this low-passed value, it indicates something is wrong.
    // This only works after initialising for a number of steps first.
    if (state.variances.steps_to_initialise > 0) {
        --state.variances.steps_to_initialise;
    }

}

ftype AP_AHRS_CINS::update_vector_measurement_cts(const Vector3F &measurement, const Vector3F& reference, const Vector2F &ref_base, const ftype& gain_R, const ftype& gain_V, const ftype& gain_gyr_bias, const ftype& gain_acc_bias, const ftype dt, ftype &variance, const ftype min_variance, const ftype gate, bool &accepted)
{
    // Compute and apply an update for an arbitrary vector-type measurement.
    // The measurement is of the form $\mu = R \mu_0 + V C$, where
    // \mu is the measured value
    // \mu_0 is a reference vector (e.g. zero for GPS and 'north' for magnetometer)
    // C is a the reference vector 'base' (e.g. zero for magnetometer and (1,0) or (0,1) for GPS vel. or pos.)
    // The return value is the test ratio: the squared measurement
    // error as a fraction of the innovation gate
    // The update equations are then drawn from Lemma 5.3 in
    // van Goor, Pieter, et al. "Constructive synchronous observer design for inertial navigation with delayed GNSS measurements." European Journal of Control (2024): 101047.

    // Set up some computation variables
    const Vector3F& muHat = state.XHat.rot() * reference + state.XHat.vel()*ref_base.x + state.XHat.pos()*ref_base.y;

    // innovation gating: skip the correction (and the running-variance
    // update) for a measurement whose squared error exceeds gate
    // times the low-passed running variance, once the
    // variance estimate has settled.  Leaving the observer state and the
    // variance untouched means a jammed or grossly-wrong GPS fix can
    // neither diverge the observer nor inflate the variance (which would
    // re-open the gate); accepted measurements keep the variance tracking
    // so a run of rejections cannot lock the gate closed.
    const ftype measurementErrorSq = sq((measurement - muHat).length());
    const ftype testRatio = measurementErrorSq / (MAX(variance, min_variance) * gate);
    if (state.variances.steps_to_initialise == 0 && testRatio > 1) {
        accepted = false;
        return testRatio;
    }
    accepted = true;
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
    heapVars.W_Delta1 += state.gyr_bias_gain_mat.vel * state.gyr_bias_correction + state.acc_bias_gain_mat.vel * state.acc_bias_correction;
    heapVars.W_Delta2 += state.gyr_bias_gain_mat.pos * state.gyr_bias_correction + state.acc_bias_gain_mat.pos * state.acc_bias_correction;

    // Construct the correction term Delta
    heapVars.Delta_SIM23 = SIM23(Matrix3F::from_angular_velocity(heapVars.Omega_Delta*dt), heapVars.W_Delta1*dt, heapVars.W_Delta2*dt, GL2::identity());
    heapVars.Delta_SIM23 = state.ZHat * heapVars.Delta_SIM23 * heapVars.ZInv;

    // Update the states using the correction terms
    state.XHat = Gal3F(heapVars.Delta_SIM23.R(), heapVars.Delta_SIM23.W2(), heapVars.Delta_SIM23.W1(), 0.) * state.XHat;
    state.ZHat = state.ZHat * heapVars.GammaInv;

    return testRatio;
}

/*
  initialise the attitude from the accelerometer and compass
 */
bool AP_AHRS_CINS::init_attitude(const Vector3F &accel)
{
    // take roll and pitch from the accelerometer, assuming the vehicle
    // is not accelerating (e.g. it is on the ground), and yaw from the
    // compass, tilt-compensated by them
    const ftype roll_rad = atan2F(-accel.y, -accel.z);
    const ftype pitch_rad = atan2F(accel.x, sqrtF(sq(accel.y) + sq(accel.z)));
    ftype yaw_rad;
    if (!get_compass_yaw(roll_rad, pitch_rad, yaw_rad)) {
        return false;
    }
    state.XHat.rot().from_euler(roll_rad, pitch_rad, yaw_rad);

    return true;
}

/*
  get yaw from compass
 */
bool AP_AHRS_CINS::get_compass_yaw(ftype roll_rad, ftype pitch_rad, ftype &yaw_rad)
{
    const auto &compass = AP::compass();
    if (compass.get_num_enabled() == 0) {
        return false;
    }
    if (compass.is_calibrating()) {
        // the offsets and scaling are being replaced
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
    last_mag_us = last_us;

    float declination_rad = compass.get_declination();
    if (is_zero(declination_rad)) {
        // the compass learns its declination from the AHRS location,
        // which CINS does not publish until its attitude is initialised, so
        // use the declination at our own origin:
        declination_rad = radians(AP_Declination::get_declination(state.origin.lat * 1.0e-7f,
                                                                  state.origin.lng * 1.0e-7f));
    }

    // rotate the field into a level frame using the roll and pitch,
    // and take the magnetic heading from its horizontal components:
    Matrix3F tilt;
    tilt.from_euler(roll_rad, pitch_rad, 0);
    const Vector3F field_level = tilt * field.toftype();
    yaw_rad = wrap_PI(atan2F(-field_level.y, field_level.x) + declination_rad);

    return true;
}


bool AP_AHRS_CINS::get_compass_vector(Vector3F &mag_vec, Vector3F &mag_ref, ftype &dt)
{
    const auto &compass = AP::compass();
    if (compass.get_num_enabled() == 0) {
        return false;
    }
    if (compass.is_calibrating()) {
        // the offsets and scaling are being replaced
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
    // bound the step, e.g. for the first sample, or after the compass
    // has been unavailable:
    dt = MIN((last_us - last_mag_us) * 1.0e-6, CINS_MAG_MAX_DT);
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

    // the attitude is only meaningful, and the observer only
    // propagating, once it has been initialised:
    if (!initialised()) {
        return;
    }

    ftype dt;
    Vector3F mag_vec, mag_ref;
    if (!get_compass_vector(mag_vec, mag_ref, dt)) {
        return;
    }
    mag_vec *= 1.e-3; // Convert mag measurement from milliGauss to Gauss

    // the innovation gate is open while the variances initialise, so
    // don't fuse a field whose strength is far from the expected earth
    // field, e.g. from an uncalibrated compass:
    const ftype field_ratio = mag_vec.length() / mag_ref.length();
    if (field_ratio < 0.5 || field_ratio > 2.0) {
        return;
    }


    Vector2F zero_2;
    bool mag_accepted;
    state.variances.magTest = update_vector_measurement_cts(mag_ref, mag_vec, zero_2, gains.mag_att.get(), 0.0, gains.mag_gyr_bias.get(), gains.mag_acc_bias.get(), dt, state.variances.magVar, 3 * sq(CINS_MAG_NOISE_MIN), CINS_MAG_INNOV_GATE, mag_accepted);
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
    // limit the correction so that no part of the resulting bias
    // exceeds the saturation limit.  Limiting the resulting bias,
    // rather than scaling the correction by the remaining headroom,
    // still lets a saturated bias move back towards zero:
    for (uint8_t i=0; i<3; i++) {
        const ftype next_bias = constrain_ftype(current_bias[i] + bias_correction[i] * dt, -saturation, saturation);
        bias_correction[i] = (next_bias - current_bias[i]) / dt;
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

    // like the EKFs, report the square root of each test ratio: the
    // squared error as a fraction of the innovation gate, so 1.0 is
    // the point at which CINS rejects the measurement:
    velVar = sqrtF(state.variances.velTest);
    posVar = sqrtF(state.variances.posTest);
    magVar.x = sqrtF(state.variances.magTest);
    magVar.y = magVar.x;
    magVar.z = magVar.x;

    return true;
}

#endif // AP_AHRS_CINS_ENABLED
