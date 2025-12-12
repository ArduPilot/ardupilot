#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor_SITL.h"
#include <AP_Logger/AP_Logger.h>
#include <SITL/SITL.h>
#include <AP_Filesystem/AP_Filesystem.h>

#if AP_SIM_INS_ENABLED

const extern AP_HAL::HAL& hal;

AP_InertialSensor_SITL::AP_InertialSensor_SITL(AP_InertialSensor &imu, const uint16_t sample_rates[]) :
    AP_InertialSensor_Backend(imu),
    gyro_sample_hz(sample_rates[0]),
    accel_sample_hz(sample_rates[1])
{
}

/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_SITL::detect(AP_InertialSensor &_imu, const uint16_t sample_rates[])
{
    AP_InertialSensor_SITL *sensor = NEW_NOTHROW AP_InertialSensor_SITL(_imu, sample_rates);
    if (sensor == nullptr) {
        return nullptr;
    }
    if (!sensor->init_sensor()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_InertialSensor_SITL::init_sensor(void)
{
    sitl = AP::sitl();
    if (sitl == nullptr) {
        return false;
    }
    return true;
}

// calculate a noisy noise component
static float calculate_noise(float noise, float noise_variation) {
    return noise * (1.0f + noise_variation * rand_float());
}

float AP_InertialSensor_SITL::get_temperature(void)
{
#if HAL_INS_TEMPERATURE_CAL_ENABLE
    if (!is_zero(sitl->imu_temp_fixed)) {
        // user wants fixed temperature
        return sitl->imu_temp_fixed;
    }
    uint32_t now = AP_HAL::millis();
    if (temp_start_ms == 0) {
        temp_start_ms = now;
    }
    // follow a curve with given start, end and time constant
    const float tsec = (AP_HAL::millis() - temp_start_ms) * 0.001f;
    const float T0 = sitl->imu_temp_start;
    const float T1 = sitl->imu_temp_end;
    const float tconst = sitl->imu_temp_tconst;
    return T1 - (T1 - T0) * expf(-tsec / tconst);
#else
    return 20.0f;
#endif
}

/*
  generate an accelerometer sample
 */
void AP_InertialSensor_SITL::generate_accel()
{
    Vector3f accel_accum;
    uint8_t nsamples = enable_fast_sampling(accel_instance) ? 4 : 1;

    for (uint8_t j = 0; j < nsamples; j++) {

        Vector3f accel = Vector3f(sitl->state.xAccel,
                                  sitl->state.yAccel,
                                  sitl->state.zAccel);

        const Vector3f &accel_trim = sitl->accel_trim.get();
        if (!accel_trim.is_zero()) {
            Matrix3f trim_rotation;
            trim_rotation.from_euler(accel_trim.x, accel_trim.y, 0);
            accel = trim_rotation.transposed() * accel;
        }

        // add scaling
        Vector3f accel_scale = sitl->accel_scale[accel_instance].get();
        // note that we divide so the SIM_ACC values match the
        // INS_ACCSCAL values
        if (!is_zero(accel_scale.x)) {
            accel.x /= accel_scale.x;
        }
        if (!is_zero(accel_scale.y)) {
            accel.y /= accel_scale.y;
        }
        if (!is_zero(accel_scale.z)) {
            accel.z /= accel_scale.z;
        }

        // apply bias
        const Vector3f &accel_bias = sitl->accel_bias[accel_instance].get();
        accel += accel_bias;

        // minimum noise levels are 2 bits, but averaged over many
        // samples, giving around 0.01 m/s/s
        float accel_noise = 0.01f;
        float noise_variation = 0.05f;
        // this smears the individual motor peaks somewhat emulating physical motors
        float freq_variation = 0.12f;

        // add in sensor noise
        accel += Vector3f{rand_float(), rand_float(), rand_float()} * accel_noise;

        bool motors_on = sitl->throttle > sitl->ins_noise_throttle_min;

        // on a real 180mm copter gyro noise varies between 0.8-4 m/s/s for throttle 0.2-0.8
        // giving a accel noise variation of 5.33 m/s/s over the full throttle range
        if (motors_on) {
            // add extra noise when the motors are on
            accel_noise = sitl->accel_noise[accel_instance];
        }

        // VIB_FREQ is a static vibration applied to each axis
        const Vector3f &vibe_freq = sitl->vibe_freq;

        if (!vibe_freq.is_zero() && motors_on) {
            accel.x += sinf(accel_time * 2 * M_PI * vibe_freq.x) * calculate_noise(accel_noise, noise_variation);
            accel.y += sinf(accel_time * 2 * M_PI * vibe_freq.y) * calculate_noise(accel_noise, noise_variation);
            accel.z += sinf(accel_time * 2 * M_PI * vibe_freq.z) * calculate_noise(accel_noise, noise_variation);
            accel_time += 1.0f / (accel_sample_hz * nsamples);
        }

        // VIB_MOT_MAX is a rpm-scaled vibration applied to each axis
        if (!is_zero(sitl->vibe_motor) && motors_on) {
            uint32_t mask = sitl->state.motor_mask;
            uint8_t mbit;
            while ((mbit = __builtin_ffs(mask)) != 0) {
                const uint8_t motor = mbit-1;
                mask &= ~(1U<<motor);
                uint32_t harmonics = uint32_t(sitl->vibe_motor_harmonics);
                const float base_freq = calculate_noise(sitl->state.rpm[motor] / 60.0f, freq_variation);
                while (harmonics != 0) {
                    const uint8_t bit = __builtin_ffs(harmonics);
                    harmonics &= ~(1U<<(bit-1U));
                    const float phase = accel_motor_phase[motor] * float(bit);
                    accel.x += sinf(phase) * calculate_noise(accel_noise * sitl->vibe_motor_scale, noise_variation);
                    accel.y += sinf(phase) * calculate_noise(accel_noise * sitl->vibe_motor_scale, noise_variation);
                    accel.z += sinf(phase) * calculate_noise(accel_noise * sitl->vibe_motor_scale, noise_variation);
                }
                const float phase_incr = base_freq * 2 * M_PI / (accel_sample_hz * nsamples);
                accel_motor_phase[motor] = wrap_PI(accel_motor_phase[motor] + phase_incr);
            }
        }

        // correct for the acceleration due to the IMU position offset and angular acceleration
        // correct for the centripetal acceleration
        // only apply corrections to first accelerometer
        Vector3f pos_offset = sitl->imu_pos_offset;
        if (!pos_offset.is_zero()) {
            // calculate sensed acceleration due to lever arm effect
            // Note: the % operator has been overloaded to provide a cross product
            Vector3f angular_accel = Vector3f(radians(sitl->state.angAccel.x), radians(sitl->state.angAccel.y), radians(sitl->state.angAccel.z));
            Vector3f lever_arm_accel = angular_accel % pos_offset;

            // calculate sensed acceleration due to centripetal acceleration
            Vector3f angular_rate = Vector3f(radians(sitl->state.rollRate), radians(sitl->state.pitchRate), radians(sitl->state.yawRate));
            Vector3f centripetal_accel = angular_rate % (angular_rate % pos_offset);

            // apply corrections
            accel += lever_arm_accel + centripetal_accel;
        }

        if (fabsf(sitl->accel_fail[accel_instance]) > 1.0e-6f) {
            accel.x = accel.y = accel.z = sitl->accel_fail[accel_instance];
        }

#if HAL_INS_TEMPERATURE_CAL_ENABLE
        const float T = get_temperature();
        sitl->imu_tcal[gyro_instance].sitl_apply_accel(T, accel);
#endif

        _notify_new_accel_sensor_rate_sample(accel_instance, accel);

#if AP_SIM_INS_FILE_ENABLED
        if (sitl->accel_file_rw == SITL::SIM::INSFileMode::INS_FILE_WRITE) {
            write_accel_to_file(accel);
        }
#endif
        accel_accum += accel;
    }

    accel_accum /= nsamples;

    accel_accum.rotate(sitl->imu_orientation);

    _rotate_and_correct_accel(accel_instance, accel_accum);
    _notify_new_accel_raw_sample(accel_instance, accel_accum, AP_HAL::micros64());

    _publish_temperature(accel_instance, get_temperature());
}

/*
  generate a gyro sample
 */
void AP_InertialSensor_SITL::generate_gyro()
{
    Vector3f gyro_accum;
    uint8_t nsamples = enable_fast_sampling(gyro_instance) ? 8 : 1;

    const float _gyro_drift = gyro_drift();
    for (uint8_t j = 0; j < nsamples; j++) {
        float p = radians(sitl->state.rollRate) + _gyro_drift;
        float q = radians(sitl->state.pitchRate) + _gyro_drift;
        float r = radians(sitl->state.yawRate) + _gyro_drift;

        // minimum gyro noise is less than 1 bit
        float gyro_noise = radians(0.04f);
        constexpr float noise_variation = 0.05f;
        // this smears the individual motor peaks somewhat emulating physical motors
        constexpr float freq_variation = 0.12f;
        // add in sensor noise
        p += gyro_noise * rand_float();
        q += gyro_noise * rand_float();
        r += gyro_noise * rand_float();

        bool motors_on = sitl->throttle > sitl->ins_noise_throttle_min;
        // on a real 180mm copter gyro noise varies between 0.2-0.4 rad/s for throttle 0.2-0.8
        // giving a gyro noise variation of 0.33 rad/s or 20deg/s over the full throttle range
        if (motors_on) {
            // add extra noise when the motors are on
            gyro_noise = radians(sitl->gyro_noise[gyro_instance]) * sitl->throttle;
        }

        // VIB_FREQ is a static vibration applied to each axis
        const Vector3f &vibe_freq = sitl->vibe_freq;

        if (vibe_freq.is_zero() && is_zero(sitl->vibe_motor)) {
            // no rpm noise, so add in background noise if any
            p += gyro_noise * rand_float();
            q += gyro_noise * rand_float();
            r += gyro_noise * rand_float();
        }

        if (!vibe_freq.is_zero() && motors_on) {
            p += sinf(gyro_time * 2 * M_PI * vibe_freq.x) * calculate_noise(gyro_noise, noise_variation);
            q += sinf(gyro_time * 2 * M_PI * vibe_freq.y) * calculate_noise(gyro_noise, noise_variation);
            r += sinf(gyro_time * 2 * M_PI * vibe_freq.z) * calculate_noise(gyro_noise, noise_variation);
            gyro_time += 1.0f / (gyro_sample_hz * nsamples);
        }

        // VIB_MOT_MAX is a rpm-scaled vibration applied to each axis
        if (!is_zero(sitl->vibe_motor) && motors_on) {
            uint32_t mask = sitl->state.motor_mask;
            uint8_t mbit;
            while ((mbit = __builtin_ffs(mask)) != 0) {
                const uint8_t motor = mbit-1;
                mask &= ~(1U<<motor);
                uint32_t harmonics = uint32_t(sitl->vibe_motor_harmonics);
                const float base_freq = calculate_noise(sitl->state.rpm[motor] / 60.0f, freq_variation);
                while (harmonics != 0) {
                    const uint8_t bit = __builtin_ffs(harmonics);
                    harmonics &= ~(1U<<(bit-1U));
                    const float phase = gyro_motor_phase[motor] * float(bit);
                    p += sinf(phase) * calculate_noise(gyro_noise * sitl->vibe_motor_scale, noise_variation);
                    q += sinf(phase) * calculate_noise(gyro_noise * sitl->vibe_motor_scale, noise_variation);
                    r += sinf(phase) * calculate_noise(gyro_noise * sitl->vibe_motor_scale, noise_variation);
                }
                const float phase_incr = base_freq * 2 * M_PI / (gyro_sample_hz * nsamples);
                gyro_motor_phase[motor] = wrap_PI(gyro_motor_phase[motor] + phase_incr);
            }
        }

        Vector3f gyro {p, q, r};

#if HAL_INS_TEMPERATURE_CAL_ENABLE
        sitl->imu_tcal[gyro_instance].sitl_apply_gyro(get_temperature(), gyro);
#endif

        // add in gyro scaling
        const Vector3f &scale = sitl->gyro_scale[gyro_instance];
        gyro.x *= (1 + scale.x * 0.01f);
        gyro.y *= (1 + scale.y * 0.01f);
        gyro.z *= (1 + scale.z * 0.01f);

        // apply bias
        const Vector3f &gyro_bias = sitl->gyro_bias[gyro_instance].get();
        gyro += gyro_bias;

        gyro_accum += gyro;
        _notify_new_gyro_sensor_rate_sample(gyro_instance, gyro);

#if AP_SIM_INS_FILE_ENABLED
        if (sitl->gyro_file_rw == SITL::SIM::INSFileMode::INS_FILE_WRITE) {
            write_gyro_to_file(gyro);
        }
#endif
    }
    gyro_accum /= nsamples;

    gyro_accum.rotate(sitl->imu_orientation);

    _rotate_and_correct_gyro(gyro_instance, gyro_accum);
    _notify_new_gyro_raw_sample(gyro_instance, gyro_accum, AP_HAL::micros64());
}

void AP_InertialSensor_SITL::timer_update(void)
{
    // on some simulations (RealFlight) the aircraft sim decides when a new sample is available.
    if (sitl->state.flightaxis_imu_frame_num > 0) {
        update_from_frame();
        return;
    }

    uint64_t now = AP_HAL::micros64();
#if 0
    // insert a 1s pause in IMU data. This triggers a pause in EK2
    // processing that leads to some interesting issues
    if (now > 5e6 && now < 6e6) {
        return;
    }
#endif
    if (sitl == nullptr) {
        return;
    }

    if (now >= next_accel_sample) {
        if (((1U << accel_instance) & sitl->accel_fail_mask) == 0) {
#if AP_SIM_INS_FILE_ENABLED
            if (sitl->accel_file_rw == SITL::SIM::INSFileMode::INS_FILE_READ
                || sitl->accel_file_rw == SITL::SIM::INSFileMode::INS_FILE_READ_STOP_ON_EOF) {
                read_accel_from_file();
            } else
#endif
            generate_accel();
            if (next_accel_sample == 0) {
                next_accel_sample = now + 1000000UL / accel_sample_hz;
            } else {
                while (now >= next_accel_sample) {
                    next_accel_sample += 1000000UL / accel_sample_hz;
                }
            }
        }
    }
    if (now >= next_gyro_sample) {
        if (((1U << gyro_instance) & sitl->gyro_fail_mask) == 0) {
#if AP_SIM_INS_FILE_ENABLED
            if (sitl->gyro_file_rw == SITL::SIM::INSFileMode::INS_FILE_READ
                || sitl->gyro_file_rw == SITL::SIM::INSFileMode::INS_FILE_READ_STOP_ON_EOF) {
                read_gyro_from_file();
            } else
#endif
            generate_gyro();

            if (next_gyro_sample == 0) {
                next_gyro_sample = now + 1000000UL / gyro_sample_hz;
            } else {
                while (now >= next_gyro_sample) {
                    next_gyro_sample += 1000000UL / gyro_sample_hz;
                }
            }
        }
    }
}

void AP_InertialSensor_SITL::update_from_frame(void)
{
    if (sitl == nullptr) {
        return;
    }

    if (flightaxis_imu_frame_num == sitl->state.flightaxis_imu_frame_num) {
        return;
    }

    flightaxis_imu_frame_num = sitl->state.flightaxis_imu_frame_num;

    if (((1U << accel_instance) & sitl->accel_fail_mask) == 0) {
        generate_accel();
    }
    if (((1U << gyro_instance) & sitl->gyro_fail_mask) == 0) {
        generate_gyro();
    }
}

float AP_InertialSensor_SITL::gyro_drift(void) const
{
    if (is_zero(sitl->drift_speed) ||
        is_zero(sitl->drift_time)) {
        return 0;
    }
    double period  = sitl->drift_time * 2;
    double minutes = fmod(AP_HAL::micros64() / 60.0e6, period);
    if (minutes < period/2) {
        return minutes * radians(sitl->drift_speed);
    }
    return (period - minutes) * radians(sitl->drift_speed);
}


bool AP_InertialSensor_SITL::update(void) 
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

uint8_t AP_InertialSensor_SITL::bus_id = 0;

void AP_InertialSensor_SITL::start()
{
    if (!_imu.register_gyro(gyro_instance, gyro_sample_hz,
                            AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SITL, bus_id, 1, DEVTYPE_SITL)) ||
        !_imu.register_accel(accel_instance, accel_sample_hz,
                             AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SITL, bus_id, 2, DEVTYPE_SITL))) {
        return;
    }
    bus_id++;
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_SITL::timer_update, void));

#if AP_SIM_INS_FILE_ENABLED
    if (sitl->accel_file_rw == SITL::SIM::INSFileMode::INS_FILE_READ
        || sitl->accel_file_rw == SITL::SIM::INSFileMode::INS_FILE_READ_STOP_ON_EOF) {
        hal.console->printf("Reading accel data from file for IMU[%u]\n", accel_instance);
    }
    if (sitl->gyro_file_rw == SITL::SIM::INSFileMode::INS_FILE_READ
        || sitl->gyro_file_rw == SITL::SIM::INSFileMode::INS_FILE_READ) {
        hal.console->printf("Reading gyro data from file for IMU[%u]\n", gyro_instance);
    }
#endif
}

/*
  temporary method to use file as GPS data
 */
#if AP_SIM_INS_FILE_ENABLED
void AP_InertialSensor_SITL::read_gyro_from_file()
{
    if (gyro_fd == -1) {
        char namebuf[32];
        snprintf(namebuf, sizeof(namebuf), "/tmp/gyro%d.dat", gyro_instance);
        gyro_fd = AP::FS().open(namebuf, O_RDONLY);
        if (gyro_fd == -1) {
            AP_HAL::panic("gyro data file %s not found", namebuf);
        }
    }

    float buf[8 * 3 * sizeof(float)];

    uint8_t nsamples = enable_fast_sampling(gyro_instance) ? 8 : 1;
    ssize_t ret = ::read(gyro_fd, buf, nsamples * 3 * sizeof(float));
    if (ret == (ssize_t)(nsamples * 3 * sizeof(float))) {
        read_gyro(buf, nsamples);
    }

    if (ret <= 0) {
        if (sitl->gyro_file_rw == SITL::SIM::INSFileMode::INS_FILE_READ_STOP_ON_EOF) {
#if HAL_LOGGING_ENABLED
            //stop logging
            if (AP_Logger::get_singleton()) {
                AP::logger().StopLogging();
            }
#endif
            exit(0);
        }
        lseek(gyro_fd, 0, SEEK_SET);
    }
}

void AP_InertialSensor_SITL::read_gyro(const float* buf, uint8_t nsamples)
{
    Vector3f gyro_accum;

    for (uint8_t j = 0; j < nsamples; j++) {
        float p = buf[j*3];
        float q = buf[j*3+1];
        float r = buf[j*3+2];

        Vector3f gyro = Vector3f(p, q, r);

        _notify_new_gyro_sensor_rate_sample(gyro_instance, gyro);

        gyro_accum += gyro;
    }
    gyro_accum /= nsamples;
    _rotate_and_correct_gyro(gyro_instance, gyro_accum);
    _notify_new_gyro_raw_sample(gyro_instance, gyro_accum, AP_HAL::micros64());
}

void AP_InertialSensor_SITL::write_gyro_to_file(const Vector3f& gyro)
{
    if (gyro_fd == -1) {
        char namebuf[32];
        snprintf(namebuf, sizeof(namebuf), "/tmp/gyro%d.dat", gyro_instance);
        gyro_fd = open(namebuf, O_WRONLY|O_TRUNC|O_CREAT, S_IRWXU|S_IRGRP|S_IROTH);
    }

    float buf[] { gyro.x, gyro.y, gyro.z };

    if (::write(gyro_fd, (void*)buf, sizeof(float) * 3) < 0) {
        ::printf("Could not write to file\n");
    }
}

void AP_InertialSensor_SITL::read_accel_from_file()
{
    if (accel_fd == -1) {
        char namebuf[32];
        snprintf(namebuf, sizeof(namebuf), "/tmp/accel%d.dat", accel_instance);
        accel_fd = open(namebuf, O_RDONLY|O_CLOEXEC);
        if (accel_fd == -1) {
            AP_HAL::panic("accel data file %s not found", namebuf);
        }
    }

    float buf[4*3*sizeof(float)];

    uint8_t nsamples = enable_fast_sampling(accel_instance) ? 4 : 1;
    ssize_t ret = ::read(accel_fd, buf, nsamples * 3 * sizeof(float));
    if (ret == (ssize_t)(nsamples * 3 * sizeof(float))) {
        read_accel(buf, nsamples);
    }

    if (ret <= 0) {
        if (sitl->accel_file_rw == SITL::SIM::INSFileMode::INS_FILE_READ_STOP_ON_EOF) {
#if HAL_LOGGING_ENABLED
            //stop logging
            if (AP_Logger::get_singleton()) {
                AP::logger().StopLogging();
            }
#endif
            exit(0);
        }
        lseek(accel_fd, 0, SEEK_SET);
    }
}

void AP_InertialSensor_SITL::read_accel(const float* buf, uint8_t nsamples)
{
    Vector3f accel_accum;

    for (uint8_t j = 0; j < nsamples; j++) {
        float p = buf[j*3];
        float q = buf[j*3+1];
        float r = buf[j*3+2];

        Vector3f accel = Vector3f(p, q, r);

        _notify_new_accel_sensor_rate_sample(accel_instance, accel);

        accel_accum += accel;
    }

    accel_accum /= nsamples;
    _rotate_and_correct_accel(accel_instance, accel_accum);
    _notify_new_accel_raw_sample(accel_instance, accel_accum, AP_HAL::micros64());

    _publish_temperature(accel_instance, get_temperature());
}

void AP_InertialSensor_SITL::write_accel_to_file(const Vector3f& accel)
{

    if (accel_fd == -1) {
        char namebuf[32];
        snprintf(namebuf, sizeof(namebuf), "/tmp/accel%d.dat", accel_instance);
        accel_fd = open(namebuf, O_WRONLY|O_TRUNC|O_CREAT, S_IRWXU|S_IRGRP|S_IROTH);
    }

    float buf[] { accel.x, accel.y, accel.z };

    if (::write(accel_fd, (void*)buf, sizeof(float) * 3) < 0) {
        ::printf("Could not write to file\n");
    }
}

#endif  // AP_SIM_INS_FILE_ENABLED

#endif // AP_SIM_INS_ENABLED
