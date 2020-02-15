#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor_SITL.h"
#include <SITL/SITL.h>
#include <stdio.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

const extern AP_HAL::HAL& hal;

AP_InertialSensor_SITL::AP_InertialSensor_SITL(AP_InertialSensor &imu) :
    AP_InertialSensor_Backend(imu)
{
}

/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_SITL::detect(AP_InertialSensor &_imu)
{
    AP_InertialSensor_SITL *sensor = new AP_InertialSensor_SITL(_imu);
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

    // grab the used instances
    for (uint8_t i=0; i<INS_SITL_INSTANCES; i++) {

        gyro_instance[i] = _imu.register_gyro(gyro_sample_hz[i],
                                              AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SITL, i, 1, DEVTYPE_SITL));
        accel_instance[i] = _imu.register_accel(accel_sample_hz[i],
                                              AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SITL, i, 2, DEVTYPE_SITL));
        if (enable_fast_sampling(accel_instance[i])) {
            _set_accel_raw_sample_rate(accel_instance[i], accel_sample_hz[i]*4);
        }
        if (enable_fast_sampling(gyro_instance[i])) {
            _set_gyro_raw_sample_rate(gyro_instance[i], gyro_sample_hz[i]*8);
        }
    }

    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_SITL::timer_update, void));

    return true;
}

// calculate a noisy noise component
static float calculate_noise(float noise, float noise_variation) {
    return noise * (1.0f + noise_variation * rand_float());
}

/*
  generate an accelerometer sample
 */
void AP_InertialSensor_SITL::generate_accel(uint8_t instance)
{
    Vector3f accel_accum;
    uint8_t nsamples = enable_fast_sampling(accel_instance[instance]) ? 4 : 1;
    for (uint8_t j = 0; j < nsamples; j++) {

        // add accel bias and noise
        Vector3f accel_bias = instance == 0 ? sitl->accel_bias.get() : sitl->accel2_bias.get();
        float xAccel = sitl->state.xAccel + accel_bias.x;
        float yAccel = sitl->state.yAccel + accel_bias.y;
        float zAccel = sitl->state.zAccel + accel_bias.z;

        // minimum noise levels are 2 bits, but averaged over many
        // samples, giving around 0.01 m/s/s
        float accel_noise = 0.01f;
        float noise_variation = 0.05;
        // this smears the individual motor peaks somewhat emulating physical motors
        float freq_variation = 0.12;

        xAccel += accel_noise * rand_float();
        yAccel += accel_noise * rand_float();
        zAccel += accel_noise * rand_float();

        bool motors_on = sitl->throttle > sitl->ins_noise_throttle_min;
        // on a real 180mm copter gyro noise varies between 0.8-4 m/s/s for throttle 0.2-0.8
        // giving a accel noise variation of 5.33 m/s/s over the full throttle range
        if (motors_on) {
            // add extra noise when the motors are on
            accel_noise = (instance == 0 ? sitl->accel_noise : sitl->accel2_noise) * sitl->throttle;
        }

        // VIB_FREQ is a static vibration applied to each axis
        const Vector3f &vibe_freq = sitl->vibe_freq;
        if (!vibe_freq.is_zero() && motors_on) {
            float t = AP_HAL::micros() * 1.0e-6f;
            xAccel += sinf(t * 2 * M_PI * vibe_freq.x) * calculate_noise(accel_noise, noise_variation);
            yAccel += sinf(t * 2 * M_PI * vibe_freq.y) * calculate_noise(accel_noise, noise_variation);
            zAccel += sinf(t * 2 * M_PI * vibe_freq.z) * calculate_noise(accel_noise, noise_variation);
        }

        // VIB_MOT_MAX is a rpm-scaled vibration applied to each axis
        if (!is_zero(sitl->vibe_motor) && motors_on) {
            for (uint8_t i = 0; i < sitl->state.num_motors; i++) {
                float &phase = accel_motor_phase[instance][i];
                float motor_freq = calculate_noise(sitl->state.rpm[i] / 60.0f, freq_variation);
                float phase_incr = motor_freq * 2 * M_PI / (accel_sample_hz[instance] * nsamples);
                phase += phase_incr;
                if (phase_incr > M_PI) {
                    phase -= 2 * M_PI;
                }
                else if (phase_incr < -M_PI) {
                    phase += 2 * M_PI;
                }
                xAccel += sinf(phase) * calculate_noise(accel_noise, noise_variation);
                yAccel += sinf(phase) * calculate_noise(accel_noise, noise_variation);
                zAccel += sinf(phase) * calculate_noise(accel_noise, noise_variation);
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
            xAccel += lever_arm_accel.x + centripetal_accel.x;
            yAccel += lever_arm_accel.y + centripetal_accel.y;
            zAccel += lever_arm_accel.z + centripetal_accel.z;
        }

        if (fabsf(sitl->accel_fail) > 1.0e-6f) {
            xAccel = sitl->accel_fail;
            yAccel = sitl->accel_fail;
            zAccel = sitl->accel_fail;
        }

        Vector3f accel = Vector3f(xAccel, yAccel, zAccel);

        _rotate_and_correct_accel(accel_instance[instance], accel);
        _notify_new_accel_sensor_rate_sample(instance, accel);

        accel_accum += accel;
    }

    accel_accum /= nsamples;
    _notify_new_accel_raw_sample(accel_instance[instance], accel_accum);

    _publish_temperature(instance, 23);
}

/*
  generate a gyro sample
 */
void AP_InertialSensor_SITL::generate_gyro(uint8_t instance)
{
    Vector3f gyro_accum;
    uint8_t nsamples = enable_fast_sampling(gyro_instance[instance]) ? 8 : 1;
    for (uint8_t j = 0; j < nsamples; j++) {
        float p = radians(sitl->state.rollRate) + gyro_drift();
        float q = radians(sitl->state.pitchRate) + gyro_drift();
        float r = radians(sitl->state.yawRate) + gyro_drift();

        // minimum gyro noise is less than 1 bit
        float gyro_noise = ToRad(0.04f);
        float noise_variation = 0.05f;
        // this smears the individual motor peaks somewhat emulating physical motors
        float freq_variation = 0.12f;

        p += gyro_noise * rand_float();
        q += gyro_noise * rand_float();
        r += gyro_noise * rand_float();

        bool motors_on = sitl->throttle > sitl->ins_noise_throttle_min;
        // on a real 180mm copter gyro noise varies between 0.2-0.4 rad/s for throttle 0.2-0.8
        // giving a gyro noise variation of 0.33 rad/s or 20deg/s over the full throttle range
        if (motors_on) {
            // add extra noise when the motors are on
            gyro_noise = ToRad(sitl->gyro_noise) * sitl->throttle;
        }

        // VIB_FREQ is a static vibration applied to each axis
        const Vector3f &vibe_freq = sitl->vibe_freq;
        if (!vibe_freq.is_zero() && motors_on) {
            float t = AP_HAL::micros() * 1.0e-6f;
            p += sinf(t * 2 * M_PI * vibe_freq.x) * calculate_noise(gyro_noise, noise_variation);
            q += sinf(t * 2 * M_PI * vibe_freq.y) * calculate_noise(gyro_noise, noise_variation);
            r += sinf(t * 2 * M_PI * vibe_freq.z) * calculate_noise(gyro_noise, noise_variation);
        }

        // VIB_MOT_MAX is a rpm-scaled vibration applied to each axis
        if (!is_zero(sitl->vibe_motor) && motors_on) {
            for (uint8_t i = 0; i < sitl->state.num_motors; i++) {
                float motor_freq = calculate_noise(sitl->state.rpm[i] / 60.0f, freq_variation);
                float phase_incr = motor_freq * 2 * M_PI / (gyro_sample_hz[instance] * nsamples);
                float &phase = gyro_motor_phase[instance][i];
                phase += phase_incr;
                if (phase_incr > M_PI) {
                    phase -= 2 * M_PI;
                }
                else if (phase_incr < -M_PI) {
                    phase += 2 * M_PI;
                }
                p += sinf(phase) * calculate_noise(gyro_noise, noise_variation);
                q += sinf(phase) * calculate_noise(gyro_noise, noise_variation);
                r += sinf(phase) * calculate_noise(gyro_noise, noise_variation);
            }
        }

        Vector3f gyro = Vector3f(p, q, r);

        // add in gyro scaling
        Vector3f scale = sitl->gyro_scale;
        gyro.x *= (1 + scale.x * 0.01f);
        gyro.y *= (1 + scale.y * 0.01f);
        gyro.z *= (1 + scale.z * 0.01f);

        _rotate_and_correct_gyro(gyro_instance[instance], gyro);
        gyro_accum += gyro;
        _notify_new_gyro_sensor_rate_sample(instance, gyro);
    }
    gyro_accum /= nsamples;
    _notify_new_gyro_raw_sample(gyro_instance[instance], gyro_accum);
}

void AP_InertialSensor_SITL::timer_update(void)
{
    uint64_t now = AP_HAL::micros64();
#if 0
    // insert a 1s pause in IMU data. This triggers a pause in EK2
    // processing that leads to some interesting issues
    if (now > 5e6 && now < 6e6) {
        return;
    }
#endif
    for (uint8_t i=0; i<INS_SITL_INSTANCES; i++) {
        if (now >= next_accel_sample[i]) {
            if (((1U<<i) & sitl->accel_fail_mask) == 0) {
                generate_accel(i);
                if (next_accel_sample[i] == 0) {
                    next_accel_sample[i] = now + 1000000UL / accel_sample_hz[i];
                }
                else {
                    while (now >= next_accel_sample[i]) {
                        next_accel_sample[i] += 1000000UL / accel_sample_hz[i];
                    }
                }
            }
        }
        if (now >= next_gyro_sample[i]) {
            if (((1U<<i) & sitl->gyro_fail_mask) == 0) {
                generate_gyro(i);
                if (next_gyro_sample[i] == 0) {
                    next_gyro_sample[i] = now + 1000000UL / gyro_sample_hz[i];
                }
                else {
                    while (now >= next_gyro_sample[i]) {
                        next_gyro_sample[i] += 1000000UL / gyro_sample_hz[i];
                    }
                }
            }
        }
    }
}

float AP_InertialSensor_SITL::gyro_drift(void)
{
    if (is_zero(sitl->drift_speed) ||
        is_zero(sitl->drift_time)) {
        return 0;
    }
    double period  = sitl->drift_time * 2;
    double minutes = fmod(AP_HAL::micros64() / 60.0e6, period);
    if (minutes < period/2) {
        return minutes * ToRad(sitl->drift_speed);
    }
    return (period - minutes) * ToRad(sitl->drift_speed);
}


bool AP_InertialSensor_SITL::update(void) 
{
    for (uint8_t i=0; i<INS_SITL_INSTANCES; i++) {
        update_accel(accel_instance[i]);
        update_gyro(gyro_instance[i]);
    }
    return true;
}

#endif // HAL_BOARD_SITL
