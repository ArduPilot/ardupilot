#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor_NONE.h"
#include <SITL/SITL.h>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32


static float sim_rand_float(void)
{
    return ((((unsigned)random()) % 2000000) - 1.0e6) / 1.0e6;
}

const extern AP_HAL::HAL& hal;

AP_InertialSensor_NONE::AP_InertialSensor_NONE(AP_InertialSensor &imu, const uint16_t sample_rates[]) :
    AP_InertialSensor_Backend(imu),
    gyro_sample_hz(sample_rates[0]),
    accel_sample_hz(sample_rates[1])
{
}

/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_NONE::detect(AP_InertialSensor &_imu, const uint16_t sample_rates[])
{
    AP_InertialSensor_NONE *sensor = new AP_InertialSensor_NONE(_imu, sample_rates);
    if (sensor == nullptr) {

        return nullptr;
    }
    if (!sensor->init_sensor()) {
        delete sensor;

        return nullptr;
    }
    return sensor;
}


bool AP_InertialSensor_NONE::init_sensor(void)
{
  
    return true;
}



void AP_InertialSensor_NONE::accumulate()
{
    // nothing to do
}


// calculate a noisy noise component
static float calculate_noise(float noise, float noise_variation) {
    return noise * (1.0f + noise_variation * sim_rand_float());
}

/*
  generate an accelerometer sample
 */
void AP_InertialSensor_NONE::generate_accel()
{
    Vector3f accel_accum;
    uint8_t nsamples = enable_fast_sampling(accel_instance) ? 4 : 1;
    for (uint8_t j = 0; j < nsamples; j++) {

        // add accel bias and noise
        //Vector3f accel_bias = Vector3f{0.01,0.01,0.01}; 
        float xAccel = 0.01;
        float yAccel = 0.01;
        float zAccel = 0.01;

        // minimum noise levels are 2 bits, but averaged over many
        // samples, giving around 0.01 m/s/s
        float accel_noise = 0.01f;
        float noise_variation = 0.05f;
        // this smears the individual motor peaks somewhat emulating physical motors
        //float freq_variation = 0.12f;
        // add in sensor noise
        xAccel += accel_noise * sim_rand_float();
        yAccel += accel_noise * sim_rand_float();
        zAccel += accel_noise * sim_rand_float();

        bool motors_on = 1; 

        // on a real 180mm copter gyro noise varies between 0.8-4 m/s/s for throttle 0.2-0.8
        // giving a accel noise variation of 5.33 m/s/s over the full throttle range
        if (motors_on) {
            // add extra noise when the motors are on
            accel_noise = 0;
        }

        // VIB_FREQ is a static vibration applied to each axis
        const Vector3f &vibe_freq =  Vector3f{0.01,0.01,0.01};

        if (vibe_freq.is_zero()) {
            // no rpm noise, so add in background noise if any
            xAccel += accel_noise * sim_rand_float();
            yAccel += accel_noise * sim_rand_float();
            zAccel += accel_noise * sim_rand_float();
        }

        if (!vibe_freq.is_zero() && motors_on) {
            xAccel += sinf(accel_time * 2 * M_PI * vibe_freq.x) * calculate_noise(accel_noise, noise_variation);
            yAccel += sinf(accel_time * 2 * M_PI * vibe_freq.y) * calculate_noise(accel_noise, noise_variation);
            zAccel += sinf(accel_time * 2 * M_PI * vibe_freq.z) * calculate_noise(accel_noise, noise_variation);
            accel_time += 1.0f / (accel_sample_hz * nsamples);
        }

        // VIB_MOT_MAX is a rpm-scaled vibration applied to each axis
        if ( motors_on) {
            for (uint8_t i = 0; i < 4; i++) {
                float &phase = accel_motor_phase[i];
                float motor_freq = 50;
                float phase_incr = motor_freq * 2 * M_PI / (accel_sample_hz * nsamples);
                phase += phase_incr;
                if (phase_incr > M_PI) {
                    phase -= 2 * M_PI;
                }
                else if (phase_incr < -M_PI) {
                    phase += 2 * M_PI;
                }
                xAccel += sinf(phase) * calculate_noise(accel_noise * 0.01, noise_variation);
                yAccel += sinf(phase) * calculate_noise(accel_noise *0.01, noise_variation);
                zAccel += sinf(phase) * calculate_noise(accel_noise * 0.01, noise_variation);
            }
        }

        // correct for the acceleration due to the IMU position offset and angular acceleration
        // correct for the centripetal acceleration
        // only apply corrections to first accelerometer
        Vector3f pos_offset =  Vector3f{0.01,0.01,0.01};
        if (!pos_offset.is_zero()) {
            // calculate sensed acceleration due to lever arm effect
            // Note: the % operator has been overloaded to provide a cross product
            Vector3f angular_accel = Vector3f(radians(0.01), radians(0.01), radians(0.01));
            Vector3f lever_arm_accel = angular_accel % pos_offset;

            // calculate sensed acceleration due to centripetal acceleration
            Vector3f angular_rate = Vector3f(radians(0.01), radians(0.01), radians(0.01));
            Vector3f centripetal_accel = angular_rate % (angular_rate % pos_offset);

            // apply corrections
            xAccel += lever_arm_accel.x + centripetal_accel.x;
            yAccel += lever_arm_accel.y + centripetal_accel.y;
            zAccel += lever_arm_accel.z + centripetal_accel.z;
        }

        if (fabsf(xAccel) > 1.0e-6f) {
            xAccel = 0.01;
            yAccel = 0.01;
            zAccel = 0.01;
        }

        Vector3f accel = Vector3f(xAccel, yAccel, zAccel);

        _notify_new_accel_sensor_rate_sample(accel_instance, accel);

        accel_accum += accel;
    }

    accel_accum /= nsamples;
    _rotate_and_correct_accel(accel_instance, accel_accum);
    _notify_new_accel_raw_sample(accel_instance, accel_accum, AP_HAL::micros64());

    _publish_temperature(accel_instance, 23);
}

/*
  generate a gyro sample
 */
void AP_InertialSensor_NONE::generate_gyro()
{
    Vector3f gyro_accum;
    uint8_t nsamples = enable_fast_sampling(gyro_instance) ? 8 : 1;

    for (uint8_t j = 0; j < nsamples; j++) {
        float p = radians(0.01) + gyro_drift();
        float q = radians(0.01) + gyro_drift();
        float r = radians(0.01) + gyro_drift();

        // minimum gyro noise is less than 1 bit
        float gyro_noise = ToRad(0.04f);
        float noise_variation = 0.05f;
        // this smears the individual motor peaks somewhat emulating physical motors
        float freq_variation = 0.12f;
        // add in sensor noise
        p += gyro_noise * sim_rand_float();
        q += gyro_noise * sim_rand_float();
        r += gyro_noise * sim_rand_float();

        bool motors_on = 1;
        // on a real 180mm copter gyro noise varies between 0.2-0.4 rad/s for throttle 0.2-0.8
        // giving a gyro noise variation of 0.33 rad/s or 20deg/s over the full throttle range
        if (motors_on) {
            // add extra noise when the motors are on
            gyro_noise = ToRad(0.01) * 0.01;
        }

        // VIB_FREQ is a static vibration applied to each axis
        const Vector3f &vibe_freq = Vector3f{0.01,0.01,0.01};

        if ( vibe_freq.is_zero() ) {
            // no rpm noise, so add in background noise if any
            p += gyro_noise * sim_rand_float();
            q += gyro_noise * sim_rand_float();
            r += gyro_noise * sim_rand_float();
        }

        if (!vibe_freq.is_zero() && motors_on) {
            p += sinf(gyro_time * 2 * M_PI * vibe_freq.x) * calculate_noise(gyro_noise, noise_variation);
            q += sinf(gyro_time * 2 * M_PI * vibe_freq.y) * calculate_noise(gyro_noise, noise_variation);
            r += sinf(gyro_time * 2 * M_PI * vibe_freq.z) * calculate_noise(gyro_noise, noise_variation);
            gyro_time += 1.0f / (gyro_sample_hz * nsamples);
        }

        // VIB_MOT_MAX is a rpm-scaled vibration applied to each axis
        if ( motors_on) {
            for (uint8_t i = 0; i < 4; i++) {
                float motor_freq = calculate_noise(0.01 / 60.0f, freq_variation);
                float phase_incr = motor_freq * 2 * M_PI / (gyro_sample_hz * nsamples);
                float &phase = gyro_motor_phase[i];
                phase += phase_incr;
                if (phase_incr > M_PI) {
                    phase -= 2 * M_PI;
                }
                else if (phase_incr < -M_PI) {
                    phase += 2 * M_PI;
                }
                p += sinf(phase) * calculate_noise(gyro_noise * 0.01, noise_variation);
                q += sinf(phase) * calculate_noise(gyro_noise * 0.01, noise_variation);
                r += sinf(phase) * calculate_noise(gyro_noise * 0.01, noise_variation);
            }
        }

        Vector3f gyro = Vector3f(p, q, r);

        // add in gyro scaling
        Vector3f scale = Vector3f{0.01,0.01,0.01};
        gyro.x *= (1 + scale.x * 0.01f);
        gyro.y *= (1 + scale.y * 0.01f);
        gyro.z *= (1 + scale.z * 0.01f);

        gyro_accum += gyro;
        _notify_new_gyro_sensor_rate_sample(gyro_instance, gyro);
    }
    gyro_accum /= nsamples;
    _rotate_and_correct_gyro(gyro_instance, gyro_accum);
    _notify_new_gyro_raw_sample(gyro_instance, gyro_accum, AP_HAL::micros64());
}

void AP_InertialSensor_NONE::timer_update(void)
{
    uint64_t now = AP_HAL::micros64();

    if (now >= next_accel_sample) {
        {
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
        {
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

float AP_InertialSensor_NONE::gyro_drift(void)
{
 
    double period  = 0.01 * 2;
    double minutes = fmod(AP_HAL::micros64() / 60.0e6, period);
    if (minutes < period/2) {
        return minutes * ToRad(0.01);
    }
    return (period - minutes) * ToRad(0.01);
}


bool AP_InertialSensor_NONE::update(void) 
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

uint8_t AP_InertialSensor_NONE::bus_id = 0;

void AP_InertialSensor_NONE::start()
{
    if (!_imu.register_gyro(gyro_instance, gyro_sample_hz,
                            AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SITL, bus_id, 1, DEVTYPE_SITL)) ||
        !_imu.register_accel(accel_instance, accel_sample_hz,
                             AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SITL, bus_id, 2, DEVTYPE_SITL))) {
        return;
    }
    bus_id++;
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_NONE::timer_update, void));

}

#endif // HAL_BOARD_NONE
