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

/*
  generate an accelerometer sample
 */
void AP_InertialSensor_SITL::generate_accel(uint8_t instance)
{
    // minimum noise levels are 2 bits, but averaged over many
    // samples, giving around 0.01 m/s/s
    float accel_noise = 0.01f;

    if (sitl->motors_on) {
        // add extra noise when the motors are on
        accel_noise += instance==0?sitl->accel_noise:sitl->accel2_noise;
    }

    // add accel bias and noise
    Vector3f accel_bias = instance==0?sitl->accel_bias.get():sitl->accel2_bias.get();
    float xAccel = sitl->state.xAccel + accel_bias.x;
    float yAccel = sitl->state.yAccel + accel_bias.y;
    float zAccel = sitl->state.zAccel + accel_bias.z;
    const Vector3f &vibe_freq = sitl->vibe_freq;
    if (vibe_freq.is_zero()) {
        xAccel += accel_noise * rand_float();
        yAccel += accel_noise * rand_float();
        zAccel += accel_noise * rand_float();
    } else {
        float t = AP_HAL::micros() * 1.0e-6f;
        xAccel += sinf(t * 2 * M_PI * vibe_freq.x) * accel_noise;
        yAccel += sinf(t * 2 * M_PI * vibe_freq.y) * accel_noise;
        zAccel += sinf(t * 2 * M_PI * vibe_freq.z) * accel_noise;
    }
    
    // correct for the acceleration due to the IMU position offset and angular acceleration
    // correct for the centripetal acceleration
    // only apply corrections to first accelerometer
    Vector3f pos_offset = sitl->imu_pos_offset;
    if (!pos_offset.is_zero()) {
        // calculate sensed acceleration due to lever arm effect
        // Note: the % operator has been overloaded to provide a cross product
        Vector3f angular_accel = Vector3f(radians(sitl->state.angAccel.x) , radians(sitl->state.angAccel.y) , radians(sitl->state.angAccel.z));
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

    uint8_t nsamples = enable_fast_sampling(accel_instance[instance])?4:1;
    for (uint8_t i=0; i<nsamples; i++) {
        _notify_new_accel_raw_sample(accel_instance[instance], accel);
    }

    _publish_temperature(instance, 23);
}

/*
  generate a gyro sample
 */
void AP_InertialSensor_SITL::generate_gyro(uint8_t instance)
{
    // minimum gyro noise is less than 1 bit
    float gyro_noise = ToRad(0.04f);
    
    if (sitl->motors_on) {
        // add extra noise when the motors are on
        gyro_noise += ToRad(sitl->gyro_noise);
    }

    float p = radians(sitl->state.rollRate) + gyro_drift();
    float q = radians(sitl->state.pitchRate) + gyro_drift();
    float r = radians(sitl->state.yawRate) + gyro_drift();

    const Vector3f &vibe_freq = sitl->vibe_freq;
    if (vibe_freq.is_zero()) {
        p += gyro_noise * rand_float();
        q += gyro_noise * rand_float();
        r += gyro_noise * rand_float();
    } else {
        float t = AP_HAL::micros() * 1.0e-6f;
        p += sinf(t * 2 * M_PI * vibe_freq.x) * gyro_noise;
        q += sinf(t * 2 * M_PI * vibe_freq.y) * gyro_noise;
        r += sinf(t * 2 * M_PI * vibe_freq.z) * gyro_noise;
    }

    Vector3f gyro = Vector3f(p, q, r);

    // add in gyro scaling
    Vector3f scale = sitl->gyro_scale;
    gyro.x *= (1 + scale.x*0.01f);
    gyro.y *= (1 + scale.y*0.01f);
    gyro.z *= (1 + scale.z*0.01f);

    _rotate_and_correct_gyro(gyro_instance[instance], gyro);
    
    uint8_t nsamples = enable_fast_sampling(gyro_instance[instance])?8:1;
    for (uint8_t i=0; i<nsamples; i++) {
        _notify_new_gyro_raw_sample(gyro_instance[instance], gyro);
    }
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
                while (now >= next_accel_sample[i]) {
                    next_accel_sample[i] += 1000000UL / accel_sample_hz[i];
                }
            }
        }
        if (now >= next_gyro_sample[i]) {
            if (((1U<<i) & sitl->gyro_fail_mask) == 0) {
                generate_gyro(i);
                while (now >= next_gyro_sample[i]) {
                    next_gyro_sample[i] += 1000000UL / gyro_sample_hz[i];
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
