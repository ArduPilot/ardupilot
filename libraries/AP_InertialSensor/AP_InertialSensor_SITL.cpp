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
    sitl = (SITL::SITL *)AP_Param::find_object("SIM_");
    if (sitl == nullptr) {
        return false;
    }

    // grab the used instances
    for (uint8_t i=0; i<INS_SITL_INSTANCES; i++) {
        gyro_instance[i] = _imu.register_gyro(gyro_sample_hz[i], i);
        accel_instance[i] = _imu.register_accel(accel_sample_hz[i], i);
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
    float xAccel = sitl->state.xAccel + accel_noise * rand_float() + accel_bias.x;
    float yAccel = sitl->state.yAccel + accel_noise * rand_float() + accel_bias.y;
    float zAccel = sitl->state.zAccel + accel_noise * rand_float() + accel_bias.z;

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

    Vector3f accel = Vector3f(xAccel, yAccel, zAccel) + _imu.get_accel_offsets(instance);

    _notify_new_accel_raw_sample(accel_instance[instance], accel, AP_HAL::micros64());
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

    p += gyro_noise * rand_float();
    q += gyro_noise * rand_float();
    r += gyro_noise * rand_float();

    Vector3f gyro = Vector3f(p, q, r) + _imu.get_gyro_offsets(instance);

    // add in gyro scaling
    Vector3f scale = sitl->gyro_scale;
    gyro.x *= (1 + scale.x*0.01);
    gyro.y *= (1 + scale.y*0.01);
    gyro.z *= (1 + scale.z*0.01);

    _notify_new_gyro_raw_sample(gyro_instance[instance], gyro, AP_HAL::micros64());
}

void AP_InertialSensor_SITL::timer_update(void)
{
    uint64_t now = AP_HAL::micros64();
    for (uint8_t i=0; i<INS_SITL_INSTANCES; i++) {
        if (now >= next_accel_sample[i]) {
            generate_accel(i);
            while (now >= next_accel_sample[i]) {
                next_accel_sample[i] += 1000000UL / accel_sample_hz[i];
            }
        }
        if (now >= next_gyro_sample[i]) {
            generate_gyro(i);
            while (now >= next_gyro_sample[i]) {
                next_gyro_sample[i] += 1000000UL / gyro_sample_hz[i];
            }
        }
    }
}

// generate a random float between -1 and 1
float AP_InertialSensor_SITL::rand_float(void)
{
    return ((((unsigned)random()) % 2000000) - 1.0e6) / 1.0e6;
}

float AP_InertialSensor_SITL::gyro_drift(void)
{
    if (sitl->drift_speed == 0.0f ||
        sitl->drift_time == 0.0f) {
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
