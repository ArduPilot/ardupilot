#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor_SITL.h"
#include <SITL/SITL.h>

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
        gyro_instance[i] = _imu.register_gyro(sitl->update_rate_hz, i);
        accel_instance[i] = _imu.register_accel(sitl->update_rate_hz, i);
    }

    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_SITL::timer_update, void));

    return true;
}

void AP_InertialSensor_SITL::timer_update(void)
{
    // minimum noise levels are 2 bits, but averaged over many
    // samples, giving around 0.01 m/s/s
    float accel_noise = 0.01f;
    float accel2_noise = 0.01f;

    // minimum gyro noise is also less than 1 bit
    float gyro_noise = ToRad(0.04f);
    if (sitl->motors_on) {
        // add extra noise when the motors are on
        accel_noise += sitl->accel_noise;
        accel2_noise += sitl->accel2_noise;
        gyro_noise += ToRad(sitl->gyro_noise);
    }

    // add accel bias and noise
    Vector3f accel_bias = sitl->accel_bias.get();
    float xAccel1 = sitl->state.xAccel + accel_noise * rand_float() + accel_bias.x;
    float yAccel1 = sitl->state.yAccel + accel_noise * rand_float() + accel_bias.y;
    float zAccel1 = sitl->state.zAccel + accel_noise * rand_float() + accel_bias.z;

    accel_bias = sitl->accel2_bias.get();
    float xAccel2 = sitl->state.xAccel + accel2_noise * rand_float() + accel_bias.x;
    float yAccel2 = sitl->state.yAccel + accel2_noise * rand_float() + accel_bias.y;
    float zAccel2 = sitl->state.zAccel + accel2_noise * rand_float() + accel_bias.z;

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
        xAccel1 += lever_arm_accel.x + centripetal_accel.x;
        yAccel1 += lever_arm_accel.y + centripetal_accel.y;
        zAccel1 += lever_arm_accel.z + centripetal_accel.z;
    }

    if (fabsf(sitl->accel_fail) > 1.0e-6f) {
        xAccel1 = sitl->accel_fail;
        yAccel1 = sitl->accel_fail;
        zAccel1 = sitl->accel_fail;
    }

    Vector3f accel0 = Vector3f(xAccel1, yAccel1, zAccel1) + _imu.get_accel_offsets(0);
    Vector3f accel1 = Vector3f(xAccel2, yAccel2, zAccel2) + _imu.get_accel_offsets(1);
    _notify_new_accel_raw_sample(accel_instance[0], accel0);
    _notify_new_accel_raw_sample(accel_instance[1], accel1);

    float p = radians(sitl->state.rollRate) + gyro_drift();
    float q = radians(sitl->state.pitchRate) + gyro_drift();
    float r = radians(sitl->state.yawRate) + gyro_drift();

    float p1 = p + gyro_noise * rand_float();
    float q1 = q + gyro_noise * rand_float();
    float r1 = r + gyro_noise * rand_float();

    float p2 = p + gyro_noise * rand_float();
    float q2 = q + gyro_noise * rand_float();
    float r2 = r + gyro_noise * rand_float();

    Vector3f gyro0 = Vector3f(p1, q1, r1) + _imu.get_gyro_offsets(0);
    Vector3f gyro1 = Vector3f(p2, q2, r2) + _imu.get_gyro_offsets(1);

    // add in gyro scaling
    Vector3f scale = sitl->gyro_scale;
    gyro0.x *= (1 + scale.x*0.01);
    gyro0.y *= (1 + scale.y*0.01);
    gyro0.z *= (1 + scale.z*0.01);

    gyro1.x *= (1 + scale.x*0.01);
    gyro1.y *= (1 + scale.y*0.01);
    gyro1.z *= (1 + scale.z*0.01);
    
    _notify_new_gyro_raw_sample(gyro_instance[0], gyro0);
    _notify_new_gyro_raw_sample(gyro_instance[1], gyro1);
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
