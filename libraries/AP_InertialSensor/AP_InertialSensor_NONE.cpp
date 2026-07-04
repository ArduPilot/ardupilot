#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor_NONE.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32 || AP_INERTIALSENSOR_ALLOW_NO_SENSORS

const extern AP_HAL::HAL& hal;

AP_InertialSensor_NONE::AP_InertialSensor_NONE(AP_InertialSensor &imu, const uint16_t sample_rates[]) :
    AP_InertialSensor_Backend(imu),
    gyro_sample_hz(sample_rates[0]),
    accel_sample_hz(sample_rates[1]),
    gyro_instance(0),
    accel_instance(0),
    next_gyro_sample(0),
    next_accel_sample(0),
    gyro_time(0),
    accel_time(0),
    gyro_motor_phase{},
    accel_motor_phase{}
{
}

AP_InertialSensor_Backend *AP_InertialSensor_NONE::detect(AP_InertialSensor &_imu, const uint16_t sample_rates[])
{
    AP_InertialSensor_NONE *sensor = NEW_NOTHROW AP_InertialSensor_NONE(_imu, sample_rates);
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
    // no-op: samples are generated from the timer callback
}

void AP_InertialSensor_NONE::generate_accel()
{
    Vector3f accel(0.0f, 0.0f, -GRAVITY_MSS);
    _notify_new_accel_sensor_rate_sample(accel_instance, accel);
    _rotate_and_correct_accel(accel_instance, accel);
    _notify_new_accel_raw_sample(accel_instance, accel, AP_HAL::micros64());
    _publish_temperature(accel_instance, 25.0f);
}

void AP_InertialSensor_NONE::generate_gyro()
{
    Vector3f gyro(0.0f, 0.0f, 0.0f);
    _notify_new_gyro_sensor_rate_sample(gyro_instance, gyro);
    _rotate_and_correct_gyro(gyro_instance, gyro);
    _notify_new_gyro_raw_sample(gyro_instance, gyro, AP_HAL::micros64());
}

void AP_InertialSensor_NONE::timer_update(void)
{
    const uint64_t now = AP_HAL::micros64();

    if (next_accel_sample == 0 || now >= next_accel_sample) {
        generate_accel();
        next_accel_sample = now + 1000000ULL / accel_sample_hz;
    }

    if (next_gyro_sample == 0 || now >= next_gyro_sample) {
        generate_gyro();
        next_gyro_sample = now + 1000000ULL / gyro_sample_hz;
    }
}

float AP_InertialSensor_NONE::gyro_drift(void)
{
    return 0.0f;
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

#endif // CONFIG_HAL_BOARD == HAL_BOARD_ESP32 || AP_INERTIALSENSOR_ALLOW_NO_SENSORS
