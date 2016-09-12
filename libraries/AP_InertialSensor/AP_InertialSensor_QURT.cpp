/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor_QURT.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_QURT

const extern AP_HAL::HAL& hal;

ObjectBuffer<mpu9x50_data> *mpu9250_mag_buffer = nullptr;

AP_InertialSensor_QURT::AP_InertialSensor_QURT(AP_InertialSensor &imu) :
    AP_InertialSensor_Backend(imu)
{
}

/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_QURT::detect(AP_InertialSensor &_imu)
{
    AP_InertialSensor_QURT *sensor = new AP_InertialSensor_QURT(_imu);
    if (sensor == NULL) {
        return NULL;
    }
    if (!sensor->init_sensor()) {
        delete sensor;
        return NULL;
    }
    return sensor;
}

bool AP_InertialSensor_QURT::init_sensor(void) 
{
    gyro_instance = _imu.register_gyro(1000);
    accel_instance = _imu.register_accel(1000);

    mpu9250_mag_buffer = new ObjectBuffer<mpu9x50_data>(20);
    init_mpu9250();

    _product_id = AP_PRODUCT_ID_MPU9250;

    return true;
}

/*
  handle data ready interrupt from mpu9250
 */
extern "C" {
    static void *mpu_data_ready_trampoline(void *ctx);
}
static void *mpu_data_ready_trampoline(void *ctx)
{
    ((AP_InertialSensor_QURT *)ctx)->data_ready();
    return NULL;
}

void AP_InertialSensor_QURT::init_mpu9250(void) 
{
    struct mpu9x50_config config;
    
    config.gyro_lpf = MPU9X50_GYRO_LPF_184HZ;
    config.acc_lpf  = MPU9X50_ACC_LPF_184HZ;
    config.gyro_fsr = MPU9X50_GYRO_FSR_2000DPS;
    config.acc_fsr  = MPU9X50_ACC_FSR_16G;
    config.gyro_sample_rate = MPU9x50_SAMPLE_RATE_1000HZ;
    config.compass_enabled = true;
    config.compass_sample_rate = MPU9x50_COMPASS_SAMPLE_RATE_100HZ;
    config.spi_dev_path = "/dev/spi-1";
        
    int ret;
    ret = mpu9x50_validate_configuration(&config);
    if (ret != 0) {
        AP_HAL::panic("Bad MPU9x50 configuration");
    }
    
    ret = mpu9x50_initialize(&config);
    if (ret != 0) {
        AP_HAL::panic("Failed to initialise mpu9250");
    }

    mpu9x50_register_interrupt(65, mpu_data_ready_trampoline, this);
    HAP_PRINTF("Opened MPU9X50");
}


void AP_InertialSensor_QURT::data_ready(void)
{
    uint64_t now = AP_HAL::micros64();
    struct mpu9x50_data data;
    int ret = mpu9x50_get_data(&data);
    if (ret == 0) {
        data.timestamp = now;
        buf.push(data);
        if (data.mag_data_ready) {
            mpu9250_mag_buffer->push(data);
        }
    }
}

void AP_InertialSensor_QURT::accumulate(void) 
{
    const float ACCEL_SCALE_1G = GRAVITY_MSS / 2048.0;
    const float GYRO_SCALE = 0.0174532 / 16.4;

    struct mpu9x50_data data;
    
    while (buf.pop(data)) {
        Vector3f accel(data.accel_raw[0]*ACCEL_SCALE_1G,
                       data.accel_raw[1]*ACCEL_SCALE_1G,
                       data.accel_raw[2]*ACCEL_SCALE_1G);
        Vector3f gyro(data.gyro_raw[0]*GYRO_SCALE,
                      data.gyro_raw[1]*GYRO_SCALE,
                      data.gyro_raw[2]*GYRO_SCALE);

        _rotate_and_correct_accel(accel_instance, accel);
        _rotate_and_correct_gyro(gyro_instance, gyro);
    
        _notify_new_gyro_raw_sample(gyro_instance, gyro, data.timestamp);
        _notify_new_accel_raw_sample(accel_instance, accel, data.timestamp);
    }
}

bool AP_InertialSensor_QURT::update(void) 
{
    accumulate();
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

#endif // HAL_BOARD_QURT
