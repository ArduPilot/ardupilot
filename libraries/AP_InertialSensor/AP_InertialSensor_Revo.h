#pragma once
/*
    copied from AP_InertialSensor_Invensense
    
  driver for the invensense range of IMUs, including:

  MPU6000
  MPU9250
  ICM-20608
 */

#include <stdint.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/SPIDevice.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_Math/AP_Math.h>
#include <Filter/Filter.h>
#include <Filter/LowPassFilter.h>
#include <Filter/LowPassFilter2p.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"
#include "AuxiliaryBus.h"

typedef struct MPU_Item {
    uint64_t time;
    uint16_t ax;
    uint16_t ay;
    uint16_t az;
    uint16_t temp;
    uint16_t gx;
    uint16_t gy;
    uint16_t gz;
} mpu_item;
    
    
typedef struct {
    uint32_t t;
    uint16_t read_ptr;
    uint16_t write_ptr;
} mpu_log_item;

class AP_Invensense_AuxiliaryBus;

class AP_InertialSensor_Revo : public AP_InertialSensor_Backend
{
    friend AP_Invensense_AuxiliaryBus;

public:
    virtual ~AP_InertialSensor_Revo();

    static AP_InertialSensor_Revo &from(AP_InertialSensor_Backend &backend) {
        return static_cast<AP_InertialSensor_Revo&>(backend);
    }

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                            enum Rotation rotation = ROTATION_NONE);

    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                            enum Rotation rotation = ROTATION_NONE);

    /* update accel and gyro state */
    bool update() override;
    void accumulate() override;

    void start() override;

    void _isr();
    void _ioc();

    enum Invensense_Type {
        Invensense_MPU6000=0,
        Invensense_MPU6500,
        Invensense_MPU9250,
        Invensense_ICM20608,
        Invensense_ICM20602,
    };
    
private:
    AP_InertialSensor_Revo(AP_InertialSensor &imu,
                              AP_HAL::OwnPtr<AP_HAL::Device> dev,
                              enum Rotation rotation);

    /* Initialize sensor*/
    bool _init();
    bool _hardware_init();
    bool _check_whoami();
    void _start();    // used for start and restart

    void _set_filter_register(void);

    /* Read samples from FIFO in RAM */
    void _read_fifo();

    /* Check if there's data available by either reading DRDY pin or register */
    bool _data_ready();

    /* Poll for new data (non-blocking) */
    void _poll_data();

    /* Read and write functions taking the differences between buses into
     * account */
    bool _block_read(uint8_t reg, uint8_t *buf, uint32_t size);
    uint8_t _register_read(uint8_t reg);
    void _register_write(uint8_t reg, uint8_t val, bool checked=false);

    bool _accumulate(uint8_t *samples, uint8_t n_samples);
    bool _accumulate_sensor_rate_sampling(uint8_t *samples, uint8_t n_samples);

    bool _check_raw_temp(int16_t t2);

    int16_t _raw_temp;
    
    // instance numbers of accel and gyro data
    uint8_t _gyro_instance;
    uint8_t _accel_instance;

    uint16_t _error_count;

    float temp_sensitivity = 1.0/340; // degC/LSB
    float temp_zero = 36.53; // degC
    
    float _temp_filtered;
    float _accel_scale;
    LowPassFilter2pFloat _temp_filter;

    enum Rotation _rotation;

    AP_HAL::DigitalSource *_drdy_pin;
    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    // which sensor type this is
    enum Invensense_Type _mpu_type;
    uint8_t product_id;

    // are we doing more than 1kHz sampling?
    bool _fast_sampling;

    // Last status from register user control
    uint8_t _last_stat_user_ctrl;    

    // buffer for fifo read
    uint8_t *_fifo_buffer; 


    /*
      accumulators for fast sampling
      See description in _accumulate_sensor_rate_sampling()
    */
    struct {
        Vector3f accel;
        Vector3f gyro;
        uint8_t count;
        LowPassFilterVector3f accel_filter{4000, 188};
        LowPassFilterVector3f gyro_filter{8000, 188};
    } _accum;
    
    uint16_t read_ptr;
    volatile uint16_t write_ptr; // changed in interrupt
    uint16_t nodata_count;
    void * task_handle;
    float accel_len;
    uint32_t last_sample;
    
    Vector3f gyro_mean;


//#define MPU_DEBUG_LOG

#ifdef MPU_DEBUG_LOG
#define MPU_LOG_SIZE 512
    static mpu_log_item mpu_log[MPU_LOG_SIZE];
    static uint16_t mpu_log_ptr;
#endif
};

