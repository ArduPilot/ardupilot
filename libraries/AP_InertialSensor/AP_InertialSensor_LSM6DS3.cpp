#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor_LSM6DS3.h"

#include <utility>

#include <AP_HAL/GPIO.h>

extern const AP_HAL::HAL& hal;


#define WHO_AM_I     0x6A

/*
 *  Accelerometer and Gyroscope registers
*/
#define LSM6DS3_FUNC_CFG_ACCESS                             0x01
#   define LSM6DS3_FUNC_CFG_ACCESS_CFG_EN               (0x1 << 7)
#define LSM6DS3_SENSOR_SYNC_TIME_FRAME                      0x04

#define LSM6DS3_FIFO_CTRL1                                  0x06
#define LSM6DS3_FIFO_CTRL2                                  0x07
#   define LSM6DS3_FIFO_CTRL2_TIMER_PEDO_FIFO_EN        (0x1 << 7)
#   define LSM6DS3_FIFO_CTRL2_TIMER_PEDO_FIFO_DRDY      (0x1 << 6)
#define LSM6DS3_FIFO_CTRL3                                  0x08
#   define LSM6DS3_FIFO_CTRL3_GYRO_factor_0             (0x1 << 3)
#   define LSM6DS3_FIFO_CTRL3_GYRO_factor_2             (0x2 << 3)
#   define LSM6DS3_FIFO_CTRL3_GYRO_factor_3             (0x3 << 3)
#   define LSM6DS3_FIFO_CTRL3_GYRO_factor_4             (0x4 << 3)
#   define LSM6DS3_FIFO_CTRL3_GYRO_factor_8             (0x5 << 3)
#   define LSM6DS3_FIFO_CTRL3_GYRO_factor_16            (0x6 << 3)
#   define LSM6DS3_FIFO_CTRL3_GYRO_factor_32            (0x7 << 3)
#   define LSM6DS3_FIFO_CTRL3_XL_factor_0               (0x1 << 0)
#   define LSM6DS3_FIFO_CTRL3_XL_factor_2               (0x2 << 0)
#   define LSM6DS3_FIFO_CTRL3_XL_factor_3               (0x3 << 0)
#   define LSM6DS3_FIFO_CTRL3_XL_factor_4               (0x4 << 0)
#   define LSM6DS3_FIFO_CTRL3_XL_factor_8               (0x5 << 0)
#   define LSM6DS3_FIFO_CTRL3_XL_factor_16              (0x6 << 0)
#   define LSM6DS3_FIFO_CTRL3_XL_factor_32              (0x7 << 0)
#define LSM6DS3_FIFO_CTRL4                                  0x09
#   define LSM6DS3_FIFO_CTRL4_STOP_ON_FTH				(0x1 << 7)
#   define LSM6DS3_FIFO_CTRL4_ONLY_HIGH_DATA            (0x1 << 6)
#   define LSM6DS3_FIFO_CTRL4_ONLY_HIGH_DATA            (0x1 << 6)
#   define LSM6DS3_FIFO_CTRL4_DEC_DS4_FIFO_factor_0     (0x1 << 3)
#   define LSM6DS3_FIFO_CTRL4_DEC_DS4_FIFO_factor_2     (0x2 << 3)
#   define LSM6DS3_FIFO_CTRL4_DEC_DS4_FIFO_factor_3     (0x3 << 3)
#   define LSM6DS3_FIFO_CTRL4_DEC_DS4_FIFO_factor_4     (0x4 << 3)
#   define LSM6DS3_FIFO_CTRL4_DEC_DS4_FIFO_factor_8     (0x5 << 3)
#   define LSM6DS3_FIFO_CTRL4_DEC_DS4_FIFO_factor_16    (0x6 << 3)
#   define LSM6DS3_FIFO_CTRL4_DEC_DS4_FIFO_factor_32    (0x7 << 3)
#   define LSM6DS3_FIFO_CTRL4_DEC_DS3_FIFO_factor_0     (0x1 << 0)
#   define LSM6DS3_FIFO_CTRL4_DEC_DS3_FIFO_factor_2     (0x2 << 0)
#   define LSM6DS3_FIFO_CTRL4_DEC_DS3_FIFO_factor_3     (0x3 << 0)
#   define LSM6DS3_FIFO_CTRL4_DEC_DS3_FIFO_factor_4     (0x4 << 0)
#   define LSM6DS3_FIFO_CTRL4_DEC_DS3_FIFO_factor_8     (0x5 << 0)
#   define LSM6DS3_FIFO_CTRL4_DEC_DS3_FIFO_factor_16    (0x6 << 0)
#   define LSM6DS3_FIFO_CTRL4_DEC_DS3_FIFO_factor_32    (0x7 << 0)
#define LSM6DS3_FIFO_CTRL5                                  0x0A
#   define LSM6DS3_FIFO_CTRL5_ODR_FIFO_12500mHz         (0x1 << 3)
#   define LSM6DS3_FIFO_CTRL5_ODR_FIFO_26Hz             (0x2 << 3)
#   define LSM6DS3_FIFO_CTRL5_ODR_FIFO_52Hz             (0x3 << 3)
#   define LSM6DS3_FIFO_CTRL5_ODR_FIFO_104Hz            (0x4 << 3)
#   define LSM6DS3_FIFO_CTRL5_ODR_FIFO_208Hz            (0x5 << 3)
#   define LSM6DS3_FIFO_CTRL5_ODR_FIFO_416Hz            (0x6 << 3)
#   define LSM6DS3_FIFO_CTRL5_ODR_FIFO_833Hz            (0x7 << 3)
#   define LSM6DS3_FIFO_CTRL5_ODR_FIFO_1660Hz           (0x8 << 3)
#   define LSM6DS3_FIFO_CTRL5_ODR_FIFO_3330Hz           (0x9 << 3)
#   define LSM6DS3_FIFO_CTRL5_ODR_FIFO_6660Hz           (0xA << 3)
#   define LSM6DS3_FIFO_CTRL5_FIFO_MODE_BYPASS          (0x0 << 0)
#   define LSM6DS3_FIFO_CTRL5_FIFO_MODE_FIFO            (0x1 << 0)
#   define LSM6DS3_FIFO_CTRL5_FIFO_MODE_STREAM          (0x3 << 0)
#   define LSM6DS3_FIFO_CTRL5_FIFO_MODE_B_to_C          (0x4 << 0)
#   define LSM6DS3_FIFO_CTRL5_FIFO_MODE_CON             (0x6 << 0)

#define LSM6DS3_WHO_AM_I                                    0x0F
#define LSM6DS3_CTRL1_XL                                    0x10
#   define LSM6DS3_CTRL1_XL_ODR_12500mHz                (0x1 << 4)
#   define LSM6DS3_CTRL1_XL_ODR_26Hz                    (0x2 << 4)
#   define LSM6DS3_CTRL1_XL_ODR_52Hz                    (0x3 << 4)
#   define LSM6DS3_CTRL1_XL_ODR_104Hz                   (0x4 << 4)
#   define LSM6DS3_CTRL1_XL_ODR_208Hz                   (0x5 << 4)
#   define LSM6DS3_CTRL1_XL_ODR_416Hz                   (0x6 << 4)
#   define LSM6DS3_CTRL1_XL_ODR_833Hz                   (0x7 << 4)
#   define LSM6DS3_CTRL1_XL_ODR_1660Hz                  (0x8 << 4)
#   define LSM6DS3_CTRL1_XL_ODR_3330Hz                  (0x9 << 4)
#   define LSM6DS3_CTRL1_XL_ODR_6660Hz                  (0xA << 4)
#   define LSM6DS3_CTRL1_XL_FS_2G                       (0x0 << 2)
#   define LSM6DS3_CTRL1_XL_FS_16G                      (0x1 << 2)
#   define LSM6DS3_CTRL1_XL_FS_4G                       (0x2 << 2)
#   define LSM6DS3_CTRL1_XL_FS_8G                       (0x3 << 2)
#   define LSM6DS3_CTRL1_XL_BW_400Hz                    (0x0 << 0)
#   define LSM6DS3_CTRL1_XL_BW_200Hz                    (0x1 << 0)
#   define LSM6DS3_CTRL1_XL_BW_100Hz                    (0x2 << 0)
#   define LSM6DS3_CTRL1_XL_BW_50Hz                     (0x3 << 0)
#define LSM6DS3_CTRL2_G                                     0x11
#   define LSM6DS3_CTRL2_G_ODR_12500mHz                 (0x1 << 4)
#   define LSM6DS3_CTRL2_G_ODR_26Hz                     (0x2 << 4)
#   define LSM6DS3_CTRL2_G_ODR_52Hz                     (0x3 << 4)
#   define LSM6DS3_CTRL2_G_ODR_104Hz                    (0x4 << 4)
#   define LSM6DS3_CTRL2_G_ODR_208Hz                    (0x5 << 4)
#   define LSM6DS3_CTRL2_G_ODR_416Hz                    (0x6 << 4)
#   define LSM6DS3_CTRL2_G_ODR_833Hz                    (0x7 << 4)
#   define LSM6DS3_CTRL2_G_ODR_1660Hz                   (0x8 << 4)
#   define LSM6DS3_CTRL2_G_FS_245DPS                    (0x0 << 2)
#   define LSM6DS3_CTRL2_G_FS_500DPS                    (0x1 << 2)
#   define LSM6DS3_CTRL2_G_FS_1000DPS                   (0x2 << 2)
#   define LSM6DS3_CTRL2_G_FS_2000DPS                   (0x3 << 2)
#   define LSM6DS3_CTRL2_G_FS_125DPS                    (0x1 << 1)
#define LSM6DS3_CTRL3_C                                     0x12
#   define LSM6DS3_CTRL3_C_BOOT                         (0x1 << 7)
#   define LSM6DS3_CTRL3_C_BDU                          (0x1 << 6)
#   define LSM6DS3_CTRL3_C_H_LACTIVE                    (0x1 << 5)
#   define LSM6DS3_CTRL3_C_PP_OD                        (0x1 << 4)
#   define LSM6DS3_CTRL3_C_SIM                          (0x1 << 3)
#   define LSM6DS3_CTRL3_C_IF_INC                       (0x1 << 2)
#   define LSM6DS3_CTRL3_C_BLE                          (0x1 << 1)
#   define LSM6DS3_CTRL3_C_SW_RESET                     (0x1 << 0)
#define LSM6DS3_CTRL4_C                                     0x13
#   define LSM6DS3_CTRL4_C_SLEEP	                    (0x1 << 6)
#   define LSM6DS3_CTRL4_C_INT2_on_INT1                 (0x1 << 5)
#   define LSM6DS3_CTRL4_C_DRDY_MASK                    (0x1 << 3)
#   define LSM6DS3_CTRL4_C_I2C_disable                  (0x1 << 2)

#define LSM6DS3_OUT_TEMP_L                                  0x20
#define LSM6DS3_OUT_TEMP_H                                  0x21

#define LSM6DS3_FIFO_STATUS1                                0x3A
#   define LSM6DS3_FIFO_STATUS1_UNREAD_WORDS              0x3F

#define LSM6DS3_FIFO_DATA_OUT_L                    0x3E
#define LSM6DS3_FIFO_DATA_OUT_H                    0x3F

AP_InertialSensor_LSM6DS3::AP_InertialSensor_LSM6DS3(AP_InertialSensor &imu,
                                                     AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                                     enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
    , _rotation(rotation)
    , _temp_filter(400, 1)
{
}

AP_InertialSensor_Backend *AP_InertialSensor_LSM6DS3::probe(AP_InertialSensor &_imu,
                                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                                            enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    AP_InertialSensor_LSM6DS3 *sensor =
        new AP_InertialSensor_LSM6DS3(_imu,std::move(dev),
                                      rotation);
    if (!sensor || !sensor->_init_sensor()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_InertialSensor_LSM6DS3::_init_sensor()
{
    _spi_sem = _dev->get_semaphore();
    
    bool success = _hardware_init();

    return success;
}

bool AP_InertialSensor_LSM6DS3::_hardware_init()
{
    _spi_sem->take_blocking();

    uint8_t tries, whoami=0;

    // set flag for reading registers
    _dev->set_read_flag(0x80);

    whoami = _register_read(LSM6DS3_WHO_AM_I);
    
    if (whoami != WHO_AM_I) {
        hal.console->printf("LSM6DS3: unexpected acc/gyro WHOAMI 0x%x\n", whoami);
        goto fail_whoami;
    }

   _register_write(LSM6DS3_CTRL3_C, LSM6DS3_CTRL3_C_SW_RESET);

    while (!_register_read(LSM6DS3_CTRL3_C))
        ;

    hal.console->printf("LSM6DS3_CTRL3_C: 0x%x\n", _register_read(LSM6DS3_CTRL3_C));

    _fifo_reset();

    hal.scheduler->delay(10);   

    for (tries = 0; tries < 5; tries++) {

        _dev->set_speed(AP_HAL::Device::SPEED_LOW);

        _gyro_init();
        _accel_init();

        _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

        hal.scheduler->delay(10);

        int frames = _register_read(LSM6DS3_FIFO_STATUS1);

        if (frames > 1) {
            break;
        }
    }
    if (tries == 5) {
        hal.console->printf("Failed to boot LSM6DS3 5 times\n\n");
        goto fail_tries;
    }

    _spi_sem->give();
    return true;

fail_tries:
fail_whoami:
    _spi_sem->give();
    return false;
}

void AP_InertialSensor_LSM6DS3::_fifo_reset()
{
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    //Disable FIFO and Enable Bypass mode
    _register_write(LSM6DS3_FIFO_CTRL5, 0x00);
    
    
    //Enable FIFO
    _register_write(LSM6DS3_FIFO_CTRL5, LSM6DS3_FIFO_CTRL5_FIFO_MODE_FIFO |
                                        LSM6DS3_FIFO_CTRL5_ODR_FIFO_833Hz);
                                        
    //Disable I2C
    _register_write(LSM6DS3_CTRL4_C, LSM6DS3_CTRL4_C_I2C_disable);


    //Enable block data update, allow auto-increment during multiple byte read
    _register_write(LSM6DS3_CTRL3_C, LSM6DS3_CTRL3_C_BDU | LSM6DS3_CTRL3_C_IF_INC);
    hal.scheduler->delay_microseconds(1);
 
    //set watermark at 32 samples and set decimation factor
    _register_write(LSM6DS3_FIFO_CTRL1, 32);
    _register_write(LSM6DS3_FIFO_CTRL3, LSM6DS3_FIFO_CTRL3_GYRO_factor_0 | LSM6DS3_FIFO_CTRL3_XL_factor_0);
    
    //Enable FIFO threshold level
    _register_write(LSM6DS3_FIFO_CTRL4, LSM6DS3_FIFO_CTRL4_STOP_ON_FTH);
    

    hal.scheduler->delay_microseconds(1);    


    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    notify_accel_fifo_reset(_accel_instance);
    notify_gyro_fifo_reset(_gyro_instance);
}

/*
  start the sensor going
 */
void AP_InertialSensor_LSM6DS3::start(void)
{
    if (!_imu.register_gyro(_gyro_instance, 833, _dev->get_bus_id_devtype(DEVTYPE_INS_LSM6DS3)) ||
        !_imu.register_accel(_accel_instance, 833, _dev->get_bus_id_devtype(DEVTYPE_INS_LSM6DS3))) {
        return;
    }

    set_accel_orientation(_accel_instance, _rotation);
    set_gyro_orientation(_gyro_instance, _rotation);

    _set_accel_max_abs_offset(_accel_instance, 5.0f);

    /* start the timer process to read samples */
    _dev->register_periodic_callback(1000, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_LSM6DS3::_poll_data, void));
}

uint8_t AP_InertialSensor_LSM6DS3::_register_read(uint8_t reg)
{
    uint8_t val = 0;
    _dev->read_registers(reg, &val, 1);
    return val;
}


void AP_InertialSensor_LSM6DS3::_register_write(uint8_t reg, uint8_t val, bool checked)
{
    _dev->write_register(reg, val, checked);
}

void AP_InertialSensor_LSM6DS3::_gyro_init()
{
    _register_write(LSM6DS3_CTRL2_G, LSM6DS3_CTRL2_G_ODR_833Hz |
                                              LSM6DS3_CTRL2_G_FS_2000DPS);
    hal.scheduler->delay(1);

    _set_gyro_scale();
    hal.scheduler->delay(1);
}

void AP_InertialSensor_LSM6DS3::_accel_init()
{
    _register_write(LSM6DS3_CTRL1_XL, LSM6DS3_CTRL1_XL_ODR_833Hz |
                                               LSM6DS3_CTRL1_XL_FS_16G);
    hal.scheduler->delay(1);

    _set_accel_scale(A_SCALE_16G);
    hal.scheduler->delay(1);
}

void AP_InertialSensor_LSM6DS3::_set_gyro_scale()
{
    /* scale value from datasheet 2000 mdps/digit */
    _gyro_scale = 70;
    /* convert mdps/digit to dps/digit */
    _gyro_scale /= 1000;
    /* convert dps/digit to (rad/s)/digit */
    _gyro_scale *= DEG_TO_RAD;
}

void AP_InertialSensor_LSM6DS3::_set_accel_scale(accel_scale scale)
{
    /*
     * Possible accelerometer scales (and their register bit settings) are:
     * 2 g (000), 4g (001), 6g (010) 8g (011), 16g (100). Here's a bit of an
     * algorithm to calculate g/(ADC tick) based on that 3-bit value:
     */
    _accel_scale = (((float) scale + 1.0f) * 2.0f) / 32768.0f;
    if (scale == A_SCALE_16G) {
        /* the datasheet shows an exception for +-16G */
        _accel_scale = 0.000488f;
    }
    /* convert to G/LSB to (m/s/s)/LSB */
    _accel_scale *= GRAVITY_MSS;
}

/**
 * Timer process to poll for new data from the LSM6DS3.
 */
void AP_InertialSensor_LSM6DS3::_poll_data()
{
    uint16_t samples=0;

    uint16_t frames = _register_read(LSM6DS3_FIFO_STATUS1);
     
    frames = frames & LSM6DS3_FIFO_STATUS1_UNREAD_WORDS;
    
    //(GYRO + XL) = 12 bytes (6 words)
    samples = frames / 6;

    if (samples > 1) {
    	_read_data_transaction(samples);    
    }

    if (frames == 30) {
        _fifo_reset();
        //hal.console->printf("FIFO_RESET\n");
    }

    // check next register value for correctness
    AP_HAL::Device::checkreg reg;
    if (!_dev->check_next_register(reg)) {
        log_register_change(_dev->get_bus_id(), reg);
        _inc_accel_error_count(_accel_instance);
    }

}


/*
 *  read from the FIFO data output registers
 */
void AP_InertialSensor_LSM6DS3::_read_data_transaction(uint16_t samples)
{
    struct sensor_data fifo_data_gyro = { };
    struct sensor_data fifo_data_accel = { };
    
    int32_t _gyro_bias[3] = {0, 0, 0};
    int32_t _accel_bias[3] = {0, 0, 0};

    const uint8_t _reg = LSM6DS3_FIFO_DATA_OUT_L | 0x80;

    // Read the gyro data stored in the FIFO
    for (int i = 0; i < samples; i++)
    {
        struct sensor_fifo_data data_temp = { };

        if (!_dev->transfer(&_reg, 1, (uint8_t *) &data_temp, sizeof(data_temp))) {
            hal.console->printf("LSM9DS1: error reading gyroscope\n");
            return;
        }

        _gyro_bias[0] += (int32_t) data_temp.gx; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        _gyro_bias[1] += (int32_t) data_temp.gy;
        _gyro_bias[2] += (int32_t) data_temp.gz;
        
        _accel_bias[0] += (int32_t) data_temp.ax; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        _accel_bias[1] += (int32_t) data_temp.ay;
        _accel_bias[2] += (int32_t) data_temp.az;
    }

    fifo_data_gyro.x = _gyro_bias[0] / samples; // average the data
    fifo_data_gyro.y = _gyro_bias[1] / samples;
    fifo_data_gyro.z = _gyro_bias[2] / samples;
    
    fifo_data_accel.x = _accel_bias[0] / samples; // average the data
    fifo_data_accel.y = _accel_bias[1] / samples;
    fifo_data_accel.z = _accel_bias[2] / samples;

    Vector3f gyro_data(fifo_data_gyro.x, fifo_data_gyro.y, fifo_data_gyro.z);
    gyro_data *= _gyro_scale;

    Vector3f accel_data(fifo_data_accel.x, fifo_data_accel.y, fifo_data_accel.z);
    accel_data *= _accel_scale;

    _rotate_and_correct_gyro(_gyro_instance, gyro_data);
    _notify_new_gyro_raw_sample(_gyro_instance, gyro_data);
    
    _rotate_and_correct_accel(_accel_instance, accel_data);
    _notify_new_accel_raw_sample(_accel_instance, accel_data);
    
    
    if (_temp_counter++ >= 10) {
        int16_t traw;
        const uint8_t regtemp = LSM6DS3_OUT_TEMP_L | 0x80;
        _temp_counter = 0;
        if (_dev->transfer(&regtemp, 1, (uint8_t *)&traw, sizeof(traw))) {
            _temperature = _temp_filter.apply(traw * 0.0625f + 25.0f);
        }
    }
    
}

bool AP_InertialSensor_LSM6DS3::update()
{
    update_gyro(_gyro_instance);
    update_accel(_accel_instance);
    _publish_temperature(_accel_instance, _temperature);

    return true;
}


