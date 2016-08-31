/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <assert.h>
#include <utility>

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor_MPU6000.h"

extern const AP_HAL::HAL& hal;

// MPU6000 accelerometer scaling
#define MPU6000_ACCEL_SCALE_1G    (GRAVITY_MSS / 4096.0f)

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <AP_HAL_Linux/GPIO.h>
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF
#define MPU6000_DRDY_PIN BBB_P8_14
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
#define MPU6000_DRDY_PIN RPI_GPIO_24
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_MINLURE
#define MPU6000_DRDY_PIN MINNOW_GPIO_I2S_CLK
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
#define MPU6000_EXT_SYNC_ENABLE 1
#endif
#endif

/*
  EXT_SYNC allows for frame synchronisation with an external device
  such as a camera. When enabled the LSB of AccelZ holds the FSYNC bit
 */
#ifndef MPU6000_EXT_SYNC_ENABLE
#define MPU6000_EXT_SYNC_ENABLE 0
#endif

// MPU 6000 registers
#define MPUREG_XG_OFFS_TC                       0x00
#define MPUREG_YG_OFFS_TC                       0x01
#define MPUREG_ZG_OFFS_TC                       0x02
#define MPUREG_X_FINE_GAIN                      0x03
#define MPUREG_Y_FINE_GAIN                      0x04
#define MPUREG_Z_FINE_GAIN                      0x05
#define MPUREG_XA_OFFS_H                        0x06    // X axis accelerometer offset (high byte)
#define MPUREG_XA_OFFS_L                        0x07    // X axis accelerometer offset (low byte)
#define MPUREG_YA_OFFS_H                        0x08    // Y axis accelerometer offset (high byte)
#define MPUREG_YA_OFFS_L                        0x09    // Y axis accelerometer offset (low byte)
#define MPUREG_ZA_OFFS_H                        0x0A    // Z axis accelerometer offset (high byte)
#define MPUREG_ZA_OFFS_L                        0x0B    // Z axis accelerometer offset (low byte)
#define MPUREG_PRODUCT_ID                       0x0C    // Product ID Register
#define MPUREG_XG_OFFS_USRH                     0x13    // X axis gyro offset (high byte)
#define MPUREG_XG_OFFS_USRL                     0x14    // X axis gyro offset (low byte)
#define MPUREG_YG_OFFS_USRH                     0x15    // Y axis gyro offset (high byte)
#define MPUREG_YG_OFFS_USRL                     0x16    // Y axis gyro offset (low byte)
#define MPUREG_ZG_OFFS_USRH                     0x17    // Z axis gyro offset (high byte)
#define MPUREG_ZG_OFFS_USRL                     0x18    // Z axis gyro offset (low byte)
#define MPUREG_SMPLRT_DIV                       0x19    // sample rate.  Fsample= 1Khz/(<this value>+1) = 200Hz
#       define MPUREG_SMPLRT_1000HZ                 0x00
#       define MPUREG_SMPLRT_500HZ                  0x01
#       define MPUREG_SMPLRT_250HZ                  0x03
#       define MPUREG_SMPLRT_200HZ                  0x04
#       define MPUREG_SMPLRT_100HZ                  0x09
#       define MPUREG_SMPLRT_50HZ                   0x13
#define MPUREG_CONFIG                           0x1A
#       define MPUREG_CONFIG_EXT_SYNC_SHIFT            3
#       define MPUREG_CONFIG_EXT_SYNC_GX            0x02
#       define MPUREG_CONFIG_EXT_SYNC_GY            0x03
#       define MPUREG_CONFIG_EXT_SYNC_GZ            0x04
#       define MPUREG_CONFIG_EXT_SYNC_AX            0x05
#       define MPUREG_CONFIG_EXT_SYNC_AY            0x06
#       define MPUREG_CONFIG_EXT_SYNC_AZ            0x07
#define MPUREG_GYRO_CONFIG                      0x1B
// bit definitions for MPUREG_GYRO_CONFIG
#       define BITS_GYRO_FS_250DPS                  0x00
#       define BITS_GYRO_FS_500DPS                  0x08
#       define BITS_GYRO_FS_1000DPS                 0x10
#       define BITS_GYRO_FS_2000DPS                 0x18
#       define BITS_GYRO_FS_MASK                    0x18 // only bits 3 and 4 are used for gyro full scale so use this to mask off other bits
#       define BITS_GYRO_ZGYRO_SELFTEST             0x20
#       define BITS_GYRO_YGYRO_SELFTEST             0x40
#       define BITS_GYRO_XGYRO_SELFTEST             0x80
#define MPUREG_ACCEL_CONFIG                     0x1C
#define MPUREG_MOT_THR                          0x1F    // detection threshold for Motion interrupt generation.  Motion is detected when the absolute value of any of the accelerometer measurements exceeds this
#define MPUREG_MOT_DUR                          0x20    // duration counter threshold for Motion interrupt generation. The duration counter ticks at 1 kHz, therefore MOT_DUR has a unit of 1 LSB = 1 ms
#define MPUREG_ZRMOT_THR                        0x21    // detection threshold for Zero Motion interrupt generation.
#define MPUREG_ZRMOT_DUR                        0x22    // duration counter threshold for Zero Motion interrupt generation. The duration counter ticks at 16 Hz, therefore ZRMOT_DUR has a unit of 1 LSB = 64 ms.
#define MPUREG_FIFO_EN                          0x23
#       define BIT_TEMP_FIFO_EN                     0x80
#       define BIT_XG_FIFO_EN                       0x40
#       define BIT_YG_FIFO_EN                       0x20
#       define BIT_ZG_FIFO_EN                       0x10
#       define BIT_ACCEL_FIFO_EN                    0x08
#       define BIT_SLV2_FIFO_EN                     0x04
#       define BIT_SLV1_FIFO_EN                     0x02
#       define BIT_SLV0_FIFI_EN0                    0x01
#define MPUREG_I2C_MST_CTRL                     0x24
#       define BIT_I2C_MST_P_NSR                    0x10
#       define BIT_I2C_MST_CLK_400KHZ               0x0D
#define MPUREG_I2C_SLV0_ADDR                    0x25
#define MPUREG_I2C_SLV1_ADDR                    0x28
#define MPUREG_I2C_SLV2_ADDR                    0x2B
#define MPUREG_I2C_SLV3_ADDR                    0x2E
#define MPUREG_INT_PIN_CFG                      0x37
#       define BIT_INT_RD_CLEAR                     0x10    // clear the interrupt when any read occurs
#       define BIT_LATCH_INT_EN                     0x20    // latch data ready pin
#define MPUREG_I2C_SLV4_CTRL                    0x34
#define MPUREG_INT_ENABLE                       0x38
// bit definitions for MPUREG_INT_ENABLE
#       define BIT_RAW_RDY_EN                       0x01
#       define BIT_DMP_INT_EN                       0x02    // enabling this bit (DMP_INT_EN) also enables RAW_RDY_EN it seems
#       define BIT_UNKNOWN_INT_EN                   0x04
#       define BIT_I2C_MST_INT_EN                   0x08
#       define BIT_FIFO_OFLOW_EN                    0x10
#       define BIT_ZMOT_EN                          0x20
#       define BIT_MOT_EN                           0x40
#       define BIT_FF_EN                            0x80
#define MPUREG_INT_STATUS                       0x3A
// bit definitions for MPUREG_INT_STATUS (same bit pattern as above because this register shows what interrupt actually fired)
#       define BIT_RAW_RDY_INT                      0x01
#       define BIT_DMP_INT                          0x02
#       define BIT_UNKNOWN_INT                      0x04
#       define BIT_I2C_MST_INT                      0x08
#       define BIT_FIFO_OFLOW_INT                   0x10
#       define BIT_ZMOT_INT                         0x20
#       define BIT_MOT_INT                          0x40
#       define BIT_FF_INT                           0x80
#define MPUREG_ACCEL_XOUT_H                     0x3B
#define MPUREG_ACCEL_XOUT_L                     0x3C
#define MPUREG_ACCEL_YOUT_H                     0x3D
#define MPUREG_ACCEL_YOUT_L                     0x3E
#define MPUREG_ACCEL_ZOUT_H                     0x3F
#define MPUREG_ACCEL_ZOUT_L                     0x40
#define MPUREG_TEMP_OUT_H                       0x41
#define MPUREG_TEMP_OUT_L                       0x42
#define MPUREG_GYRO_XOUT_H                      0x43
#define MPUREG_GYRO_XOUT_L                      0x44
#define MPUREG_GYRO_YOUT_H                      0x45
#define MPUREG_GYRO_YOUT_L                      0x46
#define MPUREG_GYRO_ZOUT_H                      0x47
#define MPUREG_GYRO_ZOUT_L                      0x48
#define MPUREG_EXT_SENS_DATA_00                 0x49
#define MPUREG_I2C_SLV0_DO                      0x63
#define MPUREG_I2C_MST_DELAY_CTRL               0x67
#       define BIT_I2C_SLV0_DLY_EN              0x01
#       define BIT_I2C_SLV1_DLY_EN              0x02
#       define BIT_I2C_SLV2_DLY_EN              0x04
#       define BIT_I2C_SLV3_DLY_EN              0x08
#define MPUREG_USER_CTRL                        0x6A
// bit definitions for MPUREG_USER_CTRL
#       define BIT_USER_CTRL_SIG_COND_RESET         0x01 // resets signal paths and results registers for all sensors (gyros, accel, temp)
#       define BIT_USER_CTRL_I2C_MST_RESET          0x02 // reset I2C Master (only applicable if I2C_MST_EN bit is set)
#       define BIT_USER_CTRL_FIFO_RESET             0x04 // Reset (i.e. clear) FIFO buffer
#       define BIT_USER_CTRL_DMP_RESET              0x08 // Reset DMP
#       define BIT_USER_CTRL_I2C_IF_DIS             0x10 // Disable primary I2C interface and enable hal.spi->interface
#       define BIT_USER_CTRL_I2C_MST_EN             0x20 // Enable MPU to act as the I2C Master to external slave sensors
#       define BIT_USER_CTRL_FIFO_EN                0x40 // Enable FIFO operations
#       define BIT_USER_CTRL_DMP_EN             0x80     // Enable DMP operations
#define MPUREG_PWR_MGMT_1                           0x6B
#       define BIT_PWR_MGMT_1_CLK_INTERNAL          0x00 // clock set to internal 8Mhz oscillator
#       define BIT_PWR_MGMT_1_CLK_XGYRO             0x01 // PLL with X axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_YGYRO             0x02 // PLL with Y axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_ZGYRO             0x03 // PLL with Z axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_EXT32KHZ          0x04 // PLL with external 32.768kHz reference
#       define BIT_PWR_MGMT_1_CLK_EXT19MHZ          0x05 // PLL with external 19.2MHz reference
#       define BIT_PWR_MGMT_1_CLK_STOP              0x07 // Stops the clock and keeps the timing generator in reset
#       define BIT_PWR_MGMT_1_TEMP_DIS              0x08 // disable temperature sensor
#       define BIT_PWR_MGMT_1_CYCLE                 0x20 // put sensor into cycle mode.  cycles between sleep mode and waking up to take a single sample of data from active sensors at a rate determined by LP_WAKE_CTRL
#       define BIT_PWR_MGMT_1_SLEEP                 0x40 // put sensor into low power sleep mode
#       define BIT_PWR_MGMT_1_DEVICE_RESET          0x80 // reset entire device
#define MPUREG_PWR_MGMT_2                       0x6C    // allows the user to configure the frequency of wake-ups in Accelerometer Only Low Power Mode
#define MPUREG_BANK_SEL                         0x6D    // DMP bank selection register (used to indirectly access DMP registers)
#define MPUREG_MEM_START_ADDR                   0x6E    // DMP memory start address (used to indirectly write to dmp memory)
#define MPUREG_MEM_R_W                              0x6F // DMP related register
#define MPUREG_DMP_CFG_1                            0x70 // DMP related register
#define MPUREG_DMP_CFG_2                            0x71 // DMP related register
#define MPUREG_FIFO_COUNTH                          0x72
#define MPUREG_FIFO_COUNTL                          0x73
#define MPUREG_FIFO_R_W                             0x74
#define MPUREG_WHOAMI                               0x75

#define BIT_READ_FLAG                           0x80
#define BIT_I2C_SLVX_EN                         0x80

// Configuration bits MPU 3000 and MPU 6000 (not revised)?
#define BITS_DLPF_CFG_256HZ_NOLPF2              0x00
#define BITS_DLPF_CFG_188HZ                         0x01
#define BITS_DLPF_CFG_98HZ                          0x02
#define BITS_DLPF_CFG_42HZ                          0x03
#define BITS_DLPF_CFG_20HZ                          0x04
#define BITS_DLPF_CFG_10HZ                          0x05
#define BITS_DLPF_CFG_5HZ                           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF              0x07
#define BITS_DLPF_CFG_MASK                          0x07

// Product ID Description for MPU6000
// high 4 bits  low 4 bits
// Product Name	Product Revision
#define MPU6000ES_REV_C4                        0x14    // 0001			0100
#define MPU6000ES_REV_C5                        0x15    // 0001			0101
#define MPU6000ES_REV_D6                        0x16    // 0001			0110
#define MPU6000ES_REV_D7                        0x17    // 0001			0111
#define MPU6000ES_REV_D8                        0x18    // 0001			1000
#define MPU6000_REV_C4                          0x54    // 0101			0100
#define MPU6000_REV_C5                          0x55    // 0101			0101
#define MPU6000_REV_D6                          0x56    // 0101			0110
#define MPU6000_REV_D7                          0x57    // 0101			0111
#define MPU6000_REV_D8                          0x58    // 0101			1000
#define MPU6000_REV_D9                          0x59    // 0101			1001

#define MPU6000_SAMPLE_SIZE 14

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BH
#define MPU6000_MAX_FIFO_SAMPLES 6
#else
#define MPU6000_MAX_FIFO_SAMPLES 3
#endif
#define MAX_DATA_READ (MPU6000_MAX_FIFO_SAMPLES * MPU6000_SAMPLE_SIZE)

#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx] << 8) | v[2*idx+1]))
#define uint16_val(v, idx)(((uint16_t)v[2*idx] << 8) | v[2*idx+1])

/*
 *  RM-MPU-6000A-00.pdf, page 33, section 4.25 lists LSB sensitivity of
 *  gyro as 16.4 LSB/DPS at scale factor of +/- 2000dps (FS_SEL==3)
 */
static const float GYRO_SCALE = (0.0174532f / 16.4f);

/*
 *  RM-MPU-6000A-00.pdf, page 31, section 4.23 lists LSB sensitivity of
 *  accel as 4096 LSB/mg at scale factor of +/- 8g (AFS_SEL==2)
 *
 *  See note below about accel scaling of engineering sample MPU6k
 *  variants however
 */

AP_InertialSensor_MPU6000::AP_InertialSensor_MPU6000(AP_InertialSensor &imu,
                                                     AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                                     bool use_fifo)
    : AP_InertialSensor_Backend(imu)
    , _use_fifo(use_fifo)
    , _temp_filter(1000, 1)
    , _dev(std::move(dev))
{
}

AP_InertialSensor_MPU6000::~AP_InertialSensor_MPU6000()
{
    delete _auxiliary_bus;
}

AP_InertialSensor_Backend *AP_InertialSensor_MPU6000::probe(AP_InertialSensor &imu,
                                                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    AP_InertialSensor_MPU6000 *sensor =
        new AP_InertialSensor_MPU6000(imu, std::move(dev), true);
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }
    sensor->_id = HAL_INS_MPU60XX_I2C;

    return sensor;
}


AP_InertialSensor_Backend *AP_InertialSensor_MPU6000::probe(AP_InertialSensor &imu,
                                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev)
{
    AP_InertialSensor_MPU6000 *sensor;

    dev->set_read_flag(0x80);

    sensor = new AP_InertialSensor_MPU6000(imu, std::move(dev), false);
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }
    sensor->_id = HAL_INS_MPU60XX_SPI;

    return sensor;
}

bool AP_InertialSensor_MPU6000::_init()
{
#ifdef MPU6000_DRDY_PIN
    _drdy_pin = hal.gpio->channel(MPU6000_DRDY_PIN);
    _drdy_pin->mode(HAL_GPIO_INPUT);
#endif

    hal.scheduler->suspend_timer_procs();
    bool success = _hardware_init();
    hal.scheduler->resume_timer_procs();

#if MPU6000_DEBUG
    _dump_registers();
#endif

    return success;
}

void AP_InertialSensor_MPU6000::_fifo_reset()
{
    _register_write(MPUREG_USER_CTRL, 0);
    _register_write(MPUREG_USER_CTRL, BIT_USER_CTRL_FIFO_RESET);
    _register_write(MPUREG_USER_CTRL, BIT_USER_CTRL_FIFO_EN);
}

void AP_InertialSensor_MPU6000::_fifo_enable()
{
    _register_write(MPUREG_FIFO_EN, BIT_XG_FIFO_EN | BIT_YG_FIFO_EN |
                    BIT_ZG_FIFO_EN | BIT_ACCEL_FIFO_EN | BIT_TEMP_FIFO_EN);
    _fifo_reset();
    hal.scheduler->delay(1);
}

bool AP_InertialSensor_MPU6000::_has_auxiliary_bus()
{
    return _dev->bus_type != AP_HAL::Device::BUS_TYPE_I2C;
}

void AP_InertialSensor_MPU6000::start()
{
    hal.scheduler->suspend_timer_procs();

    if (!_dev->get_semaphore()->take(100)) {
        AP_HAL::panic("MPU6000: Unable to get semaphore");
    }

    // initially run the bus at low speed
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    // only used for wake-up in accelerometer only low power mode
    _register_write(MPUREG_PWR_MGMT_2, 0x00);
    hal.scheduler->delay(1);

    if (_use_fifo) {
        _fifo_enable();
    }

    // disable sensor filtering
    _set_filter_register(256);

    // set sample rate to 1000Hz and apply a software filter
    // In this configuration, the gyro sample rate is 8kHz
    // Therefore the sample rate value is 8kHz/(SMPLRT_DIV + 1)
    // So we have to set it to 7 to have a 1kHz sampling
    // rate on the gyro
    _register_write(MPUREG_SMPLRT_DIV, 7);
    hal.scheduler->delay(1);

    // Gyro scale 2000º/s
    _register_write(MPUREG_GYRO_CONFIG, BITS_GYRO_FS_2000DPS);
    hal.scheduler->delay(1);

    // read the product ID rev c has 1/2 the sensitivity of rev d
    _product_id = _register_read(MPUREG_PRODUCT_ID);
    //Serial.printf("Product_ID= 0x%x\n", (unsigned) _mpu6000_product_id);

    // TODO: should be changed to 16G once we have a way to override the
    // previous offsets
    if ((_product_id == MPU6000ES_REV_C4) ||
        (_product_id == MPU6000ES_REV_C5) ||
        (_product_id == MPU6000_REV_C4)   ||
        (_product_id == MPU6000_REV_C5)) {
        // Accel scale 8g (4096 LSB/g)
        // Rev C has different scaling than rev D
        _register_write(MPUREG_ACCEL_CONFIG,1<<3);
    } else {
        // Accel scale 8g (4096 LSB/g)
        _register_write(MPUREG_ACCEL_CONFIG,2<<3);
    }
    hal.scheduler->delay(1);

    // configure interrupt to fire when new data arrives
    _register_write(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);
    hal.scheduler->delay(1);

    // clear interrupt on any read, and hold the data ready pin high
    // until we clear the interrupt
    _register_write(MPUREG_INT_PIN_CFG, BIT_INT_RD_CLEAR | BIT_LATCH_INT_EN);

    // now that we have initialised, we set the bus speed to high
    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    _dev->get_semaphore()->give();

    // grab the used instances
    _gyro_instance = _imu.register_gyro(1000);
    _accel_instance = _imu.register_accel(1000);

    hal.scheduler->resume_timer_procs();

    // start the timer process to read samples
    hal.scheduler->register_timer_process(
        FUNCTOR_BIND_MEMBER(&AP_InertialSensor_MPU6000::_poll_data, void));
}

/*
  process any
 */
bool AP_InertialSensor_MPU6000::update()
{
    update_accel(_accel_instance);
    update_gyro(_gyro_instance);

    _publish_temperature(_accel_instance, _temp_filtered);

    return true;
}

AuxiliaryBus *AP_InertialSensor_MPU6000::get_auxiliary_bus()
{
    if (_auxiliary_bus) {
        return _auxiliary_bus;
    }

    if (_has_auxiliary_bus()) {
        _auxiliary_bus = new AP_MPU6000_AuxiliaryBus(*this);
    }

    return _auxiliary_bus;
}

/*
 * Return true if the MPU6000 has new data available for reading.
 *
 * We use the data ready pin if it is available.  Otherwise, read the
 * status register.
 */
bool AP_InertialSensor_MPU6000::_data_ready()
{
    if (_drdy_pin) {
        return _drdy_pin->read() != 0;
    }
    uint8_t status = _register_read(MPUREG_INT_STATUS);
    return (status & BIT_RAW_RDY_INT) != 0;
}

/*
 * Timer process to poll for new data from the MPU6000.
 */
void AP_InertialSensor_MPU6000::_poll_data()
{
    if (!_dev->get_semaphore()->take_nonblocking()) {
        return;
    }

    if (_use_fifo) {
        _read_fifo();
    } else if (_data_ready()) {
        _read_sample();
    }

    _dev->get_semaphore()->give();
}

void AP_InertialSensor_MPU6000::_accumulate(uint8_t *samples, uint8_t n_samples)
{
    for (uint8_t i = 0; i < n_samples; i++) {
        uint8_t *data = samples + MPU6000_SAMPLE_SIZE * i;
        Vector3f accel, gyro;
        float temp;
        bool fsync_set = false;

#if MPU6000_EXT_SYNC_ENABLE
        fsync_set = (int16_val(data, 2) & 1U) != 0;
#endif
        
        accel = Vector3f(int16_val(data, 1),
                         int16_val(data, 0),
                         -int16_val(data, 2));
        accel *= MPU6000_ACCEL_SCALE_1G;

        gyro = Vector3f(int16_val(data, 5),
                        int16_val(data, 4),
                        -int16_val(data, 6));
        gyro *= GYRO_SCALE;

        temp = int16_val(data, 3);
        /* scaling/offset values from the datasheet */
        temp = temp/340 + 36.53;

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF
        accel.rotate(ROTATION_PITCH_180_YAW_90);
        gyro.rotate(ROTATION_PITCH_180_YAW_90);
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
        accel.rotate(ROTATION_YAW_270);
        gyro.rotate(ROTATION_YAW_270);
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
        accel.rotate(ROTATION_PITCH_180_YAW_90);
        gyro.rotate(ROTATION_PITCH_180_YAW_90);
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_MINLURE
        accel.rotate(ROTATION_YAW_90);
        gyro.rotate(ROTATION_YAW_90);
#endif

        _rotate_and_correct_accel(_accel_instance, accel);
        _rotate_and_correct_gyro(_gyro_instance, gyro);

        _notify_new_accel_raw_sample(_accel_instance, accel, AP_HAL::micros64(), fsync_set);
        _notify_new_gyro_raw_sample(_gyro_instance, gyro);

        _temp_filtered = _temp_filter.apply(temp);
    }
}

void AP_InertialSensor_MPU6000::_read_fifo()
{
    uint8_t n_samples;
    uint16_t bytes_read;
    uint8_t rx[MAX_DATA_READ];

    static_assert(MAX_DATA_READ <= 100, "Too big to keep on stack");

    if (!_block_read(MPUREG_FIFO_COUNTH, rx, 2)) {
        hal.console->printf("MPU60x0: error in fifo read\n");
        return;
    }

    bytes_read = uint16_val(rx, 0);
    n_samples = bytes_read / MPU6000_SAMPLE_SIZE;

    if (n_samples == 0) {
        /* Not enough data in FIFO */
        return;
    }

    if (n_samples > MPU6000_MAX_FIFO_SAMPLES) {
        hal.console->printf("bytes_read = %u, n_samples %u > %u, dropping samples\n",
                            bytes_read, n_samples, MPU6000_MAX_FIFO_SAMPLES);

        /* Too many samples, do a FIFO RESET */
        _fifo_reset();
        return;
    }

    if (!_block_read(MPUREG_FIFO_R_W, rx, n_samples * MPU6000_SAMPLE_SIZE)) {
        hal.console->printf("MPU60x0: error in fifo read %u bytes\n",
                            n_samples * MPU6000_SAMPLE_SIZE);
        return;
    }

    _accumulate(rx, n_samples);
}

void AP_InertialSensor_MPU6000::_read_sample()
{
    /* one register address followed by seven 2-byte registers */
    struct PACKED {
        uint8_t int_status;
        uint8_t d[14];
    } rx;

    if (!_block_read(MPUREG_INT_STATUS, (uint8_t *) &rx, sizeof(rx))) {
        if (++_error_count > 4) {
            // TODO: set bus speed low for this (and only this) device
            hal.console->printf("MPU60x0: error reading sample\n");
            return;
        }
    }

    _accumulate(rx.d, 1);
}

bool AP_InertialSensor_MPU6000::_block_read(uint8_t reg, uint8_t *buf,
                                            uint32_t size)
{
    return _dev->read_registers(reg, buf, size);
}

uint8_t AP_InertialSensor_MPU6000::_register_read(uint8_t reg)
{
    uint8_t val = 0;
    _dev->read_registers(reg, &val, 1);
    return val;
}

void AP_InertialSensor_MPU6000::_register_write(uint8_t reg, uint8_t val)
{
    _dev->write_register(reg, val);
}

/*
  set the DLPF filter frequency. Assumes caller has taken semaphore
 */
void AP_InertialSensor_MPU6000::_set_filter_register(uint16_t filter_hz)
{
    uint8_t filter;
    // choose filtering frequency
    if (filter_hz == 0) {
        filter = BITS_DLPF_CFG_256HZ_NOLPF2;
    } else if (filter_hz <= 5) {
        filter = BITS_DLPF_CFG_5HZ;
    } else if (filter_hz <= 10) {
        filter = BITS_DLPF_CFG_10HZ;
    } else if (filter_hz <= 20) {
        filter = BITS_DLPF_CFG_20HZ;
    } else if (filter_hz <= 42) {
        filter = BITS_DLPF_CFG_42HZ;
    } else if (filter_hz <= 98) {
        filter = BITS_DLPF_CFG_98HZ;
    } else {
        filter = BITS_DLPF_CFG_256HZ_NOLPF2;
    }

#if MPU6000_EXT_SYNC_ENABLE
    // add in EXT_SYNC bit if enabled
    filter |= (MPUREG_CONFIG_EXT_SYNC_AZ << MPUREG_CONFIG_EXT_SYNC_SHIFT);
#endif

    _register_write(MPUREG_CONFIG, filter);
}


bool AP_InertialSensor_MPU6000::_hardware_init(void)
{
    if (!_dev->get_semaphore()->take(100)) {
        AP_HAL::panic("MPU6000: Unable to get semaphore");
    }

    // initially run the bus at low speed
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    // Chip reset
    uint8_t tries;
    for (tries = 0; tries < 5; tries++) {
        uint8_t user_ctrl = _register_read(MPUREG_USER_CTRL);

        /* First disable the master I2C to avoid hanging the slaves on the
         * aulixiliar I2C bus - it will be enabled again if the AuxiliaryBus
         * is used */
        if (user_ctrl & BIT_USER_CTRL_I2C_MST_EN) {
            _register_write(MPUREG_USER_CTRL, user_ctrl & ~BIT_USER_CTRL_I2C_MST_EN);
            hal.scheduler->delay(10);
        }

        /* reset device */
        _register_write(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_DEVICE_RESET);
        hal.scheduler->delay(100);

        /* bus-dependent initialization */
        if (_dev->bus_type == AP_HAL::Device::BUS_TYPE_SPI) {
            /* Disable I2C bus if SPI selected (Recommended in Datasheet to be
             * done just after the device is reset) */
            _register_write(MPUREG_USER_CTRL, BIT_USER_CTRL_I2C_IF_DIS);
        }

        // Wake up device and select GyroZ clock. Note that the
        // MPU6000 starts up in sleep mode, and it can take some time
        // for it to come out of sleep
        _register_write(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_CLK_ZGYRO);
        hal.scheduler->delay(5);

        // check it has woken up
        if (_register_read(MPUREG_PWR_MGMT_1) == BIT_PWR_MGMT_1_CLK_ZGYRO) {
            break;
        }

        hal.scheduler->delay(10);
        if (_data_ready()) {
            break;
        }

#if MPU6000_DEBUG
        _dump_registers();
#endif
    }

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
    _dev->get_semaphore()->give();

    if (tries == 5) {
        hal.console->println("Failed to boot MPU6000 5 times");
        return false;
    }

    return true;
}

#if MPU6000_DEBUG
// dump all config registers - used for debug
void AP_InertialSensor_MPU6000::_dump_registers(void)
{
    hal.console->println("MPU6000 registers");
    if (!_dev->get_semaphore()->take(100)) {
        return;
    }

    for (uint8_t reg=MPUREG_PRODUCT_ID; reg<=108; reg++) {
        uint8_t v = _register_read(reg);
        hal.console->printf("%02x:%02x ", (unsigned)reg, (unsigned)v);
        if ((reg - (MPUREG_PRODUCT_ID-1)) % 16 == 0) {
            hal.console->println();
        }
    }
    hal.console->println();

    _dev->get_semaphore()->give();
}
#endif

AP_MPU6000_AuxiliaryBusSlave::AP_MPU6000_AuxiliaryBusSlave(AuxiliaryBus &bus, uint8_t addr,
                                                         uint8_t instance)
    : AuxiliaryBusSlave(bus, addr, instance)
    , _mpu6000_addr(MPUREG_I2C_SLV0_ADDR + _instance * 3)
    , _mpu6000_reg(_mpu6000_addr + 1)
    , _mpu6000_ctrl(_mpu6000_addr + 2)
    , _mpu6000_do(MPUREG_I2C_SLV0_DO + _instance)
{
}

int AP_MPU6000_AuxiliaryBusSlave::_set_passthrough(uint8_t reg, uint8_t size,
                                                  uint8_t *out)
{
    auto &backend = AP_InertialSensor_MPU6000::from(_bus.get_backend());
    uint8_t addr;

    /* Ensure the slave read/write is disabled before changing the registers */
    backend._register_write(_mpu6000_ctrl, 0);

    if (out) {
        backend._register_write(_mpu6000_do, *out);
        addr = _addr;
    } else {
        addr = _addr | BIT_READ_FLAG;
    }

    backend._register_write(_mpu6000_addr, addr);
    backend._register_write(_mpu6000_reg, reg);
    backend._register_write(_mpu6000_ctrl, BIT_I2C_SLVX_EN | size);

    return 0;
}

int AP_MPU6000_AuxiliaryBusSlave::passthrough_read(uint8_t reg, uint8_t *buf,
                                                   uint8_t size)
{
    assert(buf);

    if (_registered) {
        hal.console->println("Error: can't passthrough when slave is already configured");
        return -1;
    }

    int r = _set_passthrough(reg, size);
    if (r < 0) {
        return r;
    }

    /* wait the value to be read from the slave and read it back */
    hal.scheduler->delay(10);

    auto &backend = AP_InertialSensor_MPU6000::from(_bus.get_backend());
    if (!backend._block_read(MPUREG_EXT_SENS_DATA_00 + _ext_sens_data, buf, size)) {
        return -1;
    }

    /* disable new reads */
    backend._register_write(_mpu6000_ctrl, 0);

    return size;
}

int AP_MPU6000_AuxiliaryBusSlave::passthrough_write(uint8_t reg, uint8_t val)
{
    if (_registered) {
        hal.console->println("Error: can't passthrough when slave is already configured");
        return -1;
    }

    int r = _set_passthrough(reg, 1, &val);
    if (r < 0) {
        return r;
    }

    /* wait the value to be written to the slave */
    hal.scheduler->delay(10);

    auto &backend = AP_InertialSensor_MPU6000::from(_bus.get_backend());

    /* disable new writes */
    backend._register_write(_mpu6000_ctrl, 0);

    return 1;
}

int AP_MPU6000_AuxiliaryBusSlave::read(uint8_t *buf)
{
    if (!_registered) {
        hal.console->println("Error: can't read before configuring slave");
        return -1;
    }

    auto &backend = AP_InertialSensor_MPU6000::from(_bus.get_backend());
    if (!backend._block_read(MPUREG_EXT_SENS_DATA_00 + _ext_sens_data, buf, _sample_size)) {
        return -1;
    }

    return _sample_size;
}

/* MPU6000 provides up to 5 slave devices, but the 5th is way too different to
 * configure and is seldom used */
AP_MPU6000_AuxiliaryBus::AP_MPU6000_AuxiliaryBus(AP_InertialSensor_MPU6000 &backend)
    : AuxiliaryBus(backend, 4)
{
}

AP_HAL::Semaphore *AP_MPU6000_AuxiliaryBus::get_semaphore()
{
    return static_cast<AP_InertialSensor_MPU6000&>(_ins_backend)._dev->get_semaphore();
}

AuxiliaryBusSlave *AP_MPU6000_AuxiliaryBus::_instantiate_slave(uint8_t addr, uint8_t instance)
{
    /* Enable slaves on MPU6000 if this is the first time */
    if (_ext_sens_data == 0) {
        _configure_slaves();
    }

    return new AP_MPU6000_AuxiliaryBusSlave(*this, addr, instance);
}

void AP_MPU6000_AuxiliaryBus::_configure_slaves()
{
    auto &backend = AP_InertialSensor_MPU6000::from(_ins_backend);

    /* Enable the I2C master to slaves on the auxiliary I2C bus*/
    uint8_t user_ctrl = backend._register_read(MPUREG_USER_CTRL);
    backend._register_write(MPUREG_USER_CTRL, user_ctrl | BIT_USER_CTRL_I2C_MST_EN);

    /* stop condition between reads; clock at 400kHz */
    backend._register_write(MPUREG_I2C_MST_CTRL,
                            BIT_I2C_MST_P_NSR | BIT_I2C_MST_CLK_400KHZ);

    /* Hard-code divider for internal sample rate, 1 kHz, resulting in a
     * sample rate of 100Hz */
    backend._register_write(MPUREG_I2C_SLV4_CTRL, 9);

    /* All slaves are subject to the sample rate */
    backend._register_write(MPUREG_I2C_MST_DELAY_CTRL,
                            BIT_I2C_SLV0_DLY_EN | BIT_I2C_SLV1_DLY_EN |
                            BIT_I2C_SLV2_DLY_EN | BIT_I2C_SLV3_DLY_EN);
}

int AP_MPU6000_AuxiliaryBus::_configure_periodic_read(AuxiliaryBusSlave *slave,
                                                     uint8_t reg, uint8_t size)
{
    if (_ext_sens_data + size > MAX_EXT_SENS_DATA) {
        return -1;
    }

    AP_MPU6000_AuxiliaryBusSlave *mpu_slave =
        static_cast<AP_MPU6000_AuxiliaryBusSlave*>(slave);
    mpu_slave->_set_passthrough(reg, size);
    mpu_slave->_ext_sens_data = _ext_sens_data;
    _ext_sens_data += size;

    return 0;
}
