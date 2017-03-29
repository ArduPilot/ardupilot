/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/*
  driver for all supported Invensense IMUs, including MPU6000, MPU9250
  ICM-20608 and ICM-20602
 */

#include <assert.h>
#include <utility>
#include <stdio.h>

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor_Invensense.h"

extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#include <AP_HAL_Linux/GPIO.h>
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF
#define INVENSENSE_DRDY_PIN BBB_P8_14
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
#define INVENSENSE_DRDY_PIN RPI_GPIO_24
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_MINLURE
#define INVENSENSE_DRDY_PIN MINNOW_GPIO_I2S_CLK
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
#define INVENSENSE_EXT_SYNC_ENABLE 1
#endif
#endif

#define debug(fmt, args ...)  do {printf("MPU: " fmt "\n", ## args); } while(0)

/*
  EXT_SYNC allows for frame synchronisation with an external device
  such as a camera. When enabled the LSB of AccelZ holds the FSYNC bit
 */
#ifndef INVENSENSE_EXT_SYNC_ENABLE
#define INVENSENSE_EXT_SYNC_ENABLE 0
#endif

// common registers
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
#       define MPUREG_CONFIG_FIFO_MODE_STOP         0x40
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
#       define BIT_BYPASS_EN                        0x02
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

// ICM20608 specific registers
#define ICMREG_ACCEL_CONFIG2          0x1D
#define ICM_ACC_DLPF_CFG_1046HZ_NOLPF 0x00
#define ICM_ACC_DLPF_CFG_218HZ        0x01
#define ICM_ACC_DLPF_CFG_99HZ         0x02
#define ICM_ACC_DLPF_CFG_44HZ         0x03
#define ICM_ACC_DLPF_CFG_21HZ         0x04
#define ICM_ACC_DLPF_CFG_10HZ         0x05
#define ICM_ACC_DLPF_CFG_5HZ          0x06
#define ICM_ACC_DLPF_CFG_420HZ        0x07
#define ICM_ACC_FCHOICE_B             0x08

/* this is an undocumented register which
   if set incorrectly results in getting a 2.7m/s/s offset
   on the Y axis of the accelerometer
*/
#define MPUREG_ICM_UNDOC1       0x11
#define MPUREG_ICM_UNDOC1_VALUE 0xc9

// WHOAMI values
#define MPU_WHOAMI_6000			0x68
#define MPU_WHOAMI_20608		0xaf
#define MPU_WHOAMI_20602		0x12
#define MPU_WHOAMI_6500			0x70
#define MPU_WHOAMI_MPU9250      0x71
#define MPU_WHOAMI_MPU9255      0x73

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

// Product ID Description for MPU6000. Used to detect buggy chips
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

#define MPU_SAMPLE_SIZE 14
#define MPU_FIFO_DOWNSAMPLE_COUNT 8
#define MPU_FIFO_BUFFER_LEN 16

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

AP_InertialSensor_Invensense::AP_InertialSensor_Invensense(AP_InertialSensor &imu,
                                                           AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                                           enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
    , _temp_filter(1000, 1)
    , _rotation(rotation)
    , _dev(std::move(dev))
{
}

AP_InertialSensor_Invensense::~AP_InertialSensor_Invensense()
{
    if (_fifo_buffer != nullptr) {
        hal.util->dma_free(_fifo_buffer, MPU_FIFO_BUFFER_LEN * MPU_SAMPLE_SIZE);
    }
    delete _auxiliary_bus;
}

AP_InertialSensor_Backend *AP_InertialSensor_Invensense::probe(AP_InertialSensor &imu,
                                                               AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                                               enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_InertialSensor_Invensense *sensor =
        new AP_InertialSensor_Invensense(imu, std::move(dev), rotation);
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }
    if (sensor->_mpu_type == Invensense_MPU9250) {
        sensor->_id = HAL_INS_MPU9250_I2C;
    } else {
        sensor->_id = HAL_INS_MPU60XX_I2C;
    }

    return sensor;
}


AP_InertialSensor_Backend *AP_InertialSensor_Invensense::probe(AP_InertialSensor &imu,
                                                               AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                                               enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_InertialSensor_Invensense *sensor;

    dev->set_read_flag(0x80);

    sensor = new AP_InertialSensor_Invensense(imu, std::move(dev), rotation);
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }
    if (sensor->_mpu_type == Invensense_MPU9250) {
        sensor->_id = HAL_INS_MPU9250_SPI;
    } else if (sensor->_mpu_type == Invensense_MPU6500) {
        sensor->_id = HAL_INS_MPU6500;
    } else {
        sensor->_id = HAL_INS_MPU60XX_SPI;
    }

    return sensor;
}

bool AP_InertialSensor_Invensense::_init()
{
#ifdef INVENSENSE_DRDY_PIN
    _drdy_pin = hal.gpio->channel(INVENSENSE_DRDY_PIN);
    _drdy_pin->mode(HAL_GPIO_INPUT);
#endif

    bool success = _hardware_init();

    return success;
}

void AP_InertialSensor_Invensense::_fifo_reset()
{
    uint8_t user_ctrl = _last_stat_user_ctrl;
    user_ctrl &= ~(BIT_USER_CTRL_FIFO_RESET | BIT_USER_CTRL_FIFO_EN);
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);
    _register_write(MPUREG_FIFO_EN, 0);
    _register_write(MPUREG_USER_CTRL, user_ctrl);
    _register_write(MPUREG_USER_CTRL, user_ctrl | BIT_USER_CTRL_FIFO_RESET);
    _register_write(MPUREG_USER_CTRL, user_ctrl | BIT_USER_CTRL_FIFO_EN);
    _register_write(MPUREG_FIFO_EN, BIT_XG_FIFO_EN | BIT_YG_FIFO_EN |
                    BIT_ZG_FIFO_EN | BIT_ACCEL_FIFO_EN | BIT_TEMP_FIFO_EN, true);
    hal.scheduler->delay_microseconds(1);
    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
    _last_stat_user_ctrl = user_ctrl | BIT_USER_CTRL_FIFO_EN;
}

bool AP_InertialSensor_Invensense::_has_auxiliary_bus()
{
    return _dev->bus_type() != AP_HAL::Device::BUS_TYPE_I2C;
}

void AP_InertialSensor_Invensense::start()
{
    if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return;
    }

    // initially run the bus at low speed
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    // only used for wake-up in accelerometer only low power mode
    _register_write(MPUREG_PWR_MGMT_2, 0x00);
    hal.scheduler->delay(1);

    // always use FIFO
    _fifo_reset();

    // grab the used instances
    enum DevTypes gdev, adev;
    switch (_mpu_type) {
    case Invensense_MPU9250:
        gdev = DEVTYPE_GYR_MPU9250;
        adev = DEVTYPE_ACC_MPU9250;
        break;
    case Invensense_MPU6000:
    case Invensense_MPU6500:
    case Invensense_ICM20608:
    case Invensense_ICM20602:
    default:
        gdev = DEVTYPE_GYR_MPU6000;
        adev = DEVTYPE_ACC_MPU6000;
        break;
    }

    /*
      setup temperature sensitivity and offset. This varies
      considerably between parts
     */
    switch (_mpu_type) {
    case Invensense_MPU9250:
        temp_zero = 21;
        temp_sensitivity = 1.0/340;
        break;

    case Invensense_MPU6000:
    case Invensense_MPU6500:
        temp_zero = 36.53;
        temp_sensitivity = 1.0/340;
        break;

    case Invensense_ICM20608:
    case Invensense_ICM20602:
        temp_zero = 25;
        temp_sensitivity = 1.0/326.8; 
        break;
    }

    _gyro_instance = _imu.register_gyro(1000, _dev->get_bus_id_devtype(gdev));
    _accel_instance = _imu.register_accel(1000, _dev->get_bus_id_devtype(adev));

    // setup ODR and on-sensor filtering
    _set_filter_register();

    // set sample rate to 1000Hz and apply a software filter
    // In this configuration, the gyro sample rate is 8kHz
    _register_write(MPUREG_SMPLRT_DIV, 0, true);
    hal.scheduler->delay(1);

    // Gyro scale 2000º/s
    _register_write(MPUREG_GYRO_CONFIG, BITS_GYRO_FS_2000DPS, true);
    hal.scheduler->delay(1);

    // read the product ID rev c has 1/2 the sensitivity of rev d
    uint8_t product_id = _register_read(MPUREG_PRODUCT_ID);

    if (_mpu_type == Invensense_MPU6000 &&
        ((product_id == MPU6000ES_REV_C4) ||
         (product_id == MPU6000ES_REV_C5) ||
         (product_id == MPU6000_REV_C4)   ||
         (product_id == MPU6000_REV_C5))) {
        // Accel scale 8g (4096 LSB/g)
        // Rev C has different scaling than rev D
        _register_write(MPUREG_ACCEL_CONFIG,1<<3, true);
        _accel_scale = GRAVITY_MSS / 4096.f;
    } else {
        // Accel scale 16g (2048 LSB/g)
        _register_write(MPUREG_ACCEL_CONFIG,3<<3, true);
        _accel_scale = GRAVITY_MSS / 2048.f;
    }
    hal.scheduler->delay(1);

	if (_mpu_type == Invensense_ICM20608 ||
        _mpu_type == Invensense_ICM20602) {
        // this avoids a sensor bug, see description above
		_register_write(MPUREG_ICM_UNDOC1, MPUREG_ICM_UNDOC1_VALUE, true);
	}
    
    // configure interrupt to fire when new data arrives
    _register_write(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);
    hal.scheduler->delay(1);

    // clear interrupt on any read, and hold the data ready pin high
    // until we clear the interrupt
    _register_write(MPUREG_INT_PIN_CFG, _register_read(MPUREG_INT_PIN_CFG) | BIT_INT_RD_CLEAR | BIT_LATCH_INT_EN);

    // now that we have initialised, we set the bus speed to high
    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    _dev->get_semaphore()->give();

    // setup sensor rotations from probe()
    set_gyro_orientation(_gyro_instance, _rotation);
    set_accel_orientation(_accel_instance, _rotation);

    // allocate fifo buffer
    _fifo_buffer = (uint8_t *)hal.util->dma_allocate(MPU_FIFO_BUFFER_LEN * MPU_SAMPLE_SIZE);
    if (_fifo_buffer == nullptr) {
        AP_HAL::panic("Invensense: Unable to allocate FIFO buffer");
    }

    // start the timer process to read samples
    _dev->register_periodic_callback(1000, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_Invensense::_poll_data, void));
}


/*
  publish any pending data
 */
bool AP_InertialSensor_Invensense::update()
{
    update_accel(_accel_instance);
    update_gyro(_gyro_instance);

    _publish_temperature(_accel_instance, _temp_filtered);

    return true;
}

/*
  accumulate new samples
 */
void AP_InertialSensor_Invensense::accumulate()
{
    // nothing to do
}

AuxiliaryBus *AP_InertialSensor_Invensense::get_auxiliary_bus()
{
    if (_auxiliary_bus) {
        return _auxiliary_bus;
    }

    if (_has_auxiliary_bus()) {
        _auxiliary_bus = new AP_Invensense_AuxiliaryBus(*this, _dev->get_bus_id());
    }

    return _auxiliary_bus;
}

/*
 * Return true if the Invensense has new data available for reading.
 *
 * We use the data ready pin if it is available.  Otherwise, read the
 * status register.
 */
bool AP_InertialSensor_Invensense::_data_ready()
{
    if (_drdy_pin) {
        return _drdy_pin->read() != 0;
    }
    uint8_t status = _register_read(MPUREG_INT_STATUS);
    return (status & BIT_RAW_RDY_INT) != 0;
}

/*
 * Timer process to poll for new data from the Invensense. Called from bus thread with semaphore held
 */
void AP_InertialSensor_Invensense::_poll_data()
{
    _read_fifo();
}

bool AP_InertialSensor_Invensense::_accumulate(uint8_t *samples, uint8_t n_samples)
{
    for (uint8_t i = 0; i < n_samples; i++) {
        const uint8_t *data = samples + MPU_SAMPLE_SIZE * i;
        Vector3f accel, gyro;
        bool fsync_set = false;

#if INVENSENSE_EXT_SYNC_ENABLE
        fsync_set = (int16_val(data, 2) & 1U) != 0;
#endif
        
        accel = Vector3f(int16_val(data, 1),
                         int16_val(data, 0),
                         -int16_val(data, 2));
        accel *= _accel_scale;

        int16_t t2 = int16_val(data, 3);
        if (!_check_raw_temp(t2)) {
            debug("temp reset %d %d", _raw_temp, t2);
            _fifo_reset();
            return false;
        }
        float temp = t2 * temp_sensitivity + temp_zero;
        
        gyro = Vector3f(int16_val(data, 5),
                        int16_val(data, 4),
                        -int16_val(data, 6));
        gyro *= GYRO_SCALE;

        _rotate_and_correct_accel(_accel_instance, accel);
        _rotate_and_correct_gyro(_gyro_instance, gyro);

        _notify_new_accel_raw_sample(_accel_instance, accel, AP_HAL::micros64(), fsync_set);
        _notify_new_gyro_raw_sample(_gyro_instance, gyro);

        _temp_filtered = _temp_filter.apply(temp);
    }
    return true;
}

/*
  when doing fast sampling the sensor gives us 8k samples/second. Every 2nd accel sample is a duplicate.

  To filter this we first apply a 1p low pass filter at 188Hz, then we
  average over 8 samples to bring the data rate down to 1kHz. This
  gives very good aliasing rejection at frequencies well above what
  can be handled with 1kHz sample rates.
 */
bool AP_InertialSensor_Invensense::_accumulate_fast_sampling(uint8_t *samples, uint8_t n_samples)
{
    int32_t tsum = 0;
    const int32_t clip_limit = AP_INERTIAL_SENSOR_ACCEL_CLIP_THRESH_MSS / _accel_scale;
    bool clipped = false;
    bool ret = true;
    
    for (uint8_t i = 0; i < n_samples; i++) {
        const uint8_t *data = samples + MPU_SAMPLE_SIZE * i;

        // use temperatue to detect FIFO corruption
        int16_t t2 = int16_val(data, 3);
        if (!_check_raw_temp(t2)) {
            debug("temp reset %d %d", _raw_temp, t2);
            _fifo_reset();
            ret = false;
            break;
        }
        tsum += t2;

        if ((_accum.count & 1) == 0) {
            // accel data is at 4kHz
            Vector3f a(int16_val(data, 1),
                       int16_val(data, 0),
                       -int16_val(data, 2));
            if (fabsf(a.x) > clip_limit ||
                fabsf(a.y) > clip_limit ||
                fabsf(a.z) > clip_limit) {
                clipped = true;
            }
            _accum.accel += _accum.accel_filter.apply(a);
        }

        Vector3f g(int16_val(data, 5),
                   int16_val(data, 4),
                   -int16_val(data, 6));

        _accum.gyro += _accum.gyro_filter.apply(g);
        _accum.count++;

        if (_accum.count == MPU_FIFO_DOWNSAMPLE_COUNT) {
            float ascale = _accel_scale / (MPU_FIFO_DOWNSAMPLE_COUNT/2);
            _accum.accel *= ascale;

            float gscale = GYRO_SCALE / MPU_FIFO_DOWNSAMPLE_COUNT;
            _accum.gyro *= gscale;
            
            _rotate_and_correct_accel(_accel_instance, _accum.accel);
            _rotate_and_correct_gyro(_gyro_instance, _accum.gyro);
            
            _notify_new_accel_raw_sample(_accel_instance, _accum.accel, AP_HAL::micros64(), false);
            _notify_new_gyro_raw_sample(_gyro_instance, _accum.gyro);
            
            _accum.accel.zero();
            _accum.gyro.zero();
            _accum.count = 0;
        }
    }

    if (clipped) {
        increment_clip_count(_accel_instance);
    }

    if (ret) {
        float temp = (static_cast<float>(tsum)/n_samples)*temp_sensitivity + temp_zero;
        _temp_filtered = _temp_filter.apply(temp);
    }
    
    return ret;
}

void AP_InertialSensor_Invensense::_read_fifo()
{
    uint8_t n_samples;
    uint16_t bytes_read;
    uint8_t *rx = _fifo_buffer;
    bool need_reset = false;

    if (!_block_read(MPUREG_FIFO_COUNTH, rx, 2)) {
        goto check_registers;
    }

    bytes_read = uint16_val(rx, 0);
    n_samples = bytes_read / MPU_SAMPLE_SIZE;

    if (n_samples == 0) {
        /* Not enough data in FIFO */
        goto check_registers;
    }

    /*
      testing has shown that if we have more than 32 samples in the
      FIFO then some of those samples will be corrupt. It always is
      the ones at the end of the FIFO, so clear those with a reset
      once we've read the first 24. Reading 24 gives us the normal
      number of samples for fast sampling at 400Hz
     */
    if (n_samples > 32) {
        need_reset = true;
        n_samples = 24;
    }
    
    while (n_samples > 0) {
        uint8_t n = MIN(n_samples, MPU_FIFO_BUFFER_LEN);
        if (!_dev->set_chip_select(true)) {
            if (!_block_read(MPUREG_FIFO_R_W, rx, n * MPU_SAMPLE_SIZE)) {
                goto check_registers;
            }
        } else {
            // this ensures we keep things nicely setup for DMA
            uint8_t reg = MPUREG_FIFO_R_W | 0x80;
            if (!_dev->transfer(&reg, 1, nullptr, 0)) {
                _dev->set_chip_select(false);
                goto check_registers;
            }
            memset(rx, 0, n * MPU_SAMPLE_SIZE);
            if (!_dev->transfer(rx, n * MPU_SAMPLE_SIZE, rx, n * MPU_SAMPLE_SIZE)) {
                hal.console->printf("MPU60x0: error in fifo read %u bytes\n", n * MPU_SAMPLE_SIZE);
                _dev->set_chip_select(false);
                goto check_registers;
            }
            _dev->set_chip_select(false);
        }

        if (_fast_sampling) {
            if (!_accumulate_fast_sampling(rx, n)) {
                debug("stop at %u of %u", n_samples, bytes_read/MPU_SAMPLE_SIZE);
                break;
            }
        } else {
            if (!_accumulate(rx, n)) {
                break;
            }
        }
        n_samples -= n;
    }

    if (need_reset) {
        //debug("fifo reset n_samples %u", bytes_read/MPU_SAMPLE_SIZE);
        _fifo_reset();
    }
    
check_registers:
    // check next register value for correctness
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);
    if (!_dev->check_next_register()) {
        _inc_gyro_error_count(_gyro_instance);
        _inc_accel_error_count(_accel_instance);
    }
    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
}

/*
  fetch temperature in order to detect FIFO sync errors
*/
bool AP_InertialSensor_Invensense::_check_raw_temp(int16_t t2)
{
    if (abs(t2 - _raw_temp) < 400) {
        // cached copy OK
        return true;
    }
    uint8_t trx[2];
    if (_block_read(MPUREG_TEMP_OUT_H, trx, 2)) {
        _raw_temp = int16_val(trx, 0);
    }
    return (abs(t2 - _raw_temp) < 400);
}

bool AP_InertialSensor_Invensense::_block_read(uint8_t reg, uint8_t *buf,
                                            uint32_t size)
{
    return _dev->read_registers(reg, buf, size);
}

uint8_t AP_InertialSensor_Invensense::_register_read(uint8_t reg)
{
    uint8_t val = 0;
    _dev->read_registers(reg, &val, 1);
    return val;
}

void AP_InertialSensor_Invensense::_register_write(uint8_t reg, uint8_t val, bool checked)
{
    _dev->write_register(reg, val, checked);
}

/*
  set the DLPF filter frequency. Assumes caller has taken semaphore
 */
void AP_InertialSensor_Invensense::_set_filter_register(void)
{
    uint8_t config;

#if INVENSENSE_EXT_SYNC_ENABLE
    // add in EXT_SYNC bit if enabled
    config = (MPUREG_CONFIG_EXT_SYNC_AZ << MPUREG_CONFIG_EXT_SYNC_SHIFT);
#else
    config = 0;
#endif

    if (enable_fast_sampling(_accel_instance)) {
        _fast_sampling = (_mpu_type != Invensense_MPU6000 && _dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI);
        if (_fast_sampling) {
            hal.console->printf("MPU[%u]: enabled fast sampling\n", _accel_instance);
        }
    }
    
    if (_fast_sampling) {
        // this gives us 8kHz sampling on gyros and 4kHz on accels
        config |= BITS_DLPF_CFG_256HZ_NOLPF2;
    } else {
        // limit to 1kHz if not on SPI
        config |= BITS_DLPF_CFG_188HZ;
    }

    config |= MPUREG_CONFIG_FIFO_MODE_STOP;
    _register_write(MPUREG_CONFIG, config, true);

	if (_mpu_type != Invensense_MPU6000) {
        if (_fast_sampling) {
            // setup for 4kHz accels
            _register_write(ICMREG_ACCEL_CONFIG2, ICM_ACC_FCHOICE_B, true);
        } else {
            _register_write(ICMREG_ACCEL_CONFIG2, ICM_ACC_DLPF_CFG_218HZ, true);
        }
    }
}

/*
  check whoami for sensor type
 */
bool AP_InertialSensor_Invensense::_check_whoami(void)
{
    uint8_t whoami = _register_read(MPUREG_WHOAMI);
    switch (whoami) {
    case MPU_WHOAMI_6000:
        _mpu_type = Invensense_MPU6000;
        return true;
    case MPU_WHOAMI_6500:
        _mpu_type = Invensense_MPU6500;
        return true;
    case MPU_WHOAMI_MPU9250:
    case MPU_WHOAMI_MPU9255:
        _mpu_type = Invensense_MPU9250;
        return true;
    case MPU_WHOAMI_20608:
        _mpu_type = Invensense_ICM20608;
        return true;
    case MPU_WHOAMI_20602:
        _mpu_type = Invensense_ICM20602;
        return true;
    }
    // not a value WHOAMI result
    return false;
}


bool AP_InertialSensor_Invensense::_hardware_init(void)
{
    if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    // setup for register checking
    _dev->setup_checked_registers(7, 20);
    
    // initially run the bus at low speed
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    if (!_check_whoami()) {
        _dev->get_semaphore()->give();
        return false;
    }

    // Chip reset
    uint8_t tries;
    for (tries = 0; tries < 5; tries++) {
        _last_stat_user_ctrl = _register_read(MPUREG_USER_CTRL);

        /* First disable the master I2C to avoid hanging the slaves on the
         * aulixiliar I2C bus - it will be enabled again if the AuxiliaryBus
         * is used */
        if (_last_stat_user_ctrl & BIT_USER_CTRL_I2C_MST_EN) {
            _last_stat_user_ctrl &= ~BIT_USER_CTRL_I2C_MST_EN;
            _register_write(MPUREG_USER_CTRL, _last_stat_user_ctrl);
            hal.scheduler->delay(10);
        }

        /* reset device */
        _register_write(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_DEVICE_RESET);
        hal.scheduler->delay(100);

        /* bus-dependent initialization */
        if (_dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
            /* Disable I2C bus if SPI selected (Recommended in Datasheet to be
             * done just after the device is reset) */
            _last_stat_user_ctrl |= BIT_USER_CTRL_I2C_IF_DIS;
            _register_write(MPUREG_USER_CTRL, _last_stat_user_ctrl);
        }

        /* bus-dependent initialization */
        if ((_dev->bus_type() == AP_HAL::Device::BUS_TYPE_I2C) && (_mpu_type == Invensense_MPU9250)) {
            /* Enable I2C bypass to access internal AK8963 */
            _register_write(MPUREG_INT_PIN_CFG, BIT_BYPASS_EN);
        }

        // Wake up device and select GyroZ clock. Note that the
        // Invensense starts up in sleep mode, and it can take some time
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
    }

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
    _dev->get_semaphore()->give();

    if (tries == 5) {
        hal.console->printf("Failed to boot Invensense 5 times\n");
        return false;
    }

	if (_mpu_type == Invensense_ICM20608 ||
        _mpu_type == Invensense_ICM20602) {
        // this avoids a sensor bug, see description above
		_register_write(MPUREG_ICM_UNDOC1, MPUREG_ICM_UNDOC1_VALUE, true);
	}
    
    return true;
}

AP_Invensense_AuxiliaryBusSlave::AP_Invensense_AuxiliaryBusSlave(AuxiliaryBus &bus, uint8_t addr,
                                                         uint8_t instance)
    : AuxiliaryBusSlave(bus, addr, instance)
    , _mpu_addr(MPUREG_I2C_SLV0_ADDR + _instance * 3)
    , _mpu_reg(_mpu_addr + 1)
    , _mpu_ctrl(_mpu_addr + 2)
    , _mpu_do(MPUREG_I2C_SLV0_DO + _instance)
{
}

int AP_Invensense_AuxiliaryBusSlave::_set_passthrough(uint8_t reg, uint8_t size,
                                                  uint8_t *out)
{
    auto &backend = AP_InertialSensor_Invensense::from(_bus.get_backend());
    uint8_t addr;

    /* Ensure the slave read/write is disabled before changing the registers */
    backend._register_write(_mpu_ctrl, 0);

    if (out) {
        backend._register_write(_mpu_do, *out);
        addr = _addr;
    } else {
        addr = _addr | BIT_READ_FLAG;
    }

    backend._register_write(_mpu_addr, addr);
    backend._register_write(_mpu_reg, reg);
    backend._register_write(_mpu_ctrl, BIT_I2C_SLVX_EN | size);

    return 0;
}

int AP_Invensense_AuxiliaryBusSlave::passthrough_read(uint8_t reg, uint8_t *buf,
                                                   uint8_t size)
{
    assert(buf);

    if (_registered) {
        hal.console->printf("Error: can't passthrough when slave is already configured\n");
        return -1;
    }

    int r = _set_passthrough(reg, size);
    if (r < 0) {
        return r;
    }

    /* wait the value to be read from the slave and read it back */
    hal.scheduler->delay(10);

    auto &backend = AP_InertialSensor_Invensense::from(_bus.get_backend());
    if (!backend._block_read(MPUREG_EXT_SENS_DATA_00 + _ext_sens_data, buf, size)) {
        return -1;
    }

    /* disable new reads */
    backend._register_write(_mpu_ctrl, 0);

    return size;
}

int AP_Invensense_AuxiliaryBusSlave::passthrough_write(uint8_t reg, uint8_t val)
{
    if (_registered) {
        hal.console->printf("Error: can't passthrough when slave is already configured\n");
        return -1;
    }

    int r = _set_passthrough(reg, 1, &val);
    if (r < 0) {
        return r;
    }

    /* wait the value to be written to the slave */
    hal.scheduler->delay(10);

    auto &backend = AP_InertialSensor_Invensense::from(_bus.get_backend());

    /* disable new writes */
    backend._register_write(_mpu_ctrl, 0);

    return 1;
}

int AP_Invensense_AuxiliaryBusSlave::read(uint8_t *buf)
{
    if (!_registered) {
        hal.console->printf("Error: can't read before configuring slave\n");
        return -1;
    }

    auto &backend = AP_InertialSensor_Invensense::from(_bus.get_backend());
    if (!backend._block_read(MPUREG_EXT_SENS_DATA_00 + _ext_sens_data, buf, _sample_size)) {
        return -1;
    }

    return _sample_size;
}

/* Invensense provides up to 5 slave devices, but the 5th is way too different to
 * configure and is seldom used */
AP_Invensense_AuxiliaryBus::AP_Invensense_AuxiliaryBus(AP_InertialSensor_Invensense &backend, uint32_t devid)
    : AuxiliaryBus(backend, 4, devid)
{
}

AP_HAL::Semaphore *AP_Invensense_AuxiliaryBus::get_semaphore()
{
    return static_cast<AP_InertialSensor_Invensense&>(_ins_backend)._dev->get_semaphore();
}

AuxiliaryBusSlave *AP_Invensense_AuxiliaryBus::_instantiate_slave(uint8_t addr, uint8_t instance)
{
    /* Enable slaves on Invensense if this is the first time */
    if (_ext_sens_data == 0) {
        _configure_slaves();
    }

    return new AP_Invensense_AuxiliaryBusSlave(*this, addr, instance);
}

void AP_Invensense_AuxiliaryBus::_configure_slaves()
{
    auto &backend = AP_InertialSensor_Invensense::from(_ins_backend);

    /* Enable the I2C master to slaves on the auxiliary I2C bus*/
    if (!(backend._last_stat_user_ctrl & BIT_USER_CTRL_I2C_MST_EN)) {
        backend._last_stat_user_ctrl |= BIT_USER_CTRL_I2C_MST_EN;
        backend._register_write(MPUREG_USER_CTRL, backend._last_stat_user_ctrl);
    }

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

int AP_Invensense_AuxiliaryBus::_configure_periodic_read(AuxiliaryBusSlave *slave,
                                                     uint8_t reg, uint8_t size)
{
    if (_ext_sens_data + size > MAX_EXT_SENS_DATA) {
        return -1;
    }

    AP_Invensense_AuxiliaryBusSlave *mpu_slave =
        static_cast<AP_Invensense_AuxiliaryBusSlave*>(slave);
    mpu_slave->_set_passthrough(reg, size);
    mpu_slave->_ext_sens_data = _ext_sens_data;
    _ext_sens_data += size;

    return 0;
}
