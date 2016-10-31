/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#include "AP_InertialSensor_MPU6000.h"

extern const AP_HAL::HAL& hal;

// MPU6000 accelerometer scaling
#define MPU6000_ACCEL_SCALE_1G    (GRAVITY_MSS / 4096.0f)

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
#define MPU6000_DRDY_PIN 70
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLE || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF
#include "../AP_HAL_Linux/GPIO.h"
#define MPU6000_DRDY_PIN BBB_P8_14
#endif
#endif

// MPU 6000 registers
#define MPUREG_XG_OFFS_TC                               0x00
#define MPUREG_YG_OFFS_TC                               0x01
#define MPUREG_ZG_OFFS_TC                               0x02
#define MPUREG_X_FINE_GAIN                              0x03
#define MPUREG_Y_FINE_GAIN                              0x04
#define MPUREG_Z_FINE_GAIN                              0x05
#define MPUREG_XA_OFFS_H                                0x06    // X axis accelerometer offset (high byte)
#define MPUREG_XA_OFFS_L                                0x07    // X axis accelerometer offset (low byte)
#define MPUREG_YA_OFFS_H                                0x08    // Y axis accelerometer offset (high byte)
#define MPUREG_YA_OFFS_L                                0x09    // Y axis accelerometer offset (low byte)
#define MPUREG_ZA_OFFS_H                                0x0A    // Z axis accelerometer offset (high byte)
#define MPUREG_ZA_OFFS_L                                0x0B    // Z axis accelerometer offset (low byte)
#define MPUREG_PRODUCT_ID                               0x0C    // Product ID Register
#define MPUREG_XG_OFFS_USRH                     0x13    // X axis gyro offset (high byte)
#define MPUREG_XG_OFFS_USRL                     0x14    // X axis gyro offset (low byte)
#define MPUREG_YG_OFFS_USRH                     0x15    // Y axis gyro offset (high byte)
#define MPUREG_YG_OFFS_USRL                     0x16    // Y axis gyro offset (low byte)
#define MPUREG_ZG_OFFS_USRH                     0x17    // Z axis gyro offset (high byte)
#define MPUREG_ZG_OFFS_USRL                     0x18    // Z axis gyro offset (low byte)
#define MPUREG_SMPLRT_DIV                               0x19    // sample rate.  Fsample= 1Khz/(<this value>+1) = 200Hz
#       define MPUREG_SMPLRT_1000HZ                             0x00
#       define MPUREG_SMPLRT_500HZ                              0x01
#       define MPUREG_SMPLRT_250HZ                              0x03
#       define MPUREG_SMPLRT_200HZ                              0x04
#       define MPUREG_SMPLRT_100HZ                              0x09
#       define MPUREG_SMPLRT_50HZ                               0x13
#define MPUREG_CONFIG                                           0x1A
#define MPUREG_GYRO_CONFIG                                      0x1B
// bit definitions for MPUREG_GYRO_CONFIG
#       define BITS_GYRO_FS_250DPS                              0x00
#       define BITS_GYRO_FS_500DPS                              0x08
#       define BITS_GYRO_FS_1000DPS                             0x10
#       define BITS_GYRO_FS_2000DPS                             0x18
#       define BITS_GYRO_FS_MASK                                0x18    // only bits 3 and 4 are used for gyro full scale so use this to mask off other bits
#       define BITS_GYRO_ZGYRO_SELFTEST                 0x20
#       define BITS_GYRO_YGYRO_SELFTEST                 0x40
#       define BITS_GYRO_XGYRO_SELFTEST                 0x80
#define MPUREG_ACCEL_CONFIG                             0x1C
#define MPUREG_MOT_THR                                  0x1F    // detection threshold for Motion interrupt generation.  Motion is detected when the absolute value of any of the accelerometer measurements exceeds this
#define MPUREG_MOT_DUR                                  0x20    // duration counter threshold for Motion interrupt generation. The duration counter ticks at 1 kHz, therefore MOT_DUR has a unit of 1 LSB = 1 ms
#define MPUREG_ZRMOT_THR                                0x21    // detection threshold for Zero Motion interrupt generation.
#define MPUREG_ZRMOT_DUR                                0x22    // duration counter threshold for Zero Motion interrupt generation. The duration counter ticks at 16 Hz, therefore ZRMOT_DUR has a unit of 1 LSB = 64 ms.
#define MPUREG_FIFO_EN                                  0x23
#define MPUREG_INT_PIN_CFG                              0x37
#       define BIT_INT_RD_CLEAR                                 0x10    // clear the interrupt when any read occurs
#       define BIT_LATCH_INT_EN                                 0x20    // latch data ready pin 
#define MPUREG_INT_ENABLE                               0x38
// bit definitions for MPUREG_INT_ENABLE
#       define BIT_RAW_RDY_EN                                   0x01
#       define BIT_DMP_INT_EN                                   0x02    // enabling this bit (DMP_INT_EN) also enables RAW_RDY_EN it seems
#       define BIT_UNKNOWN_INT_EN                               0x04
#       define BIT_I2C_MST_INT_EN                               0x08
#       define BIT_FIFO_OFLOW_EN                                0x10
#       define BIT_ZMOT_EN                                              0x20
#       define BIT_MOT_EN                                               0x40
#       define BIT_FF_EN                                                0x80
#define MPUREG_INT_STATUS                               0x3A
// bit definitions for MPUREG_INT_STATUS (same bit pattern as above because this register shows what interrupt actually fired)
#       define BIT_RAW_RDY_INT                                  0x01
#       define BIT_DMP_INT                                              0x02
#       define BIT_UNKNOWN_INT                                  0x04
#       define BIT_I2C_MST_INT                                  0x08
#       define BIT_FIFO_OFLOW_INT                               0x10
#       define BIT_ZMOT_INT                                             0x20
#       define BIT_MOT_INT                                              0x40
#       define BIT_FF_INT                                               0x80
#define MPUREG_ACCEL_XOUT_H                             0x3B
#define MPUREG_ACCEL_XOUT_L                             0x3C
#define MPUREG_ACCEL_YOUT_H                             0x3D
#define MPUREG_ACCEL_YOUT_L                             0x3E
#define MPUREG_ACCEL_ZOUT_H                             0x3F
#define MPUREG_ACCEL_ZOUT_L                             0x40
#define MPUREG_TEMP_OUT_H                               0x41
#define MPUREG_TEMP_OUT_L                               0x42
#define MPUREG_GYRO_XOUT_H                              0x43
#define MPUREG_GYRO_XOUT_L                              0x44
#define MPUREG_GYRO_YOUT_H                              0x45
#define MPUREG_GYRO_YOUT_L                              0x46
#define MPUREG_GYRO_ZOUT_H                              0x47
#define MPUREG_GYRO_ZOUT_L                              0x48
#define MPUREG_USER_CTRL                                0x6A
// bit definitions for MPUREG_USER_CTRL
#       define BIT_USER_CTRL_SIG_COND_RESET             0x01            // resets signal paths and results registers for all sensors (gyros, accel, temp)
#       define BIT_USER_CTRL_I2C_MST_RESET              0x02            // reset I2C Master (only applicable if I2C_MST_EN bit is set)
#       define BIT_USER_CTRL_FIFO_RESET                 0x04            // Reset (i.e. clear) FIFO buffer
#       define BIT_USER_CTRL_DMP_RESET                  0x08            // Reset DMP
#       define BIT_USER_CTRL_I2C_IF_DIS                 0x10            // Disable primary I2C interface and enable hal.spi->interface
#       define BIT_USER_CTRL_I2C_MST_EN                 0x20            // Enable MPU to act as the I2C Master to external slave sensors
#       define BIT_USER_CTRL_FIFO_EN                    0x40            // Enable FIFO operations
#       define BIT_USER_CTRL_DMP_EN                             0x80            // Enable DMP operations
#define MPUREG_PWR_MGMT_1                               0x6B
#       define BIT_PWR_MGMT_1_CLK_INTERNAL              0x00            // clock set to internal 8Mhz oscillator
#       define BIT_PWR_MGMT_1_CLK_XGYRO                 0x01            // PLL with X axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_YGYRO                 0x02            // PLL with Y axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_ZGYRO                 0x03            // PLL with Z axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_EXT32KHZ              0x04            // PLL with external 32.768kHz reference
#       define BIT_PWR_MGMT_1_CLK_EXT19MHZ              0x05            // PLL with external 19.2MHz reference
#       define BIT_PWR_MGMT_1_CLK_STOP                  0x07            // Stops the clock and keeps the timing generator in reset
#       define BIT_PWR_MGMT_1_TEMP_DIS                  0x08            // disable temperature sensor
#       define BIT_PWR_MGMT_1_CYCLE                             0x20            // put sensor into cycle mode.  cycles between sleep mode and waking up to take a single sample of data from active sensors at a rate determined by LP_WAKE_CTRL
#       define BIT_PWR_MGMT_1_SLEEP                             0x40            // put sensor into low power sleep mode
#       define BIT_PWR_MGMT_1_DEVICE_RESET              0x80            // reset entire device
#define MPUREG_PWR_MGMT_2                               0x6C            // allows the user to configure the frequency of wake-ups in Accelerometer Only Low Power Mode
#define MPUREG_BANK_SEL                                 0x6D            // DMP bank selection register (used to indirectly access DMP registers)
#define MPUREG_MEM_START_ADDR                   0x6E            // DMP memory start address (used to indirectly write to dmp memory)
#define MPUREG_MEM_R_W                                  0x6F            // DMP related register
#define MPUREG_DMP_CFG_1                                0x70            // DMP related register
#define MPUREG_DMP_CFG_2                                0x71            // DMP related register
#define MPUREG_FIFO_COUNTH                              0x72
#define MPUREG_FIFO_COUNTL                              0x73
#define MPUREG_FIFO_R_W                                 0x74
#define MPUREG_WHOAMI                                   0x75


// Configuration bits MPU 3000 and MPU 6000 (not revised)?
#define BITS_DLPF_CFG_256HZ_NOLPF2              0x00
#define BITS_DLPF_CFG_188HZ                             0x01
#define BITS_DLPF_CFG_98HZ                              0x02
#define BITS_DLPF_CFG_42HZ                              0x03
#define BITS_DLPF_CFG_20HZ                              0x04
#define BITS_DLPF_CFG_10HZ                              0x05
#define BITS_DLPF_CFG_5HZ                               0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF              0x07
#define BITS_DLPF_CFG_MASK                              0x07

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


/*
 *  RM-MPU-6000A-00.pdf, page 33, section 4.25 lists LSB sensitivity of
 *  gyro as 16.4 LSB/DPS at scale factor of +/- 2000dps (FS_SEL==3)
 */
const float AP_InertialSensor_MPU6000::_gyro_scale = (0.0174532f / 16.4f);

/*
 *  RM-MPU-6000A-00.pdf, page 31, section 4.23 lists LSB sensitivity of
 *  accel as 4096 LSB/mg at scale factor of +/- 8g (AFS_SEL==2)
 *
 *  See note below about accel scaling of engineering sample MPU6k
 *  variants however
 */

AP_InertialSensor_MPU6000::AP_InertialSensor_MPU6000(AP_InertialSensor &imu) :
    AP_InertialSensor_Backend(imu),
    _drdy_pin(NULL),
    _spi(NULL),
    _spi_sem(NULL),
    _last_filter_hz(0),
    _error_count(0),
#if MPU6000_FAST_SAMPLING
    _accel_filter_x(1000, 15),
    _accel_filter_y(1000, 15),
    _accel_filter_z(1000, 15),
    _gyro_filter_x(1000, 15),
    _gyro_filter_y(1000, 15),
    _gyro_filter_z(1000, 15),    
#else
    _sample_count(0),
    _accel_sum(),
    _gyro_sum(),
#endif
    _sum_count(0)
{
}

/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_MPU6000::detect(AP_InertialSensor &_imu)
{
    AP_InertialSensor_MPU6000 *sensor = new AP_InertialSensor_MPU6000(_imu);
    if (sensor == NULL) {
        return NULL;
    }
    if (!sensor->_init_sensor()) {
        delete sensor;
        return NULL;
    }

    return sensor;
}

/*
  initialise the sensor
 */
bool AP_InertialSensor_MPU6000::_init_sensor(void)
{
    _spi = hal.spi->device(AP_HAL::SPIDevice_MPU6000);
    _spi_sem = _spi->get_semaphore();

#ifdef MPU6000_DRDY_PIN
    _drdy_pin = hal.gpio->channel(MPU6000_DRDY_PIN);
    _drdy_pin->mode(HAL_GPIO_INPUT);
#endif

    hal.scheduler->suspend_timer_procs();

    uint8_t tries = 0;
    do {
        bool success = _hardware_init();
        if (success) {
            hal.scheduler->delay(5+2);
            if (!_spi_sem->take(100)) {
                return false;
            }
            if (_data_ready()) {
                _spi_sem->give();
                break;
            } else {
                return false;
            }
            _spi_sem->give();
        }
        if (tries++ > 5) {
            hal.console->print_P(PSTR("failed to boot MPU6000 5 times")); 
            return false;
        }
    } while (1);

    // grab the used instances
    _gyro_instance = _imu.register_gyro();
    _accel_instance = _imu.register_accel();

    hal.scheduler->resume_timer_procs();
    
    // start the timer process to read samples
    hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&AP_InertialSensor_MPU6000::_poll_data));

#if MPU6000_DEBUG
    _dump_registers();
#endif

    return true;
}


/*
  process any 
 */
bool AP_InertialSensor_MPU6000::update( void )
{    
#if !MPU6000_FAST_SAMPLING
    if (_sum_count < _sample_count) {
        // we don't have enough samples yet
        return false;
    }
#endif

    // we have a full set of samples
    uint16_t num_samples;
    Vector3f accel, gyro;

    hal.scheduler->suspend_timer_procs();
#if MPU6000_FAST_SAMPLING
    gyro = _gyro_filtered;
    accel = _accel_filtered;
    num_samples = 1;
#else
    gyro(_gyro_sum.x, _gyro_sum.y, _gyro_sum.z);
    accel(_accel_sum.x, _accel_sum.y, _accel_sum.z);
    num_samples = _sum_count;
    _accel_sum.zero();
    _gyro_sum.zero();
#endif
    _sum_count = 0;
    hal.scheduler->resume_timer_procs();

    gyro *= _gyro_scale / num_samples;
    _rotate_and_offset_gyro(_gyro_instance, gyro);

    accel *= MPU6000_ACCEL_SCALE_1G / num_samples;
    _rotate_and_offset_accel(_accel_instance, accel);

    if (_last_filter_hz != _imu.get_filter()) {
        if (_spi_sem->take(10)) {
            _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_LOW);
            _set_filter_register(_imu.get_filter());
            _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_HIGH);
            _spi_sem->give();
        }
    }

    return true;
}

/*================ HARDWARE FUNCTIONS ==================== */

/**
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

/**
 * Timer process to poll for new data from the MPU6000.
 */
void AP_InertialSensor_MPU6000::_poll_data(void)
{
    if (!_spi_sem->take_nonblocking()) {
        return;
    }   
    if (_data_ready()) {
        _read_data_transaction(); 
    }
    _spi_sem->give();
}


void AP_InertialSensor_MPU6000::_read_data_transaction() {
    /* one resister address followed by seven 2-byte registers */
    struct PACKED {
        uint8_t cmd;
        uint8_t int_status;
        uint8_t v[14];
    } rx, tx = { cmd : MPUREG_INT_STATUS | 0x80, };
    
    _spi->transaction((const uint8_t *)&tx, (uint8_t *)&rx, sizeof(rx));

    /*
      detect a bad SPI bus transaction by looking for all 14 bytes
      zero, or the wrong INT_STATUS register value. This is used to
      detect a too high SPI bus speed.
     */
    uint8_t i;
    for (i=0; i<14; i++) {
        if (rx.v[i] != 0) break;
    }
    if ((rx.int_status&~0x6) != (_drdy_pin==NULL?0:BIT_RAW_RDY_INT) || i == 14) {
        // likely a bad bus transaction
        if (++_error_count > 4) {
            _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_LOW);
        }
    }

#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx] << 8) | v[2*idx+1]))
#if MPU6000_FAST_SAMPLING
    _accel_filtered = Vector3f(_accel_filter_x.apply(int16_val(rx.v, 1)), 
                               _accel_filter_y.apply(int16_val(rx.v, 0)), 
                               _accel_filter_z.apply(-int16_val(rx.v, 2)));

    _gyro_filtered = Vector3f(_gyro_filter_x.apply(int16_val(rx.v, 5)), 
                              _gyro_filter_y.apply(int16_val(rx.v, 4)), 
                              _gyro_filter_z.apply(-int16_val(rx.v, 6)));
#else
    _accel_sum.x += int16_val(rx.v, 1);
    _accel_sum.y += int16_val(rx.v, 0);
    _accel_sum.z -= int16_val(rx.v, 2);
    _gyro_sum.x  += int16_val(rx.v, 5);
    _gyro_sum.y  += int16_val(rx.v, 4);
    _gyro_sum.z  -= int16_val(rx.v, 6);
#endif
    _sum_count++;

#if !MPU6000_FAST_SAMPLING
    if (_sum_count == 0) {
        // rollover - v unlikely
        _accel_sum.zero();
        _gyro_sum.zero();
    }
#endif
}

uint8_t AP_InertialSensor_MPU6000::_register_read( uint8_t reg )
{
    uint8_t addr = reg | 0x80; // Set most significant bit

    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = addr;
    tx[1] = 0;
    _spi->transaction(tx, rx, 2);

    return rx[1];
}

void AP_InertialSensor_MPU6000::_register_write(uint8_t reg, uint8_t val)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = reg;
    tx[1] = val;
    _spi->transaction(tx, rx, 2);
}

/*
  useful when debugging SPI bus errors
 */
void AP_InertialSensor_MPU6000::_register_write_check(uint8_t reg, uint8_t val)
{
    uint8_t readed;
    _register_write(reg, val);
    readed = _register_read(reg);
    if (readed != val){
	hal.console->printf_P(PSTR("Values doesn't match; written: %02x; read: %02x "), val, readed);
    }
#if MPU6000_DEBUG
    hal.console->printf_P(PSTR("Values written: %02x; readed: %02x "), val, readed);
#endif
}

/*
  set the DLPF filter frequency. Assumes caller has taken semaphore
 */
void AP_InertialSensor_MPU6000::_set_filter_register(uint8_t filter_hz)
{
    if (filter_hz == 0) {
        filter_hz = _default_filter();
    }
    uint8_t filter;
    // choose filtering frequency
    if (filter_hz <= 5) {
        filter = BITS_DLPF_CFG_5HZ;
    } else if (filter_hz <= 10) {
        filter = BITS_DLPF_CFG_10HZ;
    } else if (filter_hz <= 20) {
        filter = BITS_DLPF_CFG_20HZ;
    } else if (filter_hz <= 42) {
        filter = BITS_DLPF_CFG_42HZ;
    } else {
        filter = BITS_DLPF_CFG_98HZ;
    }
    _last_filter_hz = filter_hz;
    _register_write(MPUREG_CONFIG, filter);
}


bool AP_InertialSensor_MPU6000::_hardware_init(void)
{
    if (!_spi_sem->take(100)) {
        hal.scheduler->panic(PSTR("MPU6000: Unable to get semaphore"));
    }

    // initially run the bus at low speed (500kHz on APM2)
    _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_LOW);

    // Chip reset
    uint8_t tries;
    for (tries = 0; tries<5; tries++) {
        _register_write(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_DEVICE_RESET);
        hal.scheduler->delay(100);

        // Wake up device and select GyroZ clock. Note that the
        // MPU6000 starts up in sleep mode, and it can take some time
        // for it to come out of sleep
        _register_write(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_CLK_ZGYRO);
        hal.scheduler->delay(5);

        // check it has woken up
        if (_register_read(MPUREG_PWR_MGMT_1) == BIT_PWR_MGMT_1_CLK_ZGYRO) {
            break;
        }
#if MPU6000_DEBUG
        _dump_registers();
#endif
    }
    if (tries == 5) {
        hal.console->println_P(PSTR("Failed to boot MPU6000 5 times"));
        _spi_sem->give();
        return false;
    }

    _register_write(MPUREG_PWR_MGMT_2, 0x00);            // only used for wake-up in accelerometer only low power mode
    hal.scheduler->delay(1);

    // Disable I2C bus (recommended on datasheet)
    _register_write(MPUREG_USER_CTRL, BIT_USER_CTRL_I2C_IF_DIS);
    hal.scheduler->delay(1);

#if MPU6000_FAST_SAMPLING
    _sample_count = 1;
#else
    // sample rate and filtering
    // to minimise the effects of aliasing we choose a filter
    // that is less than half of the sample rate
    switch (_imu.get_sample_rate()) {
    case AP_InertialSensor::RATE_50HZ:
        // this is used for plane and rover, where noise resistance is
        // more important than update rate. Tests on an aerobatic plane
        // show that 10Hz is fine, and makes it very noise resistant
        _sample_count = 4;
        break;
    case AP_InertialSensor::RATE_100HZ:
        _sample_count = 2;
        break;
    case AP_InertialSensor::RATE_200HZ:
        _sample_count = 1;
        break;
    default:
        return false;
    }
#endif

    _set_filter_register(_imu.get_filter());

#if MPU6000_FAST_SAMPLING
    // set sample rate to 1000Hz and apply a software filter
    _register_write(MPUREG_SMPLRT_DIV, MPUREG_SMPLRT_1000HZ);
#else
    // set sample rate to 200Hz, and use _sample_divider to give
    // the requested rate to the application
    _register_write(MPUREG_SMPLRT_DIV, MPUREG_SMPLRT_200HZ);
#endif
    hal.scheduler->delay(1);

    _register_write(MPUREG_GYRO_CONFIG, BITS_GYRO_FS_2000DPS);  // Gyro scale 2000º/s
    hal.scheduler->delay(1);

    // read the product ID rev c has 1/2 the sensitivity of rev d
    _product_id = _register_read(MPUREG_PRODUCT_ID);
    //Serial.printf("Product_ID= 0x%x\n", (unsigned) _mpu6000_product_id);

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

    // now that we have initialised, we set the SPI bus speed to high
    // (8MHz on APM2)
    _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_HIGH);

    _spi_sem->give();

    return true;
}

#if MPU6000_DEBUG
// dump all config registers - used for debug
void AP_InertialSensor_MPU6000::_dump_registers(void)
{
    hal.console->println_P(PSTR("MPU6000 registers"));
    if (_spi_sem->take(100)) {
        for (uint8_t reg=MPUREG_PRODUCT_ID; reg<=108; reg++) {
            uint8_t v = _register_read(reg);
            hal.console->printf_P(PSTR("%02x:%02x "), (unsigned)reg, (unsigned)v);
            if ((reg - (MPUREG_PRODUCT_ID-1)) % 16 == 0) {
                hal.console->println();
            }
        }
        hal.console->println();
        _spi_sem->give();
    }
}
#endif
