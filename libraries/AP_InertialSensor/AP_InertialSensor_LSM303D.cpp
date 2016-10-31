/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#if NOT_YET

/****************************************************************************
 *
 *	 Coded by Víctor Mayoral Vilches <v.mayoralv@gmail.com> using 
 *	 lsm3030d.cpp <https://github.com/diydrones/PX4Firmware> from the PX4 Development Team.
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <AP_HAL.h>
#include "AP_InertialSensor_LSM303D.h"

extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
	#define LSM303D_DRDY_PIN 70
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
	#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLE || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF
		#include "../AP_HAL_Linux/GPIO.h"
		#define LSM303D_DRDY_X_PIN BBB_P8_8  // ACCEL DRDY
        #define LSM303D_DRDY_M_PIN BBB_P8_10  // MAGNETOMETER DRDY
	#endif
#endif

/* SPI protocol address bits */
#define DIR_READ                (1<<7)
#define DIR_WRITE               (0<<7)
#define ADDR_INCREMENT          (1<<6)

/* register addresses: A: accel, M: mag, T: temp */
#define ADDR_WHO_AM_I           0x0F
#define WHO_I_AM            0x49

#define ADDR_OUT_TEMP_L         0x05
#define ADDR_OUT_TEMP_H         0x06
#define ADDR_STATUS_M           0x07
#define ADDR_OUT_X_L_M              0x08
#define ADDR_OUT_X_H_M              0x09
#define ADDR_OUT_Y_L_M              0x0A
#define ADDR_OUT_Y_H_M          0x0B
#define ADDR_OUT_Z_L_M          0x0C
#define ADDR_OUT_Z_H_M          0x0D

#define ADDR_INT_CTRL_M         0x12
#define ADDR_INT_SRC_M          0x13
#define ADDR_REFERENCE_X        0x1c
#define ADDR_REFERENCE_Y        0x1d
#define ADDR_REFERENCE_Z        0x1e

#define ADDR_STATUS_A           0x27
#define ADDR_OUT_X_L_A          0x28
#define ADDR_OUT_X_H_A          0x29
#define ADDR_OUT_Y_L_A          0x2A
#define ADDR_OUT_Y_H_A          0x2B
#define ADDR_OUT_Z_L_A          0x2C
#define ADDR_OUT_Z_H_A          0x2D

#define ADDR_CTRL_REG0          0x1F
#define ADDR_CTRL_REG1          0x20
#define ADDR_CTRL_REG2          0x21
#define ADDR_CTRL_REG3          0x22
#define ADDR_CTRL_REG4          0x23
#define ADDR_CTRL_REG5          0x24
#define ADDR_CTRL_REG6          0x25
#define ADDR_CTRL_REG7          0x26

#define ADDR_FIFO_CTRL          0x2e
#define ADDR_FIFO_SRC           0x2f

#define ADDR_IG_CFG1            0x30
#define ADDR_IG_SRC1            0x31
#define ADDR_IG_THS1            0x32
#define ADDR_IG_DUR1            0x33
#define ADDR_IG_CFG2            0x34
#define ADDR_IG_SRC2            0x35
#define ADDR_IG_THS2            0x36
#define ADDR_IG_DUR2            0x37
#define ADDR_CLICK_CFG          0x38
#define ADDR_CLICK_SRC          0x39
#define ADDR_CLICK_THS          0x3a
#define ADDR_TIME_LIMIT         0x3b
#define ADDR_TIME_LATENCY       0x3c
#define ADDR_TIME_WINDOW        0x3d
#define ADDR_ACT_THS            0x3e
#define ADDR_ACT_DUR            0x3f

#define REG1_RATE_BITS_A        ((1<<7) | (1<<6) | (1<<5) | (1<<4))
#define REG1_POWERDOWN_A        ((0<<7) | (0<<6) | (0<<5) | (0<<4))
#define REG1_RATE_3_125HZ_A     ((0<<7) | (0<<6) | (0<<5) | (1<<4))
#define REG1_RATE_6_25HZ_A      ((0<<7) | (0<<6) | (1<<5) | (0<<4))
#define REG1_RATE_12_5HZ_A      ((0<<7) | (0<<6) | (1<<5) | (1<<4))
#define REG1_RATE_25HZ_A        ((0<<7) | (1<<6) | (0<<5) | (0<<4))
#define REG1_RATE_50HZ_A        ((0<<7) | (1<<6) | (0<<5) | (1<<4))
#define REG1_RATE_100HZ_A       ((0<<7) | (1<<6) | (1<<5) | (0<<4))
#define REG1_RATE_200HZ_A       ((0<<7) | (1<<6) | (1<<5) | (1<<4))
#define REG1_RATE_400HZ_A       ((1<<7) | (0<<6) | (0<<5) | (0<<4))
#define REG1_RATE_800HZ_A       ((1<<7) | (0<<6) | (0<<5) | (1<<4))
#define REG1_RATE_1600HZ_A      ((1<<7) | (0<<6) | (1<<5) | (0<<4))

#define REG1_BDU_UPDATE         (1<<3)
#define REG1_Z_ENABLE_A         (1<<2)
#define REG1_Y_ENABLE_A         (1<<1)
#define REG1_X_ENABLE_A         (1<<0)

#define REG2_ANTIALIAS_FILTER_BW_BITS_A ((1<<7) | (1<<6))
#define REG2_AA_FILTER_BW_773HZ_A       ((0<<7) | (0<<6))
#define REG2_AA_FILTER_BW_194HZ_A       ((0<<7) | (1<<6))
#define REG2_AA_FILTER_BW_362HZ_A       ((1<<7) | (0<<6))
#define REG2_AA_FILTER_BW_50HZ_A        ((1<<7) | (1<<6))

#define REG2_FULL_SCALE_BITS_A  ((1<<5) | (1<<4) | (1<<3))
#define REG2_FULL_SCALE_2G_A    ((0<<5) | (0<<4) | (0<<3))
#define REG2_FULL_SCALE_4G_A    ((0<<5) | (0<<4) | (1<<3))
#define REG2_FULL_SCALE_6G_A    ((0<<5) | (1<<4) | (0<<3))
#define REG2_FULL_SCALE_8G_A    ((0<<5) | (1<<4) | (1<<3))
#define REG2_FULL_SCALE_16G_A   ((1<<5) | (0<<4) | (0<<3))

#define REG5_ENABLE_T           (1<<7)

#define REG5_RES_HIGH_M         ((1<<6) | (1<<5))
#define REG5_RES_LOW_M          ((0<<6) | (0<<5))

#define REG5_RATE_BITS_M        ((1<<4) | (1<<3) | (1<<2))
#define REG5_RATE_3_125HZ_M     ((0<<4) | (0<<3) | (0<<2))
#define REG5_RATE_6_25HZ_M      ((0<<4) | (0<<3) | (1<<2))
#define REG5_RATE_12_5HZ_M      ((0<<4) | (1<<3) | (0<<2))
#define REG5_RATE_25HZ_M        ((0<<4) | (1<<3) | (1<<2))
#define REG5_RATE_50HZ_M        ((1<<4) | (0<<3) | (0<<2))
#define REG5_RATE_100HZ_M       ((1<<4) | (0<<3) | (1<<2))
#define REG5_RATE_DO_NOT_USE_M  ((1<<4) | (1<<3) | (0<<2))

#define REG6_FULL_SCALE_BITS_M  ((1<<6) | (1<<5))
#define REG6_FULL_SCALE_2GA_M   ((0<<6) | (0<<5))
#define REG6_FULL_SCALE_4GA_M   ((0<<6) | (1<<5))
#define REG6_FULL_SCALE_8GA_M   ((1<<6) | (0<<5))
#define REG6_FULL_SCALE_12GA_M  ((1<<6) | (1<<5))

#define REG7_CONT_MODE_M        ((0<<1) | (0<<0))


#define INT_CTRL_M              0x12
#define INT_SRC_M               0x13

/* default values for this device */
#define LSM303D_ACCEL_DEFAULT_RANGE_G           8
#define LSM303D_ACCEL_DEFAULT_RATE          800
#define LSM303D_ACCEL_DEFAULT_ONCHIP_FILTER_FREQ    50
#define LSM303D_ACCEL_DEFAULT_DRIVER_FILTER_FREQ    30

#define LSM303D_MAG_DEFAULT_RANGE_GA            2
#define LSM303D_MAG_DEFAULT_RATE            100

#define LSM303D_ONE_G                   9.80665f


AP_InertialSensor_LSM303D::AP_InertialSensor_LSM303D() : 
    AP_InertialSensor(),
    _drdy_pin_x(NULL),
    _drdy_pin_m(NULL),
    _initialised(false),
    _LSM303D_product_id(AP_PRODUCT_ID_NONE)
{
}

uint16_t AP_InertialSensor_LSM303D::_init_sensor( Sample_rate sample_rate )
{
    if (_initialised) return _LSM303D_product_id;
    _initialised = true;

    _spi = hal.spi->device(AP_HAL::SPIDevice_LSM303D);
    _spi_sem = _spi->get_semaphore();

// This device has mag and accel
#ifdef LSM303D_DRDY_X_PIN
    _drdy_pin_x = hal.gpio->channel(LSM303D_DRDY_X_PIN);
    _drdy_pin_x->mode(HAL_GPIO_INPUT);
#endif

#ifdef LSM303D_DRDY_M_PIN
    _drdy_pin_m = hal.gpio->channel(LSM303D_DRDY_M_PIN);
    _drdy_pin_m->mode(HAL_GPIO_INPUT);
#endif

    hal.scheduler->suspend_timer_procs();

    // Test WHOAMI
    uint8_t whoami = _register_read(ADDR_WHO_AM_I);
    if (whoami != WHO_I_AM) {
        // TODO: we should probably accept multiple chip
        // revisions. This is the one on the PXF
        hal.console->printf("LSM303D: unexpected WHOAMI 0x%x\n", (unsigned)whoami);
        hal.scheduler->panic(PSTR("LSM303D: bad WHOAMI"));
    }

    uint8_t tries = 0;
    do {
        bool success = _hardware_init(sample_rate);
        if (success) {
            hal.scheduler->delay(5+2);
            if (!_spi_sem->take(100)) {
                hal.scheduler->panic(PSTR("LSM303D: Unable to get semaphore"));
            }
            if (_data_ready()) {
                _spi_sem->give();
                break;
            } else {
                hal.console->println_P(
                        PSTR("LSM303D startup failed: no data ready"));
            }
            _spi_sem->give();
        }
        if (tries++ > 5) {
            hal.scheduler->panic(PSTR("PANIC: failed to boot LSM303D 5 times")); 
        }
    } while (1);

    hal.scheduler->resume_timer_procs();
    

    /* read the first lot of data.
     * _read_data_transaction requires the spi semaphore to be taken by
     * its caller. */
    _last_sample_time_micros = hal.scheduler->micros();
    hal.scheduler->delay(10);
    if (_spi_sem->take(100)) {
        _read_data_transaction();
        _spi_sem->give();
    }

    // start the timer process to read samples
    hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&AP_InertialSensor_LSM303D::_poll_data));

#if LSM303D_DEBUG
    _dump_registers();
#endif
    return _LSM303D_product_id;
}

/*================ AP_INERTIALSENSOR PUBLIC INTERFACE ==================== */

bool AP_InertialSensor_LSM303D::wait_for_sample(uint16_t timeout_ms)
{
    if (_sample_available()) {
        return true;
    }
    uint32_t start = hal.scheduler->millis();
    while ((hal.scheduler->millis() - start) < timeout_ms) {
        hal.scheduler->delay_microseconds(100);
        if (_sample_available()) {
            return true;
        }
    }
    return false;
}

bool AP_InertialSensor_LSM303D::update( void )
{
    // wait for at least 1 sample
    if (!wait_for_sample(1000)) {
        return false;
    }

    // disable timer procs for mininum time
    hal.scheduler->suspend_timer_procs();

    _accel[0]  = Vector3f(_accel_sum.x, _accel_sum.y, _accel_sum.z);
    // _mag[0]  = Vector3f(_mag_sum.x, _mag_sum.y, _mag_sum.z);

    _num_samples = _sum_count;
    _accel_sum.zero();
    _mag_sum.zero();
    _sum_count = 0;
    hal.scheduler->resume_timer_procs();

    _accel[0].rotate(_board_orientation);
    // TODO change this for the corresponding value
    // _accel[0] *= MPU6000_ACCEL_SCALE_1G / _num_samples;

    // Vector3f accel_scale = _accel_scale[0].get();
    // _accel[0].x *= accel_scale.x;
    // _accel[0].y *= accel_scale.y;
    // _accel[0].z *= accel_scale.z;
    // _accel[0] -= _accel_offset[0];

    // TODO similarly put mag values in _mag and scale them

    // if (_last_filter_hz != _LSM303D_filter) {
    //     if (_spi_sem->take(10)) {
    //         _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_LOW);
    //         _set_filter_register(_LSM303D_filter, 0);
    //         _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_HIGH);
    //         _error_count = 0;
    //         _spi_sem->give();
    //     }
    // }

    return true;
}

/*================ HARDWARE FUNCTIONS ==================== */

/**
 * Return true if the LSM303D has new data available for both the mag and the accels.
 *
 * We use the data ready pin if it is available.  Otherwise, read the
 * status register.
 */
bool AP_InertialSensor_LSM303D::_data_ready()
{
    if (_drdy_pin_m && _drdy_pin_x) {
        return (_drdy_pin_m->read() && _drdy_pin_x->read()) != 0;
    }
    // TODO: read status register
    return false;
}

/**
 * Timer process to poll for new data from the LSM303D.
 */
void AP_InertialSensor_LSM303D::_poll_data(void)
{
    if (hal.scheduler->in_timerprocess()) {
        if (!_spi_sem->take_nonblocking()) {
            /*
              the semaphore being busy is an expected condition when the
              mainline code is calling wait_for_sample() which will
              grab the semaphore. We return now and rely on the mainline
              code grabbing the latest sample.
            */
            return;
        }   
        if (_data_ready()) {
            _last_sample_time_micros = hal.scheduler->micros();
            _read_data_transaction(); 
        }
        _spi_sem->give();
    } else {
        /* Synchronous read - take semaphore */
        if (_spi_sem->take(10)) {
            if (_data_ready()) {
                _last_sample_time_micros = hal.scheduler->micros();
                _read_data_transaction(); 
            }
            _spi_sem->give();
        } else {
            hal.scheduler->panic(
                PSTR("PANIC: AP_InertialSensor_LSM303D::_poll_data "
                     "failed to take SPI semaphore synchronously"));
        }
    }
}

void AP_InertialSensor_LSM303D::_read_data_transaction_accel() 
{

    if (_register_read(ADDR_CTRL_REG1) != _reg1_expected) {
            hal.console->println_P(
                        PSTR("LSM303D _read_data_transaction_accel: _reg1_expected unexpected"));
        // reset();
        return;
    }

    struct {
        uint8_t     cmd;
        uint8_t     status;
        int16_t     x;
        int16_t     y;
        int16_t     z;
    } raw_accel_report;

    /* fetch data from the sensor */
    memset(&raw_accel_report, 0, sizeof(raw_accel_report));
    raw_accel_report.cmd = ADDR_STATUS_A | DIR_READ | ADDR_INCREMENT;
    _spi->transaction((uint8_t *)&raw_accel_report, (uint8_t *)&raw_accel_report, sizeof(raw_accel_report));

    _accel_sum.x  += raw_accel_report.x;
    _accel_sum.y  += raw_accel_report.y;
    _accel_sum.z  += raw_accel_report.z;
}

void AP_InertialSensor_LSM303D::_read_data_transaction_mag() {
    if (_register_read(ADDR_CTRL_REG7) != _reg7_expected) {
            hal.console->println_P(
                        PSTR("LSM303D _read_data_transaction_accel: _reg7_expected unexpected"));
        // reset();
        return;
    }

    struct {
        uint8_t     cmd;
        uint8_t     status;
        int16_t     x;
        int16_t     y;
        int16_t     z;
    } raw_mag_report;

    /* fetch data from the sensor */
    memset(&raw_mag_report, 0, sizeof(raw_mag_report));
    raw_mag_report.cmd = ADDR_STATUS_M | DIR_READ | ADDR_INCREMENT;
    _spi->transaction((uint8_t *)&raw_mag_report, (uint8_t *)&raw_mag_report, sizeof(raw_mag_report));

    _mag_sum.x = raw_mag_report.x;
    _mag_sum.y = raw_mag_report.y;
    _mag_sum.z = raw_mag_report.z;
}

void AP_InertialSensor_LSM303D::_read_data_transaction() {
    
    _read_data_transaction_accel();
    _read_data_transaction_mag();
    _sum_count++;

    if (_sum_count == 0) {
        // rollover - v unlikely
        _accel_sum.zero();
        _mag_sum.zero();
    }
}

uint8_t AP_InertialSensor_LSM303D::_register_read( uint8_t reg )
{
    uint8_t addr = reg | 0x80; // Set most significant bit

    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = addr;
    tx[1] = 0;
    _spi->transaction(tx, rx, 2);

    return rx[1];
}

void AP_InertialSensor_LSM303D::_register_write(uint8_t reg, uint8_t val)
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
void AP_InertialSensor_LSM303D::_register_write_check(uint8_t reg, uint8_t val)
{
    uint8_t readed;
    _register_write(reg, val);
    readed = _register_read(reg);
    if (readed != val){
	hal.console->printf_P(PSTR("Values doesn't match; written: %02x; read: %02x "), val, readed);
    }
#if LSM303D_DEBUG
    hal.console->printf_P(PSTR("Values written: %02x; readed: %02x "), val, readed);
#endif
}

void AP_InertialSensor_LSM303D::_register_modify(uint8_t reg, uint8_t clearbits, uint8_t setbits)
{
    uint8_t val;

    val = _register_read(reg);
    val &= ~clearbits;
    val |= setbits;
    _register_write(reg, val);
}


/*
  set the DLPF filter frequency. Assumes caller has taken semaphore
  TODO needs to be changed according to LSM303D needs
 */
// void AP_InertialSensor_LSM303D::_set_filter_register(uint8_t filter_hz, uint8_t default_filter)
// {
//     uint8_t filter = default_filter;
//     // choose filtering frequency
//     switch (filter_hz) {
//     case 5:
//         filter = BITS_DLPF_CFG_5HZ;
//         break;
//     case 10:
//         filter = BITS_DLPF_CFG_10HZ;
//         break;
//     case 20:
//         filter = BITS_DLPF_CFG_20HZ;
//         break;
//     case 42:
//         filter = BITS_DLPF_CFG_42HZ;
//         break;
//     case 98:
//         filter = BITS_DLPF_CFG_98HZ;
//         break;
//     }

//     if (filter != 0) {
//         _last_filter_hz = filter_hz;
//         _register_write(MPUREG_CONFIG, filter);
//     }
// }

void AP_InertialSensor_LSM303D::disable_i2c(void)
{
    uint8_t a = _register_read(0x02);
    _register_write(0x02, (0x10 | a));
    a = _register_read(0x02);
    _register_write(0x02, (0xF7 & a));
    a = _register_read(0x15);
    _register_write(0x15, (0x80 | a));
    a = _register_read(0x02);
    _register_write(0x02, (0xE7 & a));
}

uint8_t AP_InertialSensor_LSM303D::accel_set_range(uint8_t max_g)
{
    uint8_t setbits = 0;
    uint8_t clearbits = REG2_FULL_SCALE_BITS_A;
    float new_scale_g_digit = 0.0f;

    if (max_g == 0)
        max_g = 16;

    if (max_g <= 2) {
        _accel_range_m_s2 = 2.0f*LSM303D_ONE_G;
        setbits |= REG2_FULL_SCALE_2G_A;
        new_scale_g_digit = 0.061e-3f;

    } else if (max_g <= 4) {
        _accel_range_m_s2 = 4.0f*LSM303D_ONE_G;
        setbits |= REG2_FULL_SCALE_4G_A;
        new_scale_g_digit = 0.122e-3f;

    } else if (max_g <= 6) {
        _accel_range_m_s2 = 6.0f*LSM303D_ONE_G;
        setbits |= REG2_FULL_SCALE_6G_A;
        new_scale_g_digit = 0.183e-3f;

    } else if (max_g <= 8) {
        _accel_range_m_s2 = 8.0f*LSM303D_ONE_G;
        setbits |= REG2_FULL_SCALE_8G_A;
        new_scale_g_digit = 0.244e-3f;

    } else if (max_g <= 16) {
        _accel_range_m_s2 = 16.0f*LSM303D_ONE_G;
        setbits |= REG2_FULL_SCALE_16G_A;
        new_scale_g_digit = 0.732e-3f;

    } else {
        return -1;
    }

    _accel_range_scale = new_scale_g_digit * LSM303D_ONE_G;
    _register_modify(ADDR_CTRL_REG2, clearbits, setbits);
    return 0;
}

uint8_t AP_InertialSensor_LSM303D::accel_set_samplerate(uint16_t frequency)
{
    uint8_t setbits = 0;
    uint8_t clearbits = REG1_RATE_BITS_A;

    if (frequency == 0)
        frequency = 1600;

    if (frequency <= 100) {
        setbits |= REG1_RATE_100HZ_A;
        _accel_samplerate = 100;

    } else if (frequency <= 200) {
        setbits |= REG1_RATE_200HZ_A;
        _accel_samplerate = 200;

    } else if (frequency <= 400) {
        setbits |= REG1_RATE_400HZ_A;
        _accel_samplerate = 400;

    } else if (frequency <= 800) {
        setbits |= REG1_RATE_800HZ_A;
        _accel_samplerate = 800;

    } else if (frequency <= 1600) {
        setbits |= REG1_RATE_1600HZ_A;
        _accel_samplerate = 1600;

    } else {
        return -1;
    }

    _register_modify(ADDR_CTRL_REG1, clearbits, setbits);
    _reg1_expected = (_reg1_expected & ~clearbits) | setbits;
    return 0;
}

uint8_t AP_InertialSensor_LSM303D::accel_set_onchip_lowpass_filter_bandwidth(uint8_t bandwidth)
{
    uint8_t setbits = 0;
    uint8_t clearbits = REG2_ANTIALIAS_FILTER_BW_BITS_A;

    if (bandwidth == 0)
        bandwidth = 773;

    if (bandwidth <= 50) {
        setbits |= REG2_AA_FILTER_BW_50HZ_A;
        _accel_onchip_filter_bandwith = 50;

    } else if (bandwidth <= 194) {
        setbits |= REG2_AA_FILTER_BW_194HZ_A;
        _accel_onchip_filter_bandwith = 194;

    } else if (bandwidth <= 362) {
        setbits |= REG2_AA_FILTER_BW_362HZ_A;
        _accel_onchip_filter_bandwith = 362;

    } else if (bandwidth <= 773) {
        setbits |= REG2_AA_FILTER_BW_773HZ_A;
        _accel_onchip_filter_bandwith = 773;

    } else {
        return -1;
    }

    _register_modify(ADDR_CTRL_REG2, clearbits, setbits);
    return 0;
}

uint8_t AP_InertialSensor_LSM303D::mag_set_range(uint8_t max_ga)
{
    uint8_t setbits = 0;
    uint8_t clearbits = REG6_FULL_SCALE_BITS_M;
    float new_scale_ga_digit = 0.0f;

    if (max_ga == 0)
        max_ga = 12;

    if (max_ga <= 2) {
        _mag_range_ga = 2;
        setbits |= REG6_FULL_SCALE_2GA_M;
        new_scale_ga_digit = 0.080e-3f;

    } else if (max_ga <= 4) {
        _mag_range_ga = 4;
        setbits |= REG6_FULL_SCALE_4GA_M;
        new_scale_ga_digit = 0.160e-3f;

    } else if (max_ga <= 8) {
        _mag_range_ga = 8;
        setbits |= REG6_FULL_SCALE_8GA_M;
        new_scale_ga_digit = 0.320e-3f;

    } else if (max_ga <= 12) {
        _mag_range_ga = 12;
        setbits |= REG6_FULL_SCALE_12GA_M;
        new_scale_ga_digit = 0.479e-3f;

    } else {
        return -1;
    }

    _mag_range_scale = new_scale_ga_digit;
    _register_modify(ADDR_CTRL_REG6, clearbits, setbits);
    return 0;
}

uint8_t AP_InertialSensor_LSM303D::mag_set_samplerate(uint16_t frequency)
{
    uint8_t setbits = 0;
    uint8_t clearbits = REG5_RATE_BITS_M;

    if (frequency == 0)
        frequency = 100;

    if (frequency <= 25) {
        setbits |= REG5_RATE_25HZ_M;
        _mag_samplerate = 25;

    } else if (frequency <= 50) {
        setbits |= REG5_RATE_50HZ_M;
        _mag_samplerate = 50;

    } else if (frequency <= 100) {
        setbits |= REG5_RATE_100HZ_M;
        _mag_samplerate = 100;

    } else {
        return -1;
    }

    _register_modify(ADDR_CTRL_REG5, clearbits, setbits);
    return 0;
}

bool AP_InertialSensor_LSM303D::_hardware_init(Sample_rate sample_rate)
{
    if (!_spi_sem->take(100)) {
        hal.scheduler->panic(PSTR("LSM303D: Unable to get semaphore"));
    }

    // initially run the bus at low speed
    _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_LOW);
       
    // ensure the chip doesn't interpret any other bus traffic as I2C
	disable_i2c();


    /* enable accel*/
    _reg1_expected = REG1_X_ENABLE_A | REG1_Y_ENABLE_A | REG1_Z_ENABLE_A | REG1_BDU_UPDATE | REG1_RATE_800HZ_A;
    _register_write(ADDR_CTRL_REG1, _reg1_expected);

    /* enable mag */
    _reg7_expected = REG7_CONT_MODE_M;
    _register_write(ADDR_CTRL_REG7, _reg7_expected);
    _register_write(ADDR_CTRL_REG5, REG5_RES_HIGH_M);
    _register_write(ADDR_CTRL_REG3, 0x04); // DRDY on ACCEL on INT1
    _register_write(ADDR_CTRL_REG4, 0x04); // DRDY on MAG on INT2

    accel_set_range(LSM303D_ACCEL_DEFAULT_RANGE_G);
    accel_set_samplerate(LSM303D_ACCEL_DEFAULT_RATE);

    // Hardware filtering
    // we setup the anti-alias on-chip filter as 50Hz. We believe
    // this operates in the analog domain, and is critical for
    // anti-aliasing. The 2 pole software filter is designed to
    // operate in conjunction with this on-chip filter
    accel_set_onchip_lowpass_filter_bandwidth(LSM303D_ACCEL_DEFAULT_ONCHIP_FILTER_FREQ);

    mag_set_range(LSM303D_MAG_DEFAULT_RANGE_GA);
    mag_set_samplerate(LSM303D_MAG_DEFAULT_RATE);

    // TODO: Software filtering
    // accel_set_driver_lowpass_filter((float)LSM303D_ACCEL_DEFAULT_RATE, (float)LSM303D_ACCEL_DEFAULT_DRIVER_FILTER_FREQ);

    // uint8_t default_filter;

    // // sample rate and filtering
    // // to minimise the effects of aliasing we choose a filter
    // // that is less than half of the sample rate
    // switch (sample_rate) {
    // case RATE_50HZ:
    //     // this is used for plane and rover, where noise resistance is
    //     // more important than update rate. Tests on an aerobatic plane
    //     // show that 10Hz is fine, and makes it very noise resistant
    //     default_filter = BITS_DLPF_CFG_10HZ;
    //     _sample_shift = 2;
    //     break;
    // case RATE_100HZ:
    //     default_filter = BITS_DLPF_CFG_20HZ;
    //     _sample_shift = 1;
    //     break;
    // case RATE_200HZ:
    // default:
    //     default_filter = BITS_DLPF_CFG_20HZ;
    //     _sample_shift = 0;
    //     break;
    // }
    // _set_filter_register(_LSM303D_filter, default_filter);

    // now that we have initialised, we set the SPI bus speed to high
    _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_HIGH);
    _spi_sem->give();

    return true;
}

// return true if a sample is available
bool AP_InertialSensor_LSM303D::_sample_available()
{
    _poll_data();
    // return (_sum_count >> _sample_shift) > 0;
    return (_sum_count) > 0;    
}


// TODO fix dump registers
#if LSM303D_DEBUG
// dump all config registers - used for debug
void AP_InertialSensor_LSM303D::_dump_registers(void)
{
    hal.console->println_P(PSTR("LSM303D registers"));
    if (_spi_sem->take(100)) {
        for (uint8_t reg=ADDR_WHO_AM_I; reg<=56; reg++) { // 0x38 = 56
            uint8_t v = _register_read(reg);
            hal.console->printf_P(PSTR("%02x:%02x "), (unsigned)reg, (unsigned)v);
            if ((reg - (ADDR_WHO_AM_I-1)) % 16 == 0) {
                hal.console->println();
            }
        }
        hal.console->println();
        _spi_sem->give();
    }
}
#endif


// get_delta_time returns the time period in seconds overwhich the sensor data was collected
float AP_InertialSensor_LSM303D::get_delta_time() const
{
    // the sensor runs at 200Hz
    return 0.005 * _num_samples;
}
#endif
