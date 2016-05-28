/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/****************************************************************************
 *
 *	 Coded by Víctor Mayoral Vilches <v.mayoralv@gmail.com> using 
 *	 l3gd20.cpp <https://github.com/diydrones/PX4Firmware> from the PX4 Development Team.
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
#if NOT_YET

#include "AP_InertialSensor_L3GD20.h"

extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
	#define L3GD20_DRDY_PIN 70
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
	#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLE || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF
		#include "../AP_HAL_Linux/GPIO.h"
		#define L3GD20_DRDY_PIN BBB_P8_34  // GYRO_DRDY
	#endif
#endif

/* L3GD20 definitions */
/* Orientation on board */
#define SENSOR_BOARD_ROTATION_000_DEG	0
#define SENSOR_BOARD_ROTATION_090_DEG	1
#define SENSOR_BOARD_ROTATION_180_DEG	2
#define SENSOR_BOARD_ROTATION_270_DEG	3

/* SPI protocol address bits */
#define DIR_READ				(1<<7)
#define DIR_WRITE				(0<<7)
#define ADDR_INCREMENT				(1<<6)

/* register addresses */
#define ADDR_WHO_AM_I			0x0F
#define WHO_I_AM_H 				0xD7
#define WHO_I_AM				0xD4

#define ADDR_CTRL_REG1			0x20
#define REG1_RATE_LP_MASK			0xF0 /* Mask to guard partial register update */
/* keep lowpass low to avoid noise issues */
#define RATE_95HZ_LP_25HZ		((0<<7) | (0<<6) | (0<<5) | (1<<4))
#define RATE_190HZ_LP_25HZ		((0<<7) | (1<<6) | (0<<5) | (1<<4))
#define RATE_190HZ_LP_50HZ		((0<<7) | (1<<6) | (1<<5) | (0<<4))
#define RATE_190HZ_LP_70HZ		((0<<7) | (1<<6) | (1<<5) | (1<<4))
#define RATE_380HZ_LP_20HZ		((1<<7) | (0<<6) | (1<<5) | (0<<4))
#define RATE_380HZ_LP_25HZ		((1<<7) | (0<<6) | (0<<5) | (1<<4))
#define RATE_380HZ_LP_50HZ		((1<<7) | (0<<6) | (1<<5) | (0<<4))
#define RATE_380HZ_LP_100HZ		((1<<7) | (0<<6) | (1<<5) | (1<<4))
#define RATE_760HZ_LP_30HZ		((1<<7) | (1<<6) | (0<<5) | (0<<4))
#define RATE_760HZ_LP_35HZ		((1<<7) | (1<<6) | (0<<5) | (1<<4))
#define RATE_760HZ_LP_50HZ		((1<<7) | (1<<6) | (1<<5) | (0<<4))
#define RATE_760HZ_LP_100HZ		((1<<7) | (1<<6) | (1<<5) | (1<<4))

#define ADDR_CTRL_REG2			0x21
#define ADDR_CTRL_REG3			0x22
#define ADDR_CTRL_REG4			0x23
#define REG4_RANGE_MASK				0x30 /* Mask to guard partial register update */
#define RANGE_250DPS				(0<<4)
#define RANGE_500DPS				(1<<4)
#define RANGE_2000DPS				(3<<4)

#define ADDR_CTRL_REG5			0x24
#define ADDR_REFERENCE			0x25
#define ADDR_OUT_TEMP			0x26
#define ADDR_STATUS_REG			0x27
#define ADDR_OUT_X_L			0x28
#define ADDR_OUT_X_H			0x29
#define ADDR_OUT_Y_L			0x2A
#define ADDR_OUT_Y_H			0x2B
#define ADDR_OUT_Z_L			0x2C
#define ADDR_OUT_Z_H			0x2D
#define ADDR_FIFO_CTRL_REG		0x2E
#define ADDR_FIFO_SRC_REG		0x2F
#define ADDR_INT1_CFG			0x30
#define ADDR_INT1_SRC			0x31
#define ADDR_INT1_TSH_XH		0x32
#define ADDR_INT1_TSH_XL		0x33
#define ADDR_INT1_TSH_YH		0x34
#define ADDR_INT1_TSH_YL		0x35
#define ADDR_INT1_TSH_ZH		0x36
#define ADDR_INT1_TSH_ZL		0x37
#define ADDR_INT1_DURATION		0x38

/* Internal configuration values */
#define REG1_POWER_NORMAL			(1<<3)
#define REG1_Z_ENABLE				(1<<2)
#define REG1_Y_ENABLE				(1<<1)
#define REG1_X_ENABLE				(1<<0)

#define REG4_BDU				(1<<7)
#define REG4_BLE				(1<<6)
//#define REG4_SPI_3WIRE			(1<<0)

#define REG5_FIFO_ENABLE			(1<<6)
#define REG5_REBOOT_MEMORY			(1<<7)

#define STATUS_ZYXOR				(1<<7)
#define STATUS_ZOR				(1<<6)
#define STATUS_YOR				(1<<5)
#define STATUS_XOR				(1<<4)
#define STATUS_ZYXDA				(1<<3)
#define STATUS_ZDA				(1<<2)
#define STATUS_YDA				(1<<1)
#define STATUS_XDA				(1<<0)

#define FIFO_CTRL_BYPASS_MODE			(0<<5)
#define FIFO_CTRL_FIFO_MODE			(1<<5)
#define FIFO_CTRL_STREAM_MODE			(1<<6)
#define FIFO_CTRL_STREAM_TO_FIFO_MODE		(3<<5)
#define FIFO_CTRL_BYPASS_TO_STREAM_MODE		(1<<7)

#define L3GD20_DEFAULT_RATE			760
#define L3GD20_DEFAULT_RANGE_DPS		2000
#define L3GD20_DEFAULT_FILTER_FREQ		30


// const float AP_InertialSensor_L3GD20::_gyro_scale = (0.0174532f / 16.4f);


AP_InertialSensor_L3GD20::AP_InertialSensor_L3GD20() : 
	AP_InertialSensor(),
    _drdy_pin(NULL),
    _initialised(false),
    _L3GD20_product_id(AP_PRODUCT_ID_NONE)
{
}

uint16_t AP_InertialSensor_L3GD20::_init_sensor( Sample_rate sample_rate )
{
    if (_initialised) return _L3GD20_product_id;
    _initialised = true;

    _spi = hal.spi->device(AP_HAL::SPIDevice_L3GD20);
    _spi_sem = _spi->get_semaphore();

#ifdef L3GD20_DRDY_PIN
    _drdy_pin = hal.gpio->channel(L3GD20_DRDY_PIN);
    _drdy_pin->mode(HAL_GPIO_INPUT);
#endif

    hal.scheduler->suspend_timer_procs();

    // Test WHOAMI
    uint8_t whoami = _register_read(ADDR_WHO_AM_I);
    if (whoami != WHO_I_AM) {
        // TODO: we should probably accept multiple chip
        // revisions. This is the one on the PXF
        hal.console->printf("L3GD20: unexpected WHOAMI 0x%x\n", (unsigned)whoami);
        hal.scheduler->panic(PSTR("L3GD20: bad WHOAMI"));
    }

    uint8_t tries = 0;
    do {
        bool success = _hardware_init(sample_rate);
        if (success) {
            hal.scheduler->delay(5+2);
            if (!_spi_sem->take(100)) {
                hal.scheduler->panic(PSTR("L3GD20: Unable to get semaphore"));
            }
            if (_data_ready()) {
                _spi_sem->give();
                break;
            } else {
                hal.console->println_P(
                        PSTR("L3GD20 startup failed: no data ready"));
            }
            _spi_sem->give();
        }
        if (tries++ > 5) {
            hal.scheduler->panic(PSTR("PANIC: failed to boot L3GD20 5 times")); 
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
    hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&AP_InertialSensor_L3GD20::_poll_data));

#if L3GD20_DEBUG
    _dump_registers();
#endif
    return _L3GD20_product_id;
}

/*================ AP_INERTIALSENSOR PUBLIC INTERFACE ==================== */

bool AP_InertialSensor_L3GD20::wait_for_sample(uint16_t timeout_ms)
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

bool AP_InertialSensor_L3GD20::update( void )
{
    // wait for at least 1 sample
    if (!wait_for_sample(1000)) {
        return false;
    }

    // disable timer procs for mininum time
    hal.scheduler->suspend_timer_procs();
    _gyro[0]  = Vector3f(_gyro_sum.x, _gyro_sum.y, _gyro_sum.z);
    _num_samples = _sum_count;
    _gyro_sum.zero();
    _sum_count = 0;
    hal.scheduler->resume_timer_procs();

    _gyro[0].rotate(_board_orientation);
    _gyro[0] *= _gyro_scale / _num_samples;
    _gyro[0] -= _gyro_offset[0];

    // if (_last_filter_hz != _L3GD20_filter) {
    //     if (_spi_sem->take(10)) {
    //         _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_LOW);
    //         _set_filter_register(_L3GD20_filter, 0);
    //         _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_HIGH);
    //         _error_count = 0;
    //         _spi_sem->give();
    //     }
    // }

    return true;
}

/*================ HARDWARE FUNCTIONS ==================== */

/**
 * Return true if the L3GD20 has new data available for reading.
 *
 * We use the data ready pin if it is available.  Otherwise, read the
 * status register.
 */
bool AP_InertialSensor_L3GD20::_data_ready()
{
    if (_drdy_pin) {
        return _drdy_pin->read() != 0;
    }
    // TODO: read status register
    return false;
}

/**
 * Timer process to poll for new data from the L3GD20.
 */
void AP_InertialSensor_L3GD20::_poll_data(void)
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
                PSTR("PANIC: AP_InertialSensor_L3GD20::_poll_data "
                     "failed to take SPI semaphore synchronously"));
        }
    }
}

void AP_InertialSensor_L3GD20::_read_data_transaction() {
    
    struct {
        uint8_t     cmd;
        uint8_t     temp;
        uint8_t     status;
        int16_t     x;
        int16_t     y;
        int16_t     z;
    } raw_report;

    /* fetch data from the sensor */
    memset(&raw_report, 0, sizeof(raw_report));
    raw_report.cmd = ADDR_OUT_TEMP | DIR_READ | ADDR_INCREMENT;
    _spi->transaction((uint8_t *)&raw_report, (uint8_t *)&raw_report, sizeof(raw_report));

#if L3GD20_USE_DRDY
        if ((raw_report.status & 0xF) != 0xF) {
            /*
              we waited for DRDY, but did not see DRDY on all axes
              when we captured. That means a transfer error of some sort
             */
            hal.console->println_P(
                    PSTR("L3GD20: DRDY is not on in all axes, transfer error"));
            return;
        }
#endif
    _gyro_sum.x += raw_report.x;
    _gyro_sum.y  += raw_report.y;
    _gyro_sum.z  -= raw_report.z;
    _sum_count++;

    if (_sum_count == 0) {
        // rollover - v unlikely
        _gyro_sum.zero();
    }
}

uint8_t AP_InertialSensor_L3GD20::_register_read( uint8_t reg )
{
    uint8_t addr = reg | 0x80; // Set most significant bit

    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = addr;
    tx[1] = 0;
    _spi->transaction(tx, rx, 2);

    return rx[1];
}

void AP_InertialSensor_L3GD20::_register_write(uint8_t reg, uint8_t val)
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
void AP_InertialSensor_L3GD20::_register_write_check(uint8_t reg, uint8_t val)
{
    uint8_t readed;
    _register_write(reg, val);
    readed = _register_read(reg);
    if (readed != val){
	hal.console->printf_P(PSTR("Values doesn't match; written: %02x; read: %02x "), val, readed);
    }
#if L3GD20_DEBUG
    hal.console->printf_P(PSTR("Values written: %02x; readed: %02x "), val, readed);
#endif
}

/*
  set the DLPF filter frequency. Assumes caller has taken semaphore
  TODO needs to be changed according to L3GD20 needs
 */
// void AP_InertialSensor_L3GD20::_set_filter_register(uint8_t filter_hz, uint8_t default_filter)
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


void AP_InertialSensor_L3GD20::disable_i2c(void)
{
	uint8_t retries = 10;
	while (retries--) {
		// add retries
		uint8_t a = _register_read(0x05);
		_register_write(0x05, (0x20 | a));
		if (_register_read(0x05) == (a | 0x20)) {
			return;
		}
	}	
	hal.scheduler->panic(PSTR("L3GD20: Unable to disable I2C"));
}

uint8_t AP_InertialSensor_L3GD20::set_samplerate(uint16_t frequency)
{
	uint8_t bits = REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE;
	if (frequency == 0)
		frequency = 760;

	/* use limits good for H or non-H models */
	if (frequency <= 100) {
		// _current_rate = 95;
		bits |= RATE_95HZ_LP_25HZ;

	} else if (frequency <= 200) {
		// _current_rate = 190;
		bits |= RATE_190HZ_LP_50HZ;

	} else if (frequency <= 400) {
		// _current_rate = 380;
		bits |= RATE_380HZ_LP_50HZ;

	} else if (frequency <= 800) {
		// _current_rate = 760;
		bits |= RATE_760HZ_LP_50HZ;
	} else {
		return -1;
	}
	_register_write(ADDR_CTRL_REG1, bits);
	return 0;
}

uint8_t AP_InertialSensor_L3GD20::set_range(uint8_t max_dps)
{
	uint8_t bits = REG4_BDU;
	float new_range_scale_dps_digit;
	float new_range;

	if (max_dps == 0) {
		max_dps = 2000;
	}
	if (max_dps <= 250) {
		new_range = 250;
		bits |= RANGE_250DPS;
		new_range_scale_dps_digit = 8.75e-3f;

	} else if (max_dps <= 500) {
		new_range = 500;
		bits |= RANGE_500DPS;
		new_range_scale_dps_digit = 17.5e-3f;

	} else if (max_dps <= 2000) {
		new_range = 2000;
		bits |= RANGE_2000DPS;
		new_range_scale_dps_digit = 70e-3f;

	} else {
		return -1;
	}

	// _gyro_range_rad_s = new_range / 180.0f * M_PI_F;
	// _gyro_range_scale = new_range_scale_dps_digit / 180.0f * M_PI_F;	
	_gyro_scale = new_range_scale_dps_digit / 180.0f * M_PI_F;	    
	_register_write(ADDR_CTRL_REG4, bits);
	return 0;
}

bool AP_InertialSensor_L3GD20::_hardware_init(Sample_rate sample_rate)
{
    if (!_spi_sem->take(100)) {
        hal.scheduler->panic(PSTR("L3GD20: Unable to get semaphore"));
    }

    // initially run the bus at low speed
    _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_LOW);
       
    // ensure the chip doesn't interpret any other bus traffic as I2C
	disable_i2c();

	// Chip reset 
	/* set default configuration */
	_register_write(ADDR_CTRL_REG1, REG1_POWER_NORMAL | REG1_Z_ENABLE | REG1_Y_ENABLE | REG1_X_ENABLE);
    hal.scheduler->delay(1);
	_register_write(ADDR_CTRL_REG2, 0);		/* disable high-pass filters */
    hal.scheduler->delay(1);
	_register_write(ADDR_CTRL_REG3, 0x08);        /* DRDY enable */
    hal.scheduler->delay(1);
	_register_write(ADDR_CTRL_REG4, REG4_BDU);
    hal.scheduler->delay(1);
	_register_write(ADDR_CTRL_REG5, 0);
    hal.scheduler->delay(1);

	_register_write(ADDR_CTRL_REG5, REG5_FIFO_ENABLE);		/* disable wake-on-interrupt */
    hal.scheduler->delay(1);

	/* disable FIFO. This makes things simpler and ensures we
	 * aren't getting stale data. It means we must run the hrt
	 * callback fast enough to not miss data. */
	_register_write(ADDR_FIFO_CTRL_REG, FIFO_CTRL_BYPASS_MODE);
    hal.scheduler->delay(1);

	set_samplerate(0); // 760Hz
    hal.scheduler->delay(1);
	set_range(L3GD20_DEFAULT_RANGE_DPS);	
    hal.scheduler->delay(1);

    // //TODO: Filtering
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
    // _set_filter_register(_L3GD20_filter, default_filter);

    // now that we have initialised, we set the SPI bus speed to high
    _spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_HIGH);
    _spi_sem->give();

    return true;
}

// return the MPU6k gyro drift rate in radian/s/s
// note that this is much better than the oilpan gyros
float AP_InertialSensor_L3GD20::get_gyro_drift_rate(void)
{
    // 0.5 degrees/second/minute
    return ToRad(0.5/60);
}

// return true if a sample is available
bool AP_InertialSensor_L3GD20::_sample_available()
{
    _poll_data();
    // return (_sum_count >> _sample_shift) > 0;
    return (_sum_count) > 0;    
}


#if L3GD20_DEBUG
// dump all config registers - used for debug
void AP_InertialSensor_L3GD20::_dump_registers(void)
{
    hal.console->println_P(PSTR("L3GD20 registers"));
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
float AP_InertialSensor_L3GD20::get_delta_time() const
{
    // the sensor runs at 200Hz
    return 0.005 * _num_samples;
}
#endif
