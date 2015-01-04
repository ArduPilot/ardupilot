/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
 *       AP_Compass_AK8963.cpp 
 *       Code by Georgii Staroselskii. Emlid.com
 *
 *       Sensor is conected to SPI port
 *
 */

#include <AP_Math.h>
#include <AP_HAL.h>
#include "AP_Compass_AK8963.h"


#define READ_FLAG                   0x80
#define MPUREG_I2C_SLV0_ADDR        0x25
#define MPUREG_I2C_SLV0_REG         0x26
#define MPUREG_I2C_SLV0_CTRL        0x27
#define MPUREG_EXT_SENS_DATA_00     0x49
#define MPUREG_I2C_SLV0_DO          0x63

#define MPU9250_SPI_BACKEND 1

#define MPUREG_PWR_MGMT_1                               0x6B
#       define BIT_PWR_MGMT_1_CLK_INTERNAL              0x00            // clock set to internal 8Mhz oscillator
#       define BIT_PWR_MGMT_1_CLK_XGYRO                 0x01            // PLL with X axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_YGYRO                 0x02            // PLL with Y axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_ZGYRO                 0x03            // PLL with Z axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_EXT32KHZ              0x04            // PLL with external 32.768kHz reference
#       define BIT_PWR_MGMT_1_CLK_EXT19MHZ              0x05            // PLL with external 19.2MHz reference
#       define BIT_PWR_MGMT_1_CLK_STOP                  0x07            // Stops the clock and keeps the timing generator in reset
#       define BIT_PWR_MGMT_1_TEMP_DIS                  0x08            // disable temperature sensor
#       define BIT_PWR_MGMT_1_CYCLE                     0x20            // put sensor into cycle mode.  cycles between sleep mode and waking up to take a single sample of data from active sensors at a rate determined by LP_WAKE_CTRL
#       define BIT_PWR_MGMT_1_SLEEP                     0x40            // put sensor into low power sleep mode
#       define BIT_PWR_MGMT_1_DEVICE_RESET              0x80            // reset entire device

/* bit definitions for MPUREG_USER_CTRL */
#define MPUREG_USER_CTRL                                0x6A
#       define BIT_USER_CTRL_I2C_MST_EN                 0x20            /* Enable MPU to act as the I2C Master to external slave sensors */
#       define BIT_USER_CTRL_I2C_IF_DIS                 0x10

/* bit definitions for MPUREG_MST_CTRL */
#define MPUREG_I2C_MST_CTRL                             0x24
#        define I2C_SLV0_EN                             0x80
#        define I2C_MST_CLOCK_400KHZ                    0x0D

#define AK8963_I2C_ADDR                                 0x0c

#define AK8963_WIA                                      0x00
#        define AK8963_Device_ID                        0x48

#define AK8963_INFO                                     0x01

#define AK8963_ST1                                      0x02
#        define AK8963_DRDY                             0x01
#        define AK8963_DOR                              0x02

#define AK8963_HXL                                      0x03

/* bit definitions for AK8963 CNTL1 */
#define AK8963_CNTL1                                    0x0A
#        define    AK8963_CONTINUOUS_MODE1              0x2
#        define    AK8963_CONTINUOUS_MODE2              0x6
#        define    AK8963_SELFTEST_MODE                 0x8
#        define    AK8963_POWERDOWN_MODE                0x0
#        define    AK8963_FUSE_MODE                     0xf
#        define    AK8963_16BIT_ADC                     0x10
#        define    AK8963_14BIT_ADC                     0x00

#define AK8963_CNTL2                                    0x0B
#        define AK8963_RESET                            0x01

#define AK8963_ASTC                                     0x0C
#        define AK8983_SELFTEST_MAGNETIC_FIELD_ON       0x40

#define AK8963_ASAX                                     0x10

#define AK8963_DEBUG 0
#define AK8963_SELFTEST 0
#if AK8963_DEBUG
#define error(...) fprintf(stderr, __VA_ARGS__)
#define debug(...) hal.console->printf(__VA_ARGS__)
#define ASSERT(x) assert(x)
#else
#define error(...) 
#define debug(...) 
#define ASSERT(x)
#endif

extern const AP_HAL::HAL& hal;

AK8963_MPU9250_SPI_Backend::AK8963_MPU9250_SPI_Backend()
{
    _spi = hal.spi->device(AP_HAL::SPIDevice_MPU9250);

    if (_spi == NULL) {
        hal.scheduler->panic(PSTR("Cannot get SPIDevice_MPU9250"));
    }

    _spi_sem = _spi->get_semaphore();
}

bool AK8963_MPU9250_SPI_Backend::sem_take_blocking()
{
    return _spi_sem->take(10);
}

bool AK8963_MPU9250_SPI_Backend::sem_give()
{
    return _spi_sem->give();
}

bool AK8963_MPU9250_SPI_Backend::sem_take_nonblocking()
{
    /**
     * Take nonblocking from a TimerProcess context &
     * monitor for bad failures
     */
    static int _sem_failure_count = 0;
    bool got = _spi_sem->take_nonblocking();
    if (!got) {
        if (!hal.scheduler->system_initializing()) {
            _sem_failure_count++;
            if (_sem_failure_count > 100) {
                hal.scheduler->panic(PSTR("PANIC: failed to take _spi_sem "
                                          "100 times in a row, in "
                                          "AP_Compass_AK8963::_update"));
            }
        }
        return false; /* never reached */
    } else {
        _sem_failure_count = 0;
    }
    return got;
}

void AK8963_MPU9250_SPI_Backend::read(uint8_t address, uint8_t *buf, uint32_t count)
{
    ASSERT(count < 10);
    uint8_t tx[11];
    uint8_t rx[11];

    tx[0] = address | READ_FLAG;
    tx[1] = 0;
    _spi->transaction(tx, rx, count + 1);

    memcpy(buf, rx + 1, count);
}

void AK8963_MPU9250_SPI_Backend::write(uint8_t address, const uint8_t *buf, uint32_t count)
{
    ASSERT(count < 2);
    uint8_t tx[2];

    tx[0] = address;
    memcpy(tx+1, buf, count);

    _spi->transaction(tx, NULL, count + 1);
}

AP_Compass_AK8963_MPU9250::AP_Compass_AK8963_MPU9250()
{
    product_id = AP_COMPASS_TYPE_AK8963_MPU9250;
}

void AP_Compass_AK8963_MPU9250::_dump_registers()
{
    error(PSTR("MPU9250 registers\n"));
    for (uint8_t reg=0x00; reg<=0x7E; reg++) {
        uint8_t v = _backend->read(reg);
        error(("%02x:%02x "), (unsigned)reg, (unsigned)v);
        if (reg  % 16 == 0) {
            error("\n");
        }
    }
    error("\n");
}

void AP_Compass_AK8963_MPU9250::_backend_reset()
{
    _backend->write(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_DEVICE_RESET);
}

bool AP_Compass_AK8963_MPU9250::_backend_init()
{
    _backend->write(MPUREG_USER_CTRL, BIT_USER_CTRL_I2C_MST_EN);    /* I2C Master mode */
    _backend->write(MPUREG_I2C_MST_CTRL, I2C_MST_CLOCK_400KHZ);    /*  I2C configuration multi-master  IIC 400KHz */

    return true;
}

bool AP_Compass_AK8963_MPU9250::init() 
{
#if MPU9250_SPI_BACKEND
    _backend = new AK8963_MPU9250_SPI_Backend();
    if (_backend == NULL) {
        hal.scheduler->panic(PSTR("_backend coudln't be allocated"));
    }
    return AP_Compass_AK8963::init();
#else
#error Wrong backend for AK8963 is selected
    /* other backends not implented yet */
    return false;
#endif
}

void AP_Compass_AK8963_MPU9250::_register_write(uint8_t address, uint8_t value)
{
    _backend->write(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR);  /* Set the I2C slave addres of AK8963 and set for _register_write. */
    _backend->write(MPUREG_I2C_SLV0_REG, address); /* I2C slave 0 register address from where to begin data transfer */
    _backend->write(MPUREG_I2C_SLV0_DO, value); /* Register value to continuous measurement in 16-bit */
    _backend->write(MPUREG_I2C_SLV0_CTRL, I2C_SLV0_EN | 0x01); /* Enable I2C and set 1 byte */
}

void AP_Compass_AK8963_MPU9250::_register_read(uint8_t address, uint8_t count, uint8_t *value)
{
    _backend->write(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG);  /* Set the I2C slave addres of AK8963 and set for read. */
    _backend->write(MPUREG_I2C_SLV0_REG, address); /* I2C slave 0 register address from where to begin data transfer */
    _backend->write(MPUREG_I2C_SLV0_CTRL, I2C_SLV0_EN | count); /* Enable I2C and set @count byte */

    hal.scheduler->delay(10);
    _backend->read(MPUREG_EXT_SENS_DATA_00, value, count);
}

uint8_t AP_Compass_AK8963_MPU9250::_read_id()
{
    return 1;
}

bool AP_Compass_AK8963_MPU9250::_read_raw()
{
    uint8_t rx[14] = {0};

    const uint8_t count = 9;
    _backend->read(MPUREG_EXT_SENS_DATA_00, rx, count);

    uint8_t st1 = rx[1]; /* Read ST1 register */
    uint8_t st2 = rx[8]; /* End data read by reading ST2 register */

#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx + 1] << 8) | v[2*idx]))

    if(!(st2 & 0x08)) { 
        _mag_x = (float) int16_val(rx, 1);
        _mag_y = (float) int16_val(rx, 2);
        _mag_z = (float) int16_val(rx, 3);

#if 1
        error("%f %f %f st1: 0x%x st2: 0x%x\n", _mag_x, _mag_y, _mag_z, st1, st2);
#endif
        if (_mag_x == 0 && _mag_y == 0 && _mag_z == 0) {
            return false;
        }

        return true;
    } else {
        return false;
    }

}

AP_Compass_AK8963::AP_Compass_AK8963() : 
    Compass(),
    _backend(NULL)
{
    _healthy[0] = true;
    _initialised = false;
    _mag_x_accum =_mag_y_accum = _mag_z_accum = 0;
    _mag_x =_mag_y = _mag_z = 0;
    _accum_count = 0;
    _magnetometer_adc_resolution = AK8963_16BIT_ADC;
}


/* stub to satisfy Compass API*/
void AP_Compass_AK8963::accumulate(void)
{
}

bool AP_Compass_AK8963::_self_test()
{
    bool success = false;

    /* see AK8963.pdf p.19 */

    /* Set power-down mode */
    _register_write(AK8963_CNTL1, AK8963_POWERDOWN_MODE | _magnetometer_adc_resolution);

    /* Turn the internal magnetic field on */
    _register_write(AK8963_ASTC, AK8983_SELFTEST_MAGNETIC_FIELD_ON); 

    /* Register value to self-test mode in 14-bit */
    _register_write(AK8963_CNTL1, AK8963_SELFTEST_MODE | _magnetometer_adc_resolution); 

    _start_conversion();
    hal.scheduler->delay(20);
    _read_raw();

    float hx = _mag_x;
    float hy = _mag_y;
    float hz = _mag_z;

    error("AK8963's SELF-TEST STARTED\n");

    switch (_magnetometer_adc_resolution) {
        bool hx_is_in_range;
        bool hy_is_in_range;
        bool hz_is_in_range;
        case AK8963_14BIT_ADC: 
            hx_is_in_range = (hx >= - 50) && (hx <= 50);
            hy_is_in_range = (hy >= - 50) && (hy <= 50);
            hz_is_in_range = (hz >= - 800) && (hz <= -200);
            if (hx_is_in_range && hy_is_in_range && hz_is_in_range) {
                success = true;
            }
            break;
        case AK8963_16BIT_ADC:
            hx_is_in_range = (hx >= -200) && (hx <= 200);
            hy_is_in_range = (hy >= -200) && (hy <= 200);
            hz_is_in_range = (hz >= -3200) && (hz <= -800);
            if (hx_is_in_range && hy_is_in_range && hz_is_in_range) {
                success = true;
            }
            break;
        default:
            success = false;
            hal.scheduler->panic(PSTR("Wrong AK8963's ADC resolution selected"));
            break;
    }

    error("AK8963's SELF-TEST ENDED: %f %f %f\n", hx, hy, hz);

    /* Turn the internal magnetic field off */
    _register_write(AK8963_ASTC, 0x0); 

    /* Register value to continuous measurement in 14-bit */
    _register_write(AK8963_CNTL1, AK8963_POWERDOWN_MODE | _magnetometer_adc_resolution); 

    return success;
}

bool AP_Compass_AK8963::init()
{
    _healthy[0] = true;

    _field[0].x = 0.0f;
    _field[0].y = 0.0f;
    _field[0].z = 0.0f;

    hal.scheduler->suspend_timer_procs();
    if (!_backend->sem_take_blocking()) {
        error("_spi_sem->take failed\n");
        return false;
    }


    if (!_backend_init()) {
        _backend->sem_give();
        return false;
    }

    _register_write(AK8963_CNTL2, AK8963_RESET); /* Reset AK8963 */

    hal.scheduler->delay(1000);

    int id_mismatch_count;
    uint8_t deviceid;
    for (id_mismatch_count = 0; id_mismatch_count < 5; id_mismatch_count++) {
        _register_read(AK8963_WIA, 0x01, &deviceid); /* Read AK8963's id */

        if (deviceid == AK8963_Device_ID) {
            break;
        }

        error("trying to read AK8963's ID once more...\n");
        _backend_reset();
        hal.scheduler->delay(100);
        _dump_registers();
    } 

    if (id_mismatch_count == 5) {
        _initialised = false;
        hal.console->printf("WRONG AK8963 DEVICE ID: 0x%x\n", (unsigned)deviceid);
        hal.scheduler->panic(PSTR("AK8963: bad DEVICE ID"));
    }

    _calibrate();

    _initialised = true;

#if AK8963_SELFTEST
    if (_self_test()) {    
        _initialised = true;
    } else {
        _initialised = false;
    }
#endif

    /* Register value to continuous measurement */
    _register_write(AK8963_CNTL1, AK8963_CONTINUOUS_MODE2 | _magnetometer_adc_resolution);

    _backend->sem_give();

    hal.scheduler->resume_timer_procs();
    hal.scheduler->register_timer_process( AP_HAL_MEMBERPROC(&AP_Compass_AK8963::_update));

    _start_conversion();

    _initialised = true;
    return _initialised;
}

void AP_Compass_AK8963::_update()
{
    if (hal.scheduler->micros() - _last_update_timestamp < 10000) {
        return;
    }

    if (!_backend->sem_take_nonblocking()) {
        return;
    }
    uint32_t timestamp = hal.scheduler->micros(); 

    switch (_state)
       {
        case CONVERSION:
            _start_conversion();
            _state = SAMPLE;
            break;
        case    SAMPLE:
            _collect_samples();
            _state = CONVERSION;
            break;
        case ERROR:
            break;
        default:
            break;
    }
    //error("state: %u %ld\n", (uint8_t) _state, hal.scheduler->micros() - timestamp);

    _last_update_timestamp = hal.scheduler->micros();
    _backend->sem_give();
}

bool AP_Compass_AK8963::_calibrate()
{
    error("CALIBRATTION START\n");
    _register_write(AK8963_CNTL1, AK8963_FUSE_MODE | _magnetometer_adc_resolution); /* Enable FUSE-mode in order to be able to read calibreation data */

    hal.scheduler->delay(10);

    uint8_t response[3];
    _register_read(AK8963_ASAX, 0x03, response);

    for (int i = 0; i < 3; i++) {
        float data = response[i];
        magnetometer_ASA[i] = ((data-128)/256+1);
        error("%d: %lf\n", i, magnetometer_ASA[i]);
    }

    error("CALIBRATTION END\n");

    return true;
}

bool AP_Compass_AK8963::read()
{
    if (!_initialised) {
        return false;
    }

    if (_accum_count == 0) {
        /* We're not ready to publish*/
        return true;
    }

    /* Update */
    _field[0].x = _mag_x_accum * magnetometer_ASA[0] / _accum_count;
    _field[0].y = _mag_y_accum * magnetometer_ASA[1] / _accum_count;
    _field[0].z = _mag_z_accum * magnetometer_ASA[2] / _accum_count;

    _mag_x_accum = _mag_y_accum = _mag_z_accum = 0;
    _accum_count = 0;

    // apply default board orientation for this compass type. This is
    // a noop on most boards
    _field[0].rotate(MAG_BOARD_ORIENTATION);

    // add user selectable orientation
    _field[0].rotate((enum Rotation)_orientation[0].get());

    if (!_external[0]) {
        // and add in AHRS_ORIENTATION setting if not an external compass
        _field[0].rotate(_board_orientation);
    }

    apply_corrections(_field[0],0);

#if 0
    float x = _field[0].x;
    float y = _field[0].y;
    float z = _field[0].z;

    error("%f %f %f\n", x, y, z);
#endif

    last_update = hal.scheduler->micros(); // record time of update

    return true;
}

void AP_Compass_AK8963::_start_conversion()
{
    static const uint8_t address = AK8963_INFO;
    /* Read registers from INFO through ST2 */
    static const uint8_t count = 0x09;

    _backend_init();
    _backend->write(MPUREG_USER_CTRL, BIT_USER_CTRL_I2C_MST_EN);    /* I2C Master mode */
    _backend->write(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG);  /* Set the I2C slave addres of AK8963 and set for read. */
    _backend->write(MPUREG_I2C_SLV0_REG, address); /* I2C slave 0 register address from where to begin data transfer */
    _backend->write(MPUREG_I2C_SLV0_CTRL, I2C_SLV0_EN | count); /* Enable I2C and set @count byte */
}

void AP_Compass_AK8963::_collect_samples()
{
    if (!_initialised) {
        return;
    }

    if (!_read_raw()) {
        error("_read_raw failed\n");
    } else {
        _mag_x_accum += _mag_x;
        _mag_y_accum += _mag_y;
        _mag_z_accum += _mag_z;
        _accum_count++;
        if (_accum_count == 10) {
             _mag_x_accum /= 2;
             _mag_y_accum /= 2;
             _mag_z_accum /= 2;
             _accum_count = 5;
        }
    }
}
