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

   -- Adapted from Victor Mayoral Vilches's legacy driver under folder LSM9DS0
*/

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "AP_InertialSensor_LSM9DS0.h"
#include "../AP_HAL_Linux/GPIO.h"

extern const AP_HAL::HAL& hal;

#define LSM9DS0_G_WHOAMI    0xD4
#define LSM9DS0_XM_WHOAMI   0x49

/*
 * If data-ready GPIO pins are not defined, the fallback approach used is to
 * check if there's new data ready by reading the status register. It is
 * *strongly* recommended to use data-ready GPIO pins for performance reasons.
 */
#ifndef DRDY_GPIO_PIN_A
    #define DRDY_GPIO_PIN_A 0
#endif
#ifndef DRDY_GPIO_PIN_G
    #define DRDY_GPIO_PIN_G 0
#endif

////////////////////////////
// LSM9DS0 Gyro Registers //
////////////////////////////
#define WHO_AM_I_G          0x0F
#define CTRL_REG1_G         0x20
#define CTRL_REG2_G         0x21
#define CTRL_REG3_G         0x22
#define CTRL_REG4_G         0x23
#define CTRL_REG5_G         0x24
#define REFERENCE_G         0x25
#define STATUS_REG_G        0x27
#define OUT_X_L_G           0x28
#define OUT_X_H_G           0x29
#define OUT_Y_L_G           0x2A
#define OUT_Y_H_G           0x2B
#define OUT_Z_L_G           0x2C
#define OUT_Z_H_G           0x2D
#define FIFO_CTRL_REG_G     0x2E
#define FIFO_SRC_REG_G      0x2F
#define INT1_CFG_G          0x30
#define INT1_SRC_G          0x31
#define INT1_THS_XH_G       0x32
#define INT1_THS_XL_G       0x33
#define INT1_THS_YH_G       0x34
#define INT1_THS_YL_G       0x35
#define INT1_THS_ZH_G       0x36
#define INT1_THS_ZL_G       0x37
#define INT1_DURATION_G     0x38

//////////////////////////////////////////
// LSM9DS0 Accel/Magneto (XM) Registers //
//////////////////////////////////////////
#define OUT_TEMP_L_XM       0x05
#define OUT_TEMP_H_XM       0x06
#define STATUS_REG_M        0x07
#define OUT_X_L_M           0x08
#define OUT_X_H_M           0x09
#define OUT_Y_L_M           0x0A
#define OUT_Y_H_M           0x0B
#define OUT_Z_L_M           0x0C
#define OUT_Z_H_M           0x0D
#define WHO_AM_I_XM         0x0F
#define INT_CTRL_REG_M      0x12
#define INT_SRC_REG_M       0x13
#define INT_THS_L_M         0x14
#define INT_THS_H_M         0x15
#define OFFSET_X_L_M        0x16
#define OFFSET_X_H_M        0x17
#define OFFSET_Y_L_M        0x18
#define OFFSET_Y_H_M        0x19
#define OFFSET_Z_L_M        0x1A
#define OFFSET_Z_H_M        0x1B
#define REFERENCE_X         0x1C
#define REFERENCE_Y         0x1D
#define REFERENCE_Z         0x1E
#define CTRL_REG0_XM        0x1F
#define CTRL_REG1_XM        0x20
#define CTRL_REG2_XM        0x21
#define CTRL_REG3_XM        0x22
#define CTRL_REG4_XM        0x23
#define CTRL_REG5_XM        0x24
#define CTRL_REG6_XM        0x25
#define CTRL_REG7_XM        0x26
#define STATUS_REG_A        0x27
#define OUT_X_L_A           0x28
#define OUT_X_H_A           0x29
#define OUT_Y_L_A           0x2A
#define OUT_Y_H_A           0x2B
#define OUT_Z_L_A           0x2C
#define OUT_Z_H_A           0x2D
#define FIFO_CTRL_REG       0x2E
#define FIFO_SRC_REG        0x2F
#define INT_GEN_1_REG       0x30
#define INT_GEN_1_SRC       0x31
#define INT_GEN_1_THS       0x32
#define INT_GEN_1_DURATION  0x33
#define INT_GEN_2_REG       0x34
#define INT_GEN_2_SRC       0x35
#define INT_GEN_2_THS       0x36
#define INT_GEN_2_DURATION  0x37
#define CLICK_CFG           0x38
#define CLICK_SRC           0x39
#define CLICK_THS           0x3A
#define TIME_LIMIT          0x3B
#define TIME_LATENCY        0x3C
#define TIME_WINDOW         0x3D
#define ACT_THS             0x3E
#define ACT_DUR             0x3F

AP_InertialSensor_LSM9DS0::AP_InertialSensor_LSM9DS0(AP_InertialSensor &imu):
	AP_InertialSensor_Backend(imu),
    _drdy_pin_a(NULL),
    _drdy_pin_g(NULL),
    _last_accel_filter_hz(-1),
    _last_gyro_filter_hz(-1),
    _accel_filter(800, 15),
    _gyro_filter(760, 15),
    _gyro_sample_available(false),
    _accel_sample_available(false)
{
    _product_id = AP_PRODUCT_ID_NONE;
}

AP_InertialSensor_Backend *AP_InertialSensor_LSM9DS0::detect(AP_InertialSensor &_imu)
{
    AP_InertialSensor_LSM9DS0 *sensor = new AP_InertialSensor_LSM9DS0(_imu);

    if (sensor == NULL) {
        return NULL;
    }

    if (!sensor->_init_sensor()) {
        delete sensor;
        return NULL;
    }

    return sensor;
}

bool AP_InertialSensor_LSM9DS0::_init_sensor()
{
    _gyro_spi = hal.spi->device(AP_HAL::SPIDevice_LSM9DS0_G);
    _accel_spi = hal.spi->device(AP_HAL::SPIDevice_LSM9DS0_AM);
    _spi_sem = _gyro_spi->get_semaphore(); /* same semaphore for both */

#if DRDY_GPIO_PIN_A != 0
    _drdy_pin_a = hal.gpio->channel(DRDY_GPIO_PIN_A);
    if (_drdy_pin_a == NULL) {
        hal.scheduler->panic("LSM9DS0: null accel data-ready GPIO channel\n");
    }
#endif

#if DRDY_GPIO_PIN_G != 0
    _drdy_pin_g = hal.gpio->channel(DRDY_GPIO_PIN_G);
    if (_drdy_pin_g == NULL) {
        hal.scheduler->panic("LSM9DS0: null gyro data-ready GPIO channel\n");
    }
#endif

    hal.scheduler->suspend_timer_procs();

    uint8_t whoami;

    bool whoami_ok = true;

    whoami = _register_read_g(WHO_AM_I_G);
    if (whoami != LSM9DS0_G_WHOAMI) {
        hal.console->printf("LSM9DS0: unexpected gyro WHOAMI 0x%x\n", (unsigned)whoami);
        whoami_ok = false;
    }

    whoami = _register_read_xm(WHO_AM_I_XM);
    if (whoami != LSM9DS0_XM_WHOAMI) {
        hal.console->printf("LSM9DS0: unexpected acc/mag  WHOAMI 0x%x\n", (unsigned)whoami);
        whoami_ok = false;
    }

    if (!whoami_ok) return false;

    uint8_t tries = 0;
    bool a_ready = false;
    bool g_ready = false;
    do {
        bool success = _hardware_init();
        if (success) {
            hal.scheduler->delay(10);
            if (!_spi_sem->take(100)) {
                hal.console->printf("LSM9DS0: Unable to get semaphore\n");
                return false;
            }
            if (!a_ready) {
                a_ready = _accel_data_ready();
            }
            if (!g_ready) {
                g_ready = _gyro_data_ready();
            }
            if (a_ready && g_ready) {
                _spi_sem->give();
                break;
            } else {
                hal.console->printf("LSM9DS0 startup failed: no data ready\n");
            }
            _spi_sem->give();
        }
        if (tries++ > 5) {
            hal.console->printf("failed to boot LSM9DS0 5 times\n");
            return false;
        }
    } while (1);

    hal.scheduler->resume_timer_procs();

    _gyro_instance = _imu.register_gyro();
    _accel_instance = _imu.register_accel();

#if LSM9DS0_DEBUG
    _dump_registers();
#endif

    /* start the timer process to read samples */
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_LSM9DS0::_poll_data, void));

    return true;
}

bool AP_InertialSensor_LSM9DS0::_hardware_init()
{
    if (!_spi_sem->take(100)) {
        hal.console->printf("LSM9DS0: Unable to get semaphore\n");
        return false;
    }

    _gyro_spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_LOW);
    _accel_spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_LOW);

    _gyro_init();
    _accel_init();

    _gyro_spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_HIGH);
    _accel_spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_HIGH);

    _spi_sem->give();

    return true;
}

uint8_t AP_InertialSensor_LSM9DS0::_register_read_xm( uint8_t reg )
{
    uint8_t addr = reg | 0x80; /* set read bit */

    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = addr;
    tx[1] = 0;
    _accel_spi->transaction(tx, rx, 2);

    return rx[1];
}

uint8_t AP_InertialSensor_LSM9DS0::_register_read_g( uint8_t reg )
{
    uint8_t addr = reg | 0x80; /* set read bit */

    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = addr;
    tx[1] = 0;
    _gyro_spi->transaction(tx, rx, 2);

    return rx[1];
}


void AP_InertialSensor_LSM9DS0::_register_write_xm(uint8_t reg, uint8_t val)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = reg;
    tx[1] = val;
    _accel_spi->transaction(tx, rx, 2);
}

void AP_InertialSensor_LSM9DS0::_register_write_g(uint8_t reg, uint8_t val)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = reg;
    tx[1] = val;
    _gyro_spi->transaction(tx, rx, 2);
}

void AP_InertialSensor_LSM9DS0::_gyro_init()
{
    /*
     * CTRL_REG1_G sets output data rate, bandwidth, power-down and enables
     *
     * Bits[7:0]: DR1 DR0 BW1 BW0 PD Zen Xen Yen
     * DR[1:0] - Output data rate selection
     *     00=95Hz, 01=190Hz, 10=380Hz, 11=760Hz
     * BW[1:0] - Bandwidth selection (sets cutoff frequency)
     *      Value depends on ODR. See datasheet table 21.
     * PD - Power down enable (0=power down mode, 1=normal or sleep mode)
     * Zen, Xen, Yen - Axis enable (o=disabled, 1=enabled)
     *
     * Data rate of 760Hz, cutoff of 50Hz, Normal mode, enable all axes
     */
    _register_write_g(CTRL_REG1_G, 0xEF);
    hal.scheduler->delay(1);

    /*
     * CTRL_REG2_G sets up the HPF
     *
     * Bits[7:0]: 0 0 HPM1 HPM0 HPCF3 HPCF2 HPCF1 HPCF0
     * HPM[1:0] - High pass filter mode selection
     *     00=normal (reset reading HP_RESET_FILTER, 01=ref signal for filtering,
     *     10=normal, 11=autoreset on interrupt
     * HPCF[3:0] - High pass filter cutoff frequency
     *     Value depends on data rate. See datasheet table 26.
     *
     * Normal mode, high cutoff frequency
     */
    _register_write_g(CTRL_REG2_G, 0x00);
    hal.scheduler->delay(1);

    /*
     * CTRL_REG3_G sets up interrupt and DRDY_G pins
     *
     * Bits[7:0]: I1_IINT1 I1_BOOT H_LACTIVE PP_OD I2_DRDY I2_WTM I2_ORUN I2_EMPTY
     * I1_INT1 - Interrupt enable on INT_G pin (0=disable, 1=enable)
     * I1_BOOT - Boot status available on INT_G (0=disable, 1=enable)
     * H_LACTIVE - Interrupt active configuration on INT_G (0:high, 1:low)
     * PP_OD - Push-pull/open-drain (0=push-pull, 1=open-drain)
     * I2_DRDY - Data ready on DRDY_G (0=disable, 1=enable)
     * I2_WTM - FIFO watermark interrupt on DRDY_G (0=disable 1=enable)
     * I2_ORUN - FIFO overrun interrupt on DRDY_G (0=disable 1=enable)
     * I2_EMPTY - FIFO empty interrupt on DRDY_G (0=disable 1=enable)
     *
     * Gyro data ready on DRDY_G
     */
    _register_write_g(CTRL_REG3_G, 0x08);
    hal.scheduler->delay(1);

    /*
     * CTRL_REG4_G sets the scale, update mode
     *
     * Bits[7:0] - BDU BLE FS1 FS0 - ST1 ST0 SIM
     * BDU - Block data update (0=continuous, 1=output not updated until read
     * BLE - Big/little endian (0=data LSB @ lower address, 1=LSB @ higher add)
     * FS[1:0] - Full-scale selection
     *     00=245dps, 01=500dps, 10=2000dps, 11=2000dps
     * ST[1:0] - Self-test enable
     *     00=disabled, 01=st 0 (x+, y-, z-), 10=undefined, 11=st 1 (x-, y+, z+)
     * SIM - SPI serial interface mode select
     *     0=4 wire, 1=3 wire
     *
     * BUD, set scale to 2000 dps
     */
    _register_write_g(CTRL_REG4_G, 0xB0);
    _set_gyro_scale(G_SCALE_2000DPS);
    hal.scheduler->delay(1);

    /*
     * CTRL_REG5_G sets up the FIFO, HPF, and INT1
     *
     * Bits[7:0] - BOOT FIFO_EN - HPen INT1_Sel1 INT1_Sel0 Out_Sel1 Out_Sel0
     * BOOT - Reboot memory content (0=normal, 1=reboot)
     * FIFO_EN - FIFO enable (0=disable, 1=enable)
     * HPen - HPF enable (0=disable, 1=enable)
     * INT1_Sel[1:0] - Int 1 selection configuration
     * Out_Sel[1:0] - Out selection configuration
     */
    _register_write_g(CTRL_REG5_G, 0x00);
    hal.scheduler->delay(1);
}

void AP_InertialSensor_LSM9DS0::_accel_init()
{
    /*
     * CTRL_REG0_XM (0x1F) (Default value: 0x00)
     *
     * Bits (7-0): BOOT FIFO_EN WTM_EN 0 0 HP_CLICK HPIS1 HPIS2
     * BOOT - Reboot memory content (0: normal, 1: reboot)
     * FIFO_EN - Fifo enable (0: disable, 1: enable)
     * WTM_EN - FIFO watermark enable (0: disable, 1: enable)
     * HP_CLICK - HPF enabled for click (0: filter bypassed, 1: enabled)
     * HPIS1 - HPF enabled for interrupt generator 1 (0: bypassed, 1: enabled)
     * HPIS2 - HPF enabled for interrupt generator 2 (0: bypassed, 1 enabled)
     */
    _register_write_xm(CTRL_REG0_XM, 0x00);
    hal.scheduler->delay(1);

    /*
     * CTRL_REG1_XM (0x20) (Default value: 0x07)
     *
     * Bits (7-0): AODR3 AODR2 AODR1 AODR0 BDU AZEN AYEN AXEN
     * AODR[3:0] - select the acceleration data rate:
     *     0000=power down, 0001=3.125Hz, 0010=6.25Hz, 0011=12.5Hz,
     *     0100=25Hz, 0101=50Hz, 0110=100Hz, 0111=200Hz, 1000=400Hz,
     *     1001=800Hz, 1010=1600Hz, (remaining combinations undefined).
     * BDU - block data update for accel AND mag
     *     0: Continuous update
     *     1: Output registers aren't updated until MSB and LSB have been read.
     * AZEN, AYEN, and AXEN - Acceleration x/y/z-axis enabled.
     *     0: Axis disabled, 1: Axis enabled
     *
     * 800Hz data rate, BDU enabled, x,y,z all enabled
     */
    _register_write_xm(CTRL_REG1_XM, 0x9F);
    hal.scheduler->delay(1);

    /*
     * CTRL_REG2_XM (0x21) (Default value: 0x00)
     *
     * Bits (7-0): ABW1 ABW0 AFS2 AFS1 AFS0 AST1 AST0 SIM
     * ABW[1:0] - Accelerometer anti-alias filter bandwidth
     *     00=773Hz, 01=194Hz, 10=362Hz, 11=50Hz
     * AFS[2:0] - Accel full-scale selection
     *     000=+/-2g, 001=+/-4g, 010=+/-6g, 011=+/-8g, 100=+/-16g
     * AST[1:0] - Accel self-test enable
     *     00=normal (no self-test), 01=positive st, 10=negative st, 11=not allowed
     * SIM - SPI mode selection
     *     0=4-wire, 1=3-wire
     *
     * Set filter bandwidth to 50Hz and scale to 16g
     */
    _register_write_xm(CTRL_REG2_XM, 0xE0);
    _set_accel_scale(A_SCALE_16G);
    hal.scheduler->delay(1);

    /*
     * CTRL_REG3_XM is used to set interrupt generators on INT1_XM
     *
     * Bits (7-0): P1_BOOT P1_TAP P1_INT1 P1_INT2 P1_INTM P1_DRDYA P1_DRDYM P1_EMPTY
     *
     * Accel data ready on INT1
     */
    _register_write_xm(CTRL_REG3_XM, 0x04);
    hal.scheduler->delay(1);
}

void AP_InertialSensor_LSM9DS0::_set_gyro_scale(gyro_scale scale)
{
    /* scales values from datasheet in mdps/digit */
    switch (scale) {
    case G_SCALE_245DPS:
        _gyro_scale = 8.75;
        break;
    case G_SCALE_500DPS:
        _gyro_scale = 17.50;
        break;
    case G_SCALE_2000DPS:
        _gyro_scale = 70;
        break;
    }

    _gyro_scale /= 1000; /* convert mdps/digit to dps/digit */
    _gyro_scale *= DEG_TO_RAD; /* convert dps/digit to (rad/s)/digit */
}

void AP_InertialSensor_LSM9DS0::_set_accel_scale(accel_scale scale)
{
    /*
     * Possible accelerometer scales (and their register bit settings) are:
     * 2 g (000), 4g (001), 6g (010) 8g (011), 16g (100). Here's a bit of an
     * algorithm to calculate g/(ADC tick) based on that 3-bit value:
     */
    _accel_scale = (((float) scale + 1.0f) * 2.0f) / 32768.0f;
    if (scale == A_SCALE_16G) {
        _accel_scale = 0.000732; /* the datasheet shows an exception for +-16G */
    }
    _accel_scale *= GRAVITY_MSS; /* convert to G/LSB to (m/s/s)/LSB */
}

/**
 * Timer process to poll for new data from the LSM9DS0.
 */
void AP_InertialSensor_LSM9DS0::_poll_data()
{
    if (!_spi_sem->take_nonblocking()) {
        /*
         *  the semaphore being busy is an expected condition when the
         *  mainline code is calling wait_for_sample() which will
         *  grab the semaphore. We return now and rely on the mainline
         *  code grabbing the latest sample.
         */
        return;
    }

    if (_gyro_data_ready()) {
        _read_data_transaction_g();
    }

    if (_accel_data_ready()) {
        _read_data_transaction_a();
    }

    _spi_sem->give();
}

#if DRDY_GPIO_PIN_A != 0
bool AP_InertialSensor_LSM9DS0::_accel_data_ready()
{
    return _drdy_pin_a->read() != 0;
}
#else
bool AP_InertialSensor_LSM9DS0::_accel_data_ready()
{
    uint8_t status = _register_read_xm(STATUS_REG_A);
    return status & STATUS_REG_A_ZYXADA;
}
#endif /* DRDY_GPIO_PIN_A != 0 */

#if DRDY_GPIO_PIN_G != 0
bool AP_InertialSensor_LSM9DS0::_gyro_data_ready()
{
    return _drdy_pin_g->read() != 0;
}
#else
bool AP_InertialSensor_LSM9DS0::_gyro_data_ready()
{
    uint8_t status = _register_read_xm(STATUS_REG_G);
    return status & STATUS_REG_G_ZYXDA;
}
#endif /* DRDY_GPIO_PIN_G != 0 */

void AP_InertialSensor_LSM9DS0::_accel_raw_data(struct sensor_raw_data *raw_data)
{
    struct __attribute__((packed)) {
        uint8_t reg;
        struct sensor_raw_data data;
    } tx = {.reg = OUT_X_L_A | 0xC0, .data = {}}, rx;

    _accel_spi->transaction((uint8_t *)&tx, (uint8_t *)&rx, 7);
    *raw_data = rx.data;
}

void AP_InertialSensor_LSM9DS0::_gyro_raw_data(struct sensor_raw_data *raw_data)
{
    struct __attribute__((packed)) {
        uint8_t reg;
        struct sensor_raw_data data;
    } tx = {.reg = OUT_X_L_G | 0xC0, .data = {}}, rx;

    _gyro_spi->transaction((uint8_t *)&tx, (uint8_t *)&rx, 7);
    *raw_data = rx.data;
}

void AP_InertialSensor_LSM9DS0::_read_data_transaction_a()
{
    struct sensor_raw_data raw_data;
    _accel_raw_data(&raw_data);

    Vector3f accel_data(raw_data.x, -raw_data.y, -raw_data.z);
    _accel_filtered = _accel_filter.apply(accel_data);
    _accel_sample_available = true;
}

/*
  read from the data registers and update filtered data
 */
void AP_InertialSensor_LSM9DS0::_read_data_transaction_g()
{
    struct sensor_raw_data raw_data;
    _gyro_raw_data(&raw_data);

    Vector3f gyro_data(raw_data.x, -raw_data.y, -raw_data.z);
    _gyro_filtered = _gyro_filter.apply(gyro_data);
    _gyro_sample_available = true;
}

bool AP_InertialSensor_LSM9DS0::update()
{
    Vector3f gyro = _gyro_filtered;
    Vector3f accel = _accel_filtered;

    _accel_sample_available = false;
    _gyro_sample_available = false;

    accel *= _accel_scale;
    gyro *= _gyro_scale;

    _publish_gyro(_gyro_instance, gyro);
    _publish_accel(_accel_instance, accel);

    if (_last_accel_filter_hz != _accel_filter_cutoff()) {
        _set_accel_filter(_accel_filter_cutoff());
        _last_accel_filter_hz = _accel_filter_cutoff();
    }

    if (_last_gyro_filter_hz != _gyro_filter_cutoff()) {
        _set_gyro_filter(_gyro_filter_cutoff());
        _last_gyro_filter_hz = _gyro_filter_cutoff();
    }

    return true;
}

/*
  set the accel filter frequency
 */
void AP_InertialSensor_LSM9DS0::_set_accel_filter(uint8_t filter_hz)
{
    _accel_filter.set_cutoff_frequency(800, filter_hz);
}

/*
  set the gyro filter frequency
 */
void AP_InertialSensor_LSM9DS0::_set_gyro_filter(uint8_t filter_hz)
{
    _gyro_filter.set_cutoff_frequency(760, filter_hz);
}

#if LSM9DS0_DEBUG
/* dump all config registers - used for debug */
void AP_InertialSensor_LSM9DS0::_dump_registers(void)
{
    hal.console->println_P(PSTR("LSM9DS0 registers:"));
    hal.console->println_P(PSTR("Gyroscope registers:"));
    const uint8_t first = OUT_TEMP_L_XM;
    const uint8_t last = ACT_DUR;
    for (uint8_t reg=first; reg<=last; reg++) {
        uint8_t v = _register_read_g(reg);
        hal.console->printf_P(PSTR("%02x:%02x "), (unsigned)reg, (unsigned)v);
        if ((reg - (first-1)) % 16 == 0) {
            hal.console->println();
        }
    }
    hal.console->println();

    hal.console->println_P(PSTR("Accelerometer and Magnetometers registers:"));
    for (uint8_t reg=first; reg<=last; reg++) {
        uint8_t v = _register_read_xm(reg);
        hal.console->printf_P(PSTR("%02x:%02x "), (unsigned)reg, (unsigned)v);
        if ((reg - (first-1)) % 16 == 0) {
            hal.console->println();
        }
    }
    hal.console->println();

}
#endif

#endif /* CONFIG_HAL_BOARD == HAL_BOARD_LINUX */
