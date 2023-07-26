/*
 * Copyright (C) 2019 Gunar Schorcht
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_esp_common
 * @ingroup     drivers_periph_i2c
 * @{
 *
 * @file
 * @brief       Low-level I2C driver software implementation using for ESP SoCs
 *
 * @author      Gunar Schorcht <gunar@schorcht.net>
 *
 * @}
 */

/*
   PLEASE NOTE:

   The implementation bases on the bit-banging I2C master implementation as
   described in [wikipedia](https://en.wikipedia.org/wiki/I%C2%B2C#Example_of_bit-banging_the_I%C2%B2C_master_protocol).
*/

#include "esp_err.h"
#define DEBUG printf

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>

#include "esp_attr.h"

#include "rom/ets_sys.h" // seems to work for both...for now..deprecated

#include "soc/gpio_reg.h"
#include "soc/gpio_struct.h"


// IMPORTS FROM esp-idf hw implem
#include <string.h>
#include <stdio.h>
#include "esp_types.h"
#include "esp_attr.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "malloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/xtensa_api.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "soc/dport_reg.h"
#include "esp_pm.h"
#include "soc/soc_memory_layout.h"
//#include "hal/i2c_hal.h"
#include "soc/i2c_periph.h"
#include "driver/i2c.h"
#include "driver/periph_ctrl.h"
#include "lwip/netdb.h"
#include "i2c_sw.h"

/* max clock stretching counter */
#define I2C_CLOCK_STRETCH 200

/* gpio access macros */
#define GPIO_SET(l,h,b) if (b < 32) GPIO.l =  BIT(b); else GPIO.h.val =  BIT(b-32)
#define GPIO_GET(l,h,b) ((b < 32) ? GPIO.l & BIT(b) : GPIO.h.val & BIT(b-32))

/* to ensure that I2C is always optimized with -O2 to use the defined delays */
#pragma GCC optimize ("O2")

static const uint32_t _i2c_delays[][3] =
{
    /* values specify one half-period and are only valid for -O2 option     */
    /* value = [period - 0.25 us (240 MHz) / 0.5us(160MHz) / 1.0us(80MHz)]  */
    /*         * cycles per second / 2                                      */
    /* 1 us = 16 cycles (80 MHz) / 32 cycles (160 MHz) / 48 cycles (240)    */
    /* values for             80,  160,  240 MHz                            */
    [I2C_SPEED_LOW]       = {790, 1590, 2390}, /*   10 kbps (period 100 us) */
    [I2C_SPEED_NORMAL]    = { 70,  150,  230}, /*  100 kbps (period 10 us)  */
    [I2C_SPEED_FAST]      = { 11,   31,   51}, /*  400 kbps (period 2.5 us) */
    [I2C_SPEED_FAST_PLUS] = {  0,    7,   15}, /*    1 Mbps (period 1 us)   */
    [I2C_SPEED_HIGH]      = {  0,    0,    0}  /*  3.4 Mbps (period 0.3 us) not working */
};

/* forward declaration of internal functions */

static inline void _i2c_delay(_i2c_bus_t* bus);
static inline bool _i2c_scl_read(_i2c_bus_t* bus);
static inline bool _i2c_sda_read(_i2c_bus_t* bus);
static inline void _i2c_scl_high(_i2c_bus_t* bus);
static inline void _i2c_scl_low(_i2c_bus_t* bus);
static inline void _i2c_sda_high(_i2c_bus_t* bus);
static inline void _i2c_sda_low(_i2c_bus_t* bus);
static int _i2c_start_cond(_i2c_bus_t* bus);
static int _i2c_stop_cond(_i2c_bus_t* bus);
static int _i2c_write_bit(_i2c_bus_t* bus, bool bit);
static int _i2c_read_bit(_i2c_bus_t* bus, bool* bit);
static int _i2c_write_byte(_i2c_bus_t* bus, uint8_t byte);
static int _i2c_read_byte(_i2c_bus_t* bus, uint8_t* byte, bool ack);
static int _i2c_arbitration_lost(_i2c_bus_t* bus, const char* func);
static void _i2c_abort(_i2c_bus_t* bus, const char* func);
static void _i2c_clear(_i2c_bus_t* bus);

/* implementation of i2c interface */

void i2c_init(_i2c_bus_t* bus)
{
    if (bus->speed == I2C_SPEED_HIGH) {
        DEBUG("i2c I2C_SPEED_HIGH is not supported\n");
        return;
    }

    bus->scl_bit = BIT(bus->scl); /* store bit mask for faster access */
    bus->sda_bit = BIT(bus->sda); /* store bit mask for faster access */
    bus->started = false; /* for handling of repeated start condition */

    switch (ets_get_cpu_frequency()) {
        case  80: bus->delay = _i2c_delays[bus->speed][0]; break;
        case 160: bus->delay = _i2c_delays[bus->speed][1]; break;
        case 240: bus->delay = _i2c_delays[bus->speed][2]; break;
        default : DEBUG("i2c I2C software implementation is not "
                                      "supported for this CPU frequency: %d MHz\n",
                               ets_get_cpu_frequency());
            return;
    }

    DEBUG("%s: scl=%d sda=%d speed=%d\n", __func__,
          bus->scl, bus->sda, bus->speed);

    /* reset the GPIO usage if the pins were used for I2C before */
    gpio_reset_pin(bus->scl);
    gpio_reset_pin(bus->sda);

    /* Configure and initialize SDA and SCL pin. */
    /*
     * ESP32 pins are used in input/output mode with open-drain output driver.
     * Signal levels are then realized as following:
     *
     * - HIGH: Output value 1 lets the pin floating and is pulled-up to high.
     * - LOW : Output value 0 actively drives the pin to low.
     */
    gpio_config_t gpio_conf = {
        .pin_bit_mask = bus->scl_bit | bus->sda_bit,
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    esp_err_t err = gpio_config(&gpio_conf);
    assert(!err);

    /* set SDA and SCL to be floating and pulled-up to high */
    _i2c_sda_high(bus);
    _i2c_scl_high(bus);

    /* clear the bus if necessary (SDA is driven permanently low) */
    _i2c_clear(bus);
}

int IRAM_ATTR i2c_read_bytes(_i2c_bus_t* bus, uint16_t addr, void *data, size_t len, uint8_t flags)
{
    int res = 0;

    /* send START condition and address if I2C_NOSTART is not set */
    if (!(flags & I2C_NOSTART)) {

        /* START condition */
        if ((res = _i2c_start_cond(bus)) != 0) {
            return res;
        }

        /* send 10 bit or 7 bit address */
        if (flags & I2C_ADDR10) {
            /* prepare 10 bit address bytes */
            uint8_t addr1 = 0xf0 | (addr & 0x0300) >> 7 | I2C_READ;
            uint8_t addr2 = addr & 0xff;
            /* send address bytes with read flag */
            if ((res = _i2c_write_byte(bus, addr1)) != 0 ||
                (res = _i2c_write_byte(bus, addr2)) != 0) {
                /* abort transfer */
                _i2c_abort(bus, __func__);
                return -ENXIO;
            }
        }
        else {
            /* send address byte with read flag */
            if ((res = _i2c_write_byte(bus, (addr << 1 | I2C_READ))) != 0) {
                /* abort transfer */
                _i2c_abort(bus, __func__);
                return -ENXIO;
            }
        }
    }

    /* receive bytes if send address was successful */
    for (unsigned int i = 0; i < len; i++) {
        if ((res = _i2c_read_byte(bus, &(((uint8_t*)data)[i]), i < len-1)) != 0) {
            /* abort transfer */
            _i2c_abort(bus, __func__);
            return res;
        }
    }

    /* send STOP condition if I2C_NOSTOP flag is not set */
    if (!(flags & I2C_NOSTOP)) {
        res = _i2c_stop_cond(bus);
    }

    return res;
}

int IRAM_ATTR i2c_write_bytes(_i2c_bus_t* bus, uint16_t addr, const void *data, size_t len, uint8_t flags)
{
    int res = 0;

    /*  if I2C_NOSTART is not set, send START condition and ADDR */
    if (!(flags & I2C_NOSTART)) {

        /* START condition */
        if ((res = _i2c_start_cond(bus)) != 0) {
            return res;
        }

        /* send 10 bit or 7 bit address */
        if (flags & I2C_ADDR10) {
            /* prepare 10 bit address bytes */
            uint8_t addr1 = 0xf0 | (addr & 0x0300) >> 7;
            uint8_t addr2 = addr & 0xff;
            /* send address bytes without read flag */
            if ((res = _i2c_write_byte(bus, addr1)) != 0 ||
                (res = _i2c_write_byte(bus, addr2)) != 0) {
                /* abort transfer */
                _i2c_abort(bus, __func__);
                return -ENXIO;
            }
        }
        else {
            /* send address byte without read flag */
            if ((res = _i2c_write_byte(bus, addr << 1)) != 0) {
                /* abort transfer */
                _i2c_abort(bus, __func__);
                return -ENXIO;
            }
        }
    }

    /* send bytes if send address was successful */
    for (unsigned int i = 0; i < len; i++) {
        if ((res = _i2c_write_byte(bus, ((uint8_t*)data)[i])) != 0) {
            /* abort transfer */
            _i2c_abort(bus, __func__);
            return res;
        }
    }

    /* send STOP condition if I2C_NOSTOP flag is not set */
    if (!(flags & I2C_NOSTOP)) {
        res = _i2c_stop_cond(bus);
    }

    return res;
}

/* --- internal functions --- */

static inline void _i2c_delay(_i2c_bus_t* bus)
{
    /* produces a delay */
    uint32_t cycles = bus->delay;
    if (cycles) {
        __asm__ volatile ("1: _addi.n  %0, %0, -1 \n"
                          "   bnez     %0, 1b     \n" : "=r" (cycles) : "0" (cycles));
    }
}

/*
 * Please note: SDA and SDL pins are used in GPIO_OD_PU mode
 *              (open-drain with pull-ups).
 *
 * Setting a pin which is in open-drain mode leaves the pin floating and
 * the signal is pulled up to high. The signal can then be actively driven
 * to low by a slave. A read operation returns the current signal at the pin.
 *
 * Clearing a pin which is in open-drain mode actively drives the signal to
 * low.
 */

static inline bool _i2c_scl_read(_i2c_bus_t* bus)
{
    // return gpio_get_level(bus->scl);
    /* read SCL status (pin is in open-drain mode and set) */
    return GPIO_GET(in, in1, bus->scl);
}

static inline bool _i2c_sda_read(_i2c_bus_t* bus)
{
    // return gpio_get_level(bus->sda);
    /* read SDA status (pin is in open-drain mode and set) */
    return GPIO_GET(in, in1, bus->sda);
}

static inline void _i2c_scl_high(_i2c_bus_t* bus)
{
    // gpio_set_level(bus->scl, 1);
    // return;
    /* set SCL signal high (pin is in open-drain mode and pulled-up) */
    GPIO_SET(out_w1ts, out1_w1ts, bus->scl);
}

static inline void _i2c_scl_low(_i2c_bus_t* bus)
{
    // gpio_set_level(bus->scl, 0);
    // return;
    /* set SCL signal low (actively driven to low) */
    GPIO_SET(out_w1tc, out1_w1tc, bus->scl);
}

static inline void _i2c_sda_high(_i2c_bus_t* bus)
{
    // gpio_set_level(bus->sda, 1);
    // return;
    /* set SDA signal high (pin is in open-drain mode and pulled-up) */
    GPIO_SET(out_w1ts, out1_w1ts, bus->sda);
}

static inline void _i2c_sda_low(_i2c_bus_t* bus)
{
    // gpio_set_level(bus->sda, 0);
    // return;
    /* set SDA signal low (actively driven to low) */
    GPIO_SET(out_w1tc, out1_w1tc, bus->sda);
}

static void _i2c_clear(_i2c_bus_t* bus)
{
    //DEBUG("%s: dev=%u\n", __func__, bus->dev);

    /**
     * Sometimes a slave blocks and drives the SDA line permanently low.
     * Send some clock pulses in that case (10 at maximum)
     */

    /*
     * If SDA is low while SCL is high for 10 half cycles, it is not an
     * arbitration lost but a bus lock.
     */
    int count = 10;
    while (!_i2c_sda_read(bus) && _i2c_scl_read(bus) && count) {
        count--;
        _i2c_delay(bus);
    }

    if (count) {
        /* was not a bus lock */
        return;
    }

    /* send 10 clock pulses in case of bus lock */
    count = 10;
    while (!_i2c_sda_read(bus) && count--) {
        _i2c_scl_low(bus);
        _i2c_delay(bus);
        _i2c_scl_high(bus);
        _i2c_delay(bus);
    }
}

static void _i2c_abort(_i2c_bus_t* bus, const char* func)
{
    //DEBUG("%s: dev=%u\n", func, bus->dev);

    /* reset SCL and SDA to passive HIGH (floating and pulled-up) */
    _i2c_sda_high(bus);
    _i2c_scl_high(bus);

    /* reset repeated start indicator */
    bus->started = false;

    /* clear the bus if necessary (SDA is driven permanently low) */
    _i2c_clear(bus);
}

static IRAM_ATTR int _i2c_arbitration_lost(_i2c_bus_t* bus, const char* func)
{
    //DEBUG("%s: arbitration lost dev=%u\n", func, bus->dev);

    /* reset SCL and SDA to passive HIGH (floating and pulled-up) */
    _i2c_sda_high(bus);
    _i2c_scl_high(bus);

    /* reset repeated start indicator */
    bus->started = false;

    /* clear the bus if necessary (SDA is driven permanently low) */
    _i2c_clear(bus);

    return -EAGAIN;
}

static IRAM_ATTR int _i2c_start_cond(_i2c_bus_t* bus)
{
    /*
     * send start condition
     * on entry: SDA and SCL are set to be floating and pulled-up to high
     * on exit : SDA and SCL are actively driven to low
     */

    int res = 0;

    if (bus->started) {
        /* prepare the repeated start condition */

        /* SDA = passive HIGH (floating and pulled-up) */
        _i2c_sda_high(bus);

        /* t_VD;DAT not necessary */
        /* _i2c_delay(bus); */

        /* SCL = passive HIGH (floating and pulled-up) */
        _i2c_scl_high(bus);

        /* clock stretching, wait as long as clock is driven to low by the slave */
        uint32_t stretch = I2C_CLOCK_STRETCH;
        while (stretch && !_i2c_scl_read(bus)) {
            stretch--;
        }
        if (stretch == 0) {
            //DEBUG("%s: clock stretching timeout dev=%u\n", __func__, bus->dev);
            res = -ETIMEDOUT;
        }

        /* wait t_SU;STA - set-up time for a repeated START condition */
        /* min. in us: 4.7 (SM), 0.6 (FM), 0.26 (FPM), 0.16 (HSM); no max. */
        _i2c_delay(bus);
    }

    /* if SDA is low, arbitration is lost and someone else is driving the bus */
    if (!_i2c_sda_read(bus)) {
        return _i2c_arbitration_lost(bus, __func__);
    }

    /* begin the START condition: SDA = active LOW */
    _i2c_sda_low(bus);

    /* wait t_HD;STA - hold time (repeated) START condition, */
    /* max none */
    /* min 4.0 us (SM), 0.6 us (FM), 0.26 us (FPM), 0.16 us (HSM) */
    _i2c_delay(bus);

    /* complete the START condition: SCL = active LOW */
    _i2c_scl_low(bus);

    /* needed for repeated start condition */
    bus->started = true;

    return res;
}

static IRAM_ATTR int _i2c_stop_cond(_i2c_bus_t* bus)
{
    /*
     * send stop condition
     * on entry: SCL is active low and SDA can be changed
     * on exit : SCL and SDA are set to be floating and pulled-up to high
     */

    int res = 0;

    /* begin the STOP condition: SDA = active LOW */
    _i2c_sda_low(bus);

    /* wait t_LOW - LOW period of SCL clock */
    /* min. in us: 4.7 (SM), 1.3 (FM), 0.5 (FPM), 0.16 (HSM); no max. */
    _i2c_delay(bus);

    /* SCL = passive HIGH (floating and pulled up) while SDA = active LOW */
    _i2c_scl_high(bus);

    /* clock stretching, wait as long as clock is driven to low by the slave */
    uint32_t stretch = I2C_CLOCK_STRETCH;
    while (stretch && !_i2c_scl_read(bus)) {
        stretch--;
    }
    if (stretch == 0) {
        //DEBUG("%s: clock stretching timeout dev=%u\n", __func__, bus->dev);
        res = -ETIMEDOUT;
    }

    /* wait t_SU;STO - hold time STOP condition, */
    /* min. in us: 4.0 (SM), 0.6 (FM), 0.26 (FPM), 0.16 (HSM); no max. */
    _i2c_delay(bus);

    /* complete the STOP condition: SDA = passive HIGH (floating and pulled up) */
    _i2c_sda_high(bus);

    /* reset repeated start indicator */
    bus->started = false;

    /* wait t_BUF - bus free time between a STOP and a START condition */
    /* min. in us: 4.7 (SM), 1.3 (FM), 0.5 (FPM), 0.16 (HSM); no max. */
    _i2c_delay(bus);
    /* one additional delay */
    _i2c_delay(bus);

    /* if SDA is low, arbitration is lost and someone else is driving the bus */
    if (_i2c_sda_read(bus) == 0) {
        return _i2c_arbitration_lost(bus, __func__);
    }

    return res;
}

static IRAM_ATTR int _i2c_write_bit(_i2c_bus_t* bus, bool bit)
{
    /*
     * send one bit
     * on entry: SCL is active low, SDA can be changed
     * on exit : SCL is active low, SDA can be changed
     */

    int res = 0;

    /* SDA = bit */
    if (bit) {
        _i2c_sda_high(bus);
    }
    else {
        _i2c_sda_low(bus);
    }

    /* wait t_VD;DAT - data valid time (time until data are valid) */
    /* max. in us: 3.45 (SM), 0.9 (FM), 0.45 (FPM); no min */
    _i2c_delay(bus);

    /* SCL = passive HIGH (floating and pulled-up), SDA value is available */
    _i2c_scl_high(bus);

    /* wait t_HIGH - time for the slave to read SDA */
    /* min. in us: 4 (SM), 0.6 (FM), 0.26 (FPM), 0.09 (HSM); no max. */
    _i2c_delay(bus);

    /* clock stretching, wait as long as clock is driven low by the slave */
    uint32_t stretch = I2C_CLOCK_STRETCH;
    while (stretch && !_i2c_scl_read(bus)) {
        stretch--;
    }
    if (stretch == 0) {
        //DEBUG("%s: clock stretching timeout dev=%u\n", __func__, bus->dev);
        res = -ETIMEDOUT;
    }

    /* if SCL is high, now data is valid */
    /* if SDA is high, check that nobody else is driving SDA low */
    if (bit && !_i2c_sda_read(bus)) {
        return _i2c_arbitration_lost(bus, __func__);
    }

    /* SCL = active LOW to allow next SDA change */
    _i2c_scl_low(bus);

    return res;
}

static IRAM_ATTR int _i2c_read_bit(_i2c_bus_t* bus, bool* bit)
{
    /* read one bit
     * on entry: SCL is active low, SDA can be changed
     * on exit : SCL is active low, SDA can be changed
     */

    int res = 0;

    /* SDA = passive HIGH (floating and pulled-up) to let the slave drive data */
    _i2c_sda_high(bus);

    /* wait t_VD;DAT - data valid time (time until data are valid) */
    /* max. in us: 3.45 (SM), 0.9 (FM), 0.45 (FPM); no min */
    _i2c_delay(bus);

    /* SCL = passive HIGH (floating and pulled-up), SDA value is available */
    _i2c_scl_high(bus);

    /* clock stretching, wait as long as clock is driven to low by the slave */
    uint32_t stretch = I2C_CLOCK_STRETCH;
    while (stretch && !_i2c_scl_read(bus)) {
        stretch--;
    }
    if (stretch == 0) {
        //DEBUG("%s: clock stretching timeout dev=%u\n", __func__, bus->dev);
        res = -ETIMEDOUT;
    }

    /* wait t_HIGH - time for the slave to read SDA */
    /* min. in us: 4 (SM), 0.6 (FM), 0.26 (FPM), 0.09 (HSM); no max. */
    _i2c_delay(bus);

    /* SCL is high, read out bit */
    *bit = _i2c_sda_read(bus);

    /* SCL = active LOW to allow next SDA change */
    _i2c_scl_low(bus);

    return res;
}

static IRAM_ATTR int _i2c_write_byte(_i2c_bus_t* bus, uint8_t byte)
{
    /* send one byte and returns 0 in case of ACK from slave */

    /* send the byte from MSB to LSB */
    for (unsigned i = 0; i < 8; i++) {
        int res = _i2c_write_bit(bus, (byte & 0x80) != 0);
        if (res != 0) {
            return res;
        }
        byte = byte << 1;
    }

    /* read acknowledge bit (low) from slave */
    bool bit;
    int res = _i2c_read_bit(bus, &bit);
    if (res != 0) {
        return res;
    }

    return !bit ? 0 : -EIO;
}


static IRAM_ATTR int _i2c_read_byte(_i2c_bus_t* bus, uint8_t *byte, bool ack)
{
    bool bit;

    /* read the byte */
    for (unsigned i = 0; i < 8; i++) {
        int res = _i2c_read_bit(bus, &bit);
        if (res != 0) {
            return res;
        }
        *byte = (*byte << 1) | (bit ? 1 : 0);
    }

    /* write acknowledgement flag */
    _i2c_write_bit(bus, !ack);

    return 0;
}

int i2c_read_regs(_i2c_bus_t* bus, uint16_t addr, uint16_t reg,
                  void *data, size_t len, uint8_t flags)
{
    uint16_t reg_end = reg;

    if (flags & (I2C_NOSTOP | I2C_NOSTART)) {
        return -EOPNOTSUPP;
    }

    /* Handle endianness of register if 16 bit */
    if (flags & I2C_REG16) {
        reg_end = htons(reg); /* Make sure register is in big-endian on I2C bus */
    }

    /* First set ADDR and register with no stop */
    int ret = i2c_write_bytes(bus, addr, &reg_end, (flags & I2C_REG16) ? 2 : 1,
                              flags | I2C_NOSTOP);
    if (ret < 0) {
        return ret;
    }
    /* Then get the data from device */
    return i2c_read_bytes(bus, addr, data, len, flags);
}

int i2c_read_reg(_i2c_bus_t* bus, uint16_t addr, uint16_t reg,
                 void *data, uint8_t flags)
{
    return i2c_read_regs(bus, addr, reg, data, 1, flags);
}


int i2c_read_byte(_i2c_bus_t* bus, uint16_t addr, void *data, uint8_t flags)
{
    return i2c_read_bytes(bus, addr, data, 1, flags);
}

int i2c_write_byte(_i2c_bus_t* bus, uint16_t addr, uint8_t data, uint8_t flags)
{
    return i2c_write_bytes(bus, addr, &data, 1, flags);
}

int i2c_write_regs(_i2c_bus_t* bus, uint16_t addr, uint16_t reg,
                   const void *data, size_t len, uint8_t flags)
{
    uint16_t reg_end = reg;

    if (flags & (I2C_NOSTOP | I2C_NOSTART)) {
        return -EOPNOTSUPP;
    }

    /* Handle endianness of register if 16 bit */
    if (flags & I2C_REG16) {
        reg_end = htons(reg); /* Make sure register is in big-endian on I2C bus */
    }

    /* First set ADDR and register with no stop */
    int ret = i2c_write_bytes(bus, addr, &reg_end, (flags & I2C_REG16) ? 2 : 1,
                              flags | I2C_NOSTOP);
    if (ret < 0) {
        return ret;
    }
    /* Then write data to the device */
    return i2c_write_bytes(bus, addr, data, len, flags | I2C_NOSTART);
}

int i2c_write_reg(_i2c_bus_t* bus, uint16_t addr, uint16_t reg,
                  uint8_t data, uint8_t flags)
{
    return i2c_write_regs(bus, addr, reg, &data, 1, flags);
}
