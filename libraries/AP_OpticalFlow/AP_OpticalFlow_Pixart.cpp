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
  driver for Pixart PMW3900DH optical flow sensor


  NOTE: This sensor does not use traditional SPI register access. The
  timing for register reads and writes is critical
 */

#include "AP_OpticalFlow_Pixart.h"

#if AP_OPTICALFLOW_PIXART_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/crc.h>
#include <AP_AHRS/AP_AHRS.h>
#include <utility>
#include "AP_OpticalFlow.h"
#include "AP_OpticalFlow_Pixart_SROM.h"
#include <stdio.h>

#define debug(fmt, args ...)  do {printf(fmt, ## args); } while(0)

extern const AP_HAL::HAL& hal;

#define PIXART_REG_PRODUCT_ID  0x00
#define PIXART_REG_REVISION_ID 0x01
#define PIXART_REG_MOTION      0x02
#define PIXART_REG_DELTA_X_L   0x03
#define PIXART_REG_DELTA_X_H   0x04
#define PIXART_REG_DELTA_Y_L   0x05
#define PIXART_REG_DELTA_Y_H   0x06
#define PIXART_REG_SQUAL       0x07
#define PIXART_REG_RAWDATA_SUM 0x08
#define PIXART_REG_RAWDATA_MAX 0x09
#define PIXART_REG_RAWDATA_MIN 0x0A
#define PIXART_REG_SHUTTER_LOW 0x0B
#define PIXART_REG_SHUTTER_HI  0x0C
#define PIXART_REG_CONFIG1     0x0F
#define PIXART_REG_CONFIG2     0x10
#define PIXART_REG_FRAME_CAP   0x12
#define PIXART_REG_SROM_EN     0x13
#define PIXART_REG_RUN_DS      0x14
#define PIXART_REG_REST1_RATE  0x15
#define PIXART_REG_REST1_DS    0x16
#define PIXART_REG_REST2_RATE  0x17
#define PIXART_REG_REST2_DS    0x18
#define PIXART_REG_REST3_RATE  0x19
#define PIXART_REG_OBS         0x24
#define PIXART_REG_DOUT_L      0x25
#define PIXART_REG_DOUT_H      0x26
#define PIXART_REG_RAW_GRAB    0x29
#define PIXART_REG_SROM_ID     0x2A
#define PIXART_REG_POWER_RST   0x3A
#define PIXART_REG_SHUTDOWN    0x3B
#define PIXART_REG_INV_PROD_ID 0x3F
#define PIXART_REG_INV_PROD_ID2 0x5F // for 3901
#define PIXART_REG_MOT_BURST   0x50
#define PIXART_REG_MOT_BURST2  0x16
#define PIXART_REG_SROM_BURST  0x62
#define PIXART_REG_RAW_BURST   0x64

// writing to registers needs this flag
#define PIXART_WRITE_FLAG      0x80

// timings in microseconds
#define PIXART_Tsrad           300

// correct result for SROM CRC
#define PIXART_SROM_CRC_RESULT 0xBEEF

// constructor
AP_OpticalFlow_Pixart::AP_OpticalFlow_Pixart(const char *devname, AP_OpticalFlow &_frontend) :
    OpticalFlow_backend(_frontend)
{
    _dev = std::move(hal.spi->get_device(devname));
}

// detect the device
AP_OpticalFlow_Pixart *AP_OpticalFlow_Pixart::detect(const char *devname, AP_OpticalFlow &_frontend)
{
    AP_OpticalFlow_Pixart *sensor = new AP_OpticalFlow_Pixart(devname, _frontend);
    if (!sensor) {
        return nullptr;
    }
    if (!sensor->setup_sensor()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

// setup the device
bool AP_OpticalFlow_Pixart::setup_sensor(void)
{
    if (!_dev) {
        return false;
    }
    WITH_SEMAPHORE(_dev->get_semaphore());

    // power-up sequence
    reg_write(PIXART_REG_POWER_RST, 0x5A);
    hal.scheduler->delay(50);

    // check product ID
    uint8_t id1 = reg_read(PIXART_REG_PRODUCT_ID);
    uint8_t id2;
    if (id1 == 0x3f) {
        id2 = reg_read(PIXART_REG_INV_PROD_ID);
    } else {
        id2 = reg_read(PIXART_REG_INV_PROD_ID2);
    }
    debug("id1=0x%02x id2=0x%02x ~id1=0x%02x\n", id1, id2, uint8_t(~id1));
    if (id1 == 0x3F && id2 == uint8_t(~id1)) {
        model = PIXART_3900;
    } else if (id1 == 0x49 && id2 == uint8_t(~id1)) {
        model = PIXART_3901;
    } else {
        debug("Not a recognised device\n");
        return false;
    }

    if (model == PIXART_3900) {
        srom_download();

        const uint8_t id = reg_read(PIXART_REG_SROM_ID);
        if (id != srom_id) {
            debug("Pixart: bad SROM ID: 0x%02x\n", id);
            return false;
        }

        reg_write(PIXART_REG_SROM_EN, 0x15);
        hal.scheduler->delay(10);

        const uint16_t crc = reg_read16u(PIXART_REG_DOUT_L);
        if (crc != 0xBEEF) {
            debug("Pixart: bad SROM CRC: 0x%04x\n", crc);
            return false;
        }
    }

    if (model == PIXART_3900) {
        load_configuration(init_data_3900, ARRAY_SIZE(init_data_3900));
    } else {
        load_configuration(init_data_3901_1, ARRAY_SIZE(init_data_3901_1));
        hal.scheduler->delay(100);
        load_configuration(init_data_3901_2, ARRAY_SIZE(init_data_3901_2));
    }

    hal.scheduler->delay(50);

    debug("Pixart %s ready\n", model==PIXART_3900?"3900":"3901");

    integral.last_frame_us = AP_HAL::micros();

    _dev->register_periodic_callback(2000, FUNCTOR_BIND_MEMBER(&AP_OpticalFlow_Pixart::timer, void));
    return true;
}

// write an 8 bit register
void AP_OpticalFlow_Pixart::reg_write(uint8_t reg, uint8_t value)
{
    _dev->set_chip_select(true);
    reg |= PIXART_WRITE_FLAG;
    _dev->transfer(&reg, 1, nullptr, 0);
    hal.scheduler->delay_microseconds(PIXART_Tsrad);
    _dev->transfer(&value, 1, nullptr, 0);
    _dev->set_chip_select(false);
    hal.scheduler->delay_microseconds(120);
}

// read from an 8 bit register
uint8_t AP_OpticalFlow_Pixart::reg_read(uint8_t reg)
{
    uint8_t v = 0;
    _dev->set_chip_select(true);
    _dev->transfer(&reg, 1, nullptr, 0);
    hal.scheduler->delay_microseconds(35);
    _dev->transfer(nullptr, 0, &v, 1);
    _dev->set_chip_select(false);
    hal.scheduler->delay_microseconds(200);
    return v;
}

// read from a 16 bit unsigned register
uint16_t AP_OpticalFlow_Pixart::reg_read16u(uint8_t reg)
{
    uint16_t low = reg_read(reg);
    uint16_t high = reg_read(reg+1);
    return low | (high<<8);
}

// read from a 16 bit signed register
int16_t AP_OpticalFlow_Pixart::reg_read16s(uint8_t reg)
{
    return (int16_t)reg_read16u(reg);
}

void AP_OpticalFlow_Pixart::srom_download(void)
{
    reg_write(0x39, 0x02);
    hal.scheduler->delay(1);
    reg_write(PIXART_REG_SROM_EN, 0x1D);
    hal.scheduler->delay(10);
    reg_write(PIXART_REG_SROM_EN, 0x18);

    if (!_dev->set_chip_select(true)) {
        debug("Failed to force CS\n");
    }
    hal.scheduler->delay_microseconds(1);
    uint8_t reg = PIXART_REG_SROM_BURST | PIXART_WRITE_FLAG;
    _dev->transfer(&reg, 1, nullptr, 0);

    for (uint16_t i = 0; i < ARRAY_SIZE(srom_data); i++) {
        hal.scheduler->delay_microseconds(15);
        _dev->transfer(&srom_data[i], 1, nullptr, 0);
    }

    hal.scheduler->delay_microseconds(125);
    if (!_dev->set_chip_select(false)) {
        debug("Failed to force CS off\n");
    }
    hal.scheduler->delay_microseconds(160);
}

void AP_OpticalFlow_Pixart::load_configuration(const RegData *init_data, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++) {
        // writing a config register can fail - retry up to 5 times
        for (uint8_t tries=0; tries<5; tries++) {
            reg_write(init_data[i].reg, init_data[i].value);
            uint8_t v = reg_read(init_data[i].reg);
            if (v == init_data[i].value) {
                break;
            }
            //debug("reg[%u:%02x] 0x%02x 0x%02x\n", (unsigned)i, (unsigned)init_data[i].reg, (unsigned)init_data[i].value, (unsigned)v);
        }
    }
}

void AP_OpticalFlow_Pixart::motion_burst(void)
{
    uint8_t *b = (uint8_t *)&burst;

    burst.delta_x = 0;
    burst.delta_y = 0;

    _dev->set_chip_select(true);
    uint8_t reg = model==PIXART_3900?PIXART_REG_MOT_BURST:PIXART_REG_MOT_BURST2;

    _dev->transfer(&reg, 1, nullptr, 0);
    hal.scheduler->delay_microseconds(150);

    for (uint8_t i=0; i<sizeof(burst); i++) {
        _dev->transfer(nullptr, 0, &b[i], 1);
        if (i == 0 && (burst.motion & 0x80) == 0) {
            // no motion, save some bus bandwidth
            _dev->set_chip_select(false);
            return;
        }
    }
    _dev->set_chip_select(false);
}

void AP_OpticalFlow_Pixart::timer(void)
{
    if (AP_HAL::micros() - last_burst_us < 500) {
        return;
    }
    motion_burst();
    last_burst_us = AP_HAL::micros();

    uint32_t dt_us = last_burst_us - integral.last_frame_us;
    float dt = dt_us * 1.0e-6;
    const Vector3f &gyro = AP::ahrs().get_gyro();

    {
        WITH_SEMAPHORE(_sem);

        integral.sum.x += burst.delta_x;
        integral.sum.y += burst.delta_y;
        integral.sum_us += dt_us;
        integral.last_frame_us = last_burst_us;
        integral.gyro += Vector2f(gyro.x, gyro.y) * dt;
    }

#if 0
    static uint32_t last_print_ms;
    static int fd = -1;
    if (fd == -1) {
        fd = open("/dev/ttyACM0", O_WRONLY);
    }
    // used for debugging
    static int32_t sum_x;
    static int32_t sum_y;
    sum_x += burst.delta_x;
    sum_y += burst.delta_y;

    uint32_t now = AP_HAL::millis();
    if (now - last_print_ms >= 100 && (sum_x != 0 || sum_y != 0)) {
        last_print_ms = now;
        dprintf(fd, "Motion: %d %d obs:0x%02x squal:%u rds:%u maxr:%u minr:%u sup:%u slow:%u\n",
               (int)sum_x, (int)sum_y, (unsigned)burst.squal, (unsigned)burst.rawdata_sum, (unsigned)burst.max_raw,
               (unsigned)burst.max_raw, (unsigned)burst.min_raw, (unsigned)burst.shutter_upper, (unsigned)burst.shutter_lower);
        sum_x = sum_y = 0;
    }
#endif
}

// update - read latest values from sensor and fill in x,y and totals.
void AP_OpticalFlow_Pixart::update(void)
{
    uint32_t now = AP_HAL::millis();
    if (now - last_update_ms < 100) {
        return;
    }
    last_update_ms = now;

    struct AP_OpticalFlow::OpticalFlow_state state;
    state.surface_quality = burst.squal;

    if (integral.sum_us > 0) {
        WITH_SEMAPHORE(_sem);

        const Vector2f flowScaler = _flowScaler();
        float flowScaleFactorX = 1.0f + 0.001f * flowScaler.x;
        float flowScaleFactorY = 1.0f + 0.001f * flowScaler.y;
        float dt = integral.sum_us * 1.0e-6;

        state.flowRate = Vector2f(integral.sum.x * flowScaleFactorX,
                                  integral.sum.y * flowScaleFactorY);
        state.flowRate *= flow_pixel_scaling / dt;

        // we only apply yaw to flowRate as body rate comes from AHRS
        _applyYaw(state.flowRate);

        state.bodyRate = integral.gyro / dt;

        integral.sum.zero();
        integral.sum_us = 0;
        integral.gyro.zero();
    } else {
        state.flowRate.zero();
        state.bodyRate.zero();
    }

    // copy results to front end
    _update_frontend(state);
}

#endif  // AP_OPTICALFLOW_PIXART_ENABLED
