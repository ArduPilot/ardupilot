/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Compass_MMC5xx3.h"

#if AP_COMPASS_MMC5XX3_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <stdio.h>

extern const AP_HAL::HAL &hal;

// MMC5983 register definitions
#define REG_PRODUCT_ID      0x2F
#define REG_XOUT_L          0x00
#define REG_STATUS          0x08
#define REG_CONTROL0        0x09
#define REG_CONTROL1        0x0A
#define REG_CONTROL2        0x0B

// MMC5603 register definitions
#define MMC5603_REG_PRODUCT_ID   0x39
#define MMC5603_REG_XOUT_L       0x00
#define MMC5603_REG_STATUS       0x18
#define MMC5603_REG_CONTROL0     0x1B
#define MMC5603_REG_CONTROL1     0x1C
#define MMC5603_REG_CONTROL2     0x1D

// CONTROL0 bits (same bit definitions for both chips)
#define REG_CONTROL0_SET    0x08  // Pulse SET coil to magnetise sense elements
#define REG_CONTROL0_RESET  0x10  // Pulse RESET coil to demagnetise sense elements
#define REG_CONTROL0_TMM    0x01  // Take Measurement for Magnetic field
#define REG_CONTROL0_TMT    0x02  // Take Measurement for Temperature

// CONTROL1 bits
#define REG_CONTROL1_SW_RST 0x80  // Software reset
#define REG_CONTROL1_BW0    0x01
#define REG_CONTROL1_BW1    0x02

#define MMC5983_ID 0x30
#define MMC5603_ID 0x10

AP_Compass_Backend *AP_Compass_MMC5XX3::probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                              bool force_external,
                                              enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_Compass_MMC5XX3 *sensor = NEW_NOTHROW AP_Compass_MMC5XX3(std::move(dev), force_external, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_MMC5XX3::AP_Compass_MMC5XX3(AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                       bool _force_external,
                                       enum Rotation _rotation)
    : dev(std::move(_dev))
    , force_external(_force_external)
    , have_initial_offset(false)
    , rotation(_rotation)
{
}

bool AP_Compass_MMC5XX3::probe_mmc5983()
{
    // Reading REG_PRODUCT_ID fails sometimes on SPI, so we retry up to 10 times
    uint8_t whoami = 0;
    uint8_t tries = 10;
    while (whoami == 0 && tries > 0) {
        tries--;
        dev->read_registers(REG_PRODUCT_ID, &whoami, 1);
        hal.scheduler->delay(5);
    }

    if (whoami != MMC5983_ID) {
        // printf("MMC5983 got unexpected product id: %d, expected: %d\n", whoami, MMC5983_ID);
        // not a MMC5983
        return false;
    }

    chip_variant = ChipVariant::MMC5983;

    // reset sensor
    dev->write_register(REG_CONTROL1, REG_CONTROL1_SW_RST);

    // 10ms minimum startup time
    hal.scheduler->delay(15);

    // setup for 100Hz output
    if (!dev->write_register(REG_CONTROL1, 0)) {
        return false;
    }

    /* register the compass instance in the frontend */
    dev->set_device_type(DEVTYPE_MMC5983);
    if (!register_compass(dev->get_bus_id())) {
        return false;
    }

    printf("Found a MMC5983 on 0x%x as compass %u\n", unsigned(dev->get_bus_id()), instance);

    return true;
}

bool AP_Compass_MMC5XX3::probe_mmc5603()
{
    // Reading REG_PRODUCT_ID fails sometimes, so we retry up to 10 times
    uint8_t whoami = 0;
    uint8_t tries = 10;
    while (whoami == 0 && tries > 0) {
        tries--;
        dev->read_registers(MMC5603_REG_PRODUCT_ID, &whoami, 1);
        hal.scheduler->delay(5);
    }

    if (whoami != MMC5603_ID) {
        // printf("MMC5603 got unexpected product id: %d, expected: %d\n", whoami, MMC5603_ID);
        // not a MMC5603
        return false;
    }

    chip_variant = ChipVariant::MMC5603;

    // software reset, then wait for it to complete
    dev->write_register(MMC5603_REG_CONTROL1, REG_CONTROL1_SW_RST);
    hal.scheduler->delay(20);

    // clear BW bits (default bandwidth, single measurement mode)
    dev->write_register(MMC5603_REG_CONTROL1, 0x00);

    // degauss: SET pulse followed by RESET pulse
    dev->write_register(MMC5603_REG_CONTROL0, REG_CONTROL0_SET);
    hal.scheduler->delay(1);
    dev->write_register(MMC5603_REG_CONTROL0, REG_CONTROL0_RESET);
    hal.scheduler->delay(1);

    // ensure continuous mode is off; we drive measurements ourselves
    dev->write_register(MMC5603_REG_CONTROL2, 0x00);

    /* register the compass instance in the frontend */
    dev->set_device_type(DEVTYPE_MMC5603);
    if (!register_compass(dev->get_bus_id())) {
        return false;
    }

    printf("Found a MMC5603 on 0x%x as compass %u\n", unsigned(dev->get_bus_id()), instance);

    return true;
}

bool AP_Compass_MMC5XX3::init()
{
    // take i2c bus semaphore
    WITH_SEMAPHORE(dev->get_semaphore());

    dev->set_retries(10);

    // setup to allow reads on SPI
    if (dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
        dev->set_read_flag(0x80);
    }

    // probe for MMC5983 first, then fall back to MMC5603
    if (!probe_mmc5983() && !probe_mmc5603()) {
        return false;
    }

    set_rotation(rotation);

    if (force_external) {
        set_external(true);
    }

    dev->set_retries(1);

    // call timer() at 100Hz
    dev->register_periodic_callback(10000U,
                                    FUNCTOR_BIND_MEMBER(&AP_Compass_MMC5XX3::timer, void));

    return true;
}

void AP_Compass_MMC5XX3::timer()
{
    // recalculate the offset with set/reset operation every measure_count_limit measurements
    // sensor is read at about 100Hz, so about every 10 seconds
    const uint16_t measure_count_limit = 1000U;

    // MMC5983: 16-bit output, sensitivity 4096 counts/Gauss (0.244 mG/LSB)
    const uint16_t mmc5983_zero_offset = 32768U;
    const uint16_t mmc5983_sensitivity = 4096U;
    constexpr float mmc5983_counts_to_milliGauss = 1.0e3f / mmc5983_sensitivity;

    // MMC5603: 20-bit output, 0.0625 mG/LSB
    const uint32_t mmc5603_zero_offset = 524288UL;
    constexpr float mmc5603_counts_to_milliGauss = 0.0625f;

    // select register addresses and data format based on detected chip
    const uint8_t reg_ctrl0       = (chip_variant == ChipVariant::MMC5603) ? MMC5603_REG_CONTROL0 : REG_CONTROL0;
    const uint8_t reg_status      = (chip_variant == ChipVariant::MMC5603) ? MMC5603_REG_STATUS    : REG_STATUS;
    const uint8_t reg_xout_l      = (chip_variant == ChipVariant::MMC5603) ? MMC5603_REG_XOUT_L   : REG_XOUT_L;
    const uint8_t status_done_bit = (chip_variant == ChipVariant::MMC5603) ? 0x40 : 0x01;
    const uint8_t read_len        = (chip_variant == ChipVariant::MMC5603) ? 9 : 6;

    /*
      we use the SET/RESET method to remove bridge offset every
      measure_count_limit measurements. This involves a fairly complex
      state machine, but means we are much less sensitive to
      temperature changes
     */
    switch (state) {

    // perform a set operation
    case MMCState::STATE_SET: {
        if (!dev->write_register(reg_ctrl0, REG_CONTROL0_SET)) {
            break;
        }
        // minimum time to wait after set/reset before take measurement request is 1ms
        state = MMCState::STATE_SET_MEASURE;
        break;
    }

    // request a measurement for field and offset calculation after set operation
    case MMCState::STATE_SET_MEASURE: {
        if (!dev->write_register(reg_ctrl0, REG_CONTROL0_TMM)) {
            break;
        }
        state = MMCState::STATE_SET_WAIT;
        break;
    }

    // wait for measurement to be ready after set operation, then read the
    // measurement data and request a reset operation
    case MMCState::STATE_SET_WAIT: {
        uint8_t status;
        if (!dev->read_registers(reg_status, &status, 1)) {
            state = MMCState::STATE_SET;
            break;
        }

        // check if measurement is ready
        if (!(status & status_done_bit)) {
            break;
        }

        // read measurement
        if (!dev->read_registers(reg_xout_l, (uint8_t *)&data0[0], read_len)) {
            state = MMCState::STATE_SET;
            break;
        }

        // request reset operation
        if (!dev->write_register(reg_ctrl0, REG_CONTROL0_RESET)) {
            break;
        }
        // minimum time to wait after set/reset before take measurement request is 1ms
        state = MMCState::STATE_RESET_MEASURE;
        break;
    }

    // request a measurement for field and offset calculation after reset operation
    case MMCState::STATE_RESET_MEASURE: {
        // take measurement request
        if (!dev->write_register(reg_ctrl0, REG_CONTROL0_TMM)) {
            state = MMCState::STATE_SET;
            break;
        }

        state = MMCState::STATE_RESET_WAIT;
        break;
    }

    // wait for measurement to be ready after reset operation,
    // then read the measurement data, calculate the field and offset,
    // and begin requesting field measurements
    case MMCState::STATE_RESET_WAIT: {
        uint8_t status;
        if (!dev->read_registers(reg_status, &status, 1)) {
            state = MMCState::STATE_SET;
            break;
        }
        // check if measurement is ready
        if (!(status & status_done_bit)) {
            break;
        }

        uint8_t data1[9];
        if (!dev->read_registers(reg_xout_l, (uint8_t *)&data1[0], read_len)) {
            state = MMCState::STATE_SET;
            break;
        }

        /*
          calculate field and offset
         */
        Vector3f f1, f2;
        if (chip_variant == ChipVariant::MMC5603) {
            // 20-bit packed: bytes 0-5 are the upper 16 bits for each axis pair,
            // bytes 6-8 hold the lower 4 bits for X, Y, Z respectively
            int32_t x0 = ((uint32_t)data0[0] << 12) | ((uint32_t)data0[1] << 4) | ((uint32_t)data0[6] >> 4);
            int32_t y0 = ((uint32_t)data0[2] << 12) | ((uint32_t)data0[3] << 4) | ((uint32_t)data0[7] >> 4);
            int32_t z0 = ((uint32_t)data0[4] << 12) | ((uint32_t)data0[5] << 4) | ((uint32_t)data0[8] >> 4);
            int32_t x1 = ((uint32_t)data1[0] << 12) | ((uint32_t)data1[1] << 4) | ((uint32_t)data1[6] >> 4);
            int32_t y1 = ((uint32_t)data1[2] << 12) | ((uint32_t)data1[3] << 4) | ((uint32_t)data1[7] >> 4);
            int32_t z1 = ((uint32_t)data1[4] << 12) | ((uint32_t)data1[5] << 4) | ((uint32_t)data1[8] >> 4);
            f1 = Vector3f{float(x0 - (int32_t)mmc5603_zero_offset),
                          float(y0 - (int32_t)mmc5603_zero_offset),
                          float(z0 - (int32_t)mmc5603_zero_offset)};
            f2 = Vector3f{float(x1 - (int32_t)mmc5603_zero_offset),
                          float(y1 - (int32_t)mmc5603_zero_offset),
                          float(z1 - (int32_t)mmc5603_zero_offset)};
        } else {
            f1 = Vector3f{float((data0[0] << 8) + data0[1]) - mmc5983_zero_offset,
                          float((data0[2] << 8) + data0[3]) - mmc5983_zero_offset,
                          float((data0[4] << 8) + data0[5]) - mmc5983_zero_offset};
            f2 = Vector3f{float((data1[0] << 8) + data1[1]) - mmc5983_zero_offset,
                          float((data1[2] << 8) + data1[3]) - mmc5983_zero_offset,
                          float((data1[4] << 8) + data1[5]) - mmc5983_zero_offset};
        }

        const float scale = (chip_variant == ChipVariant::MMC5603) ? mmc5603_counts_to_milliGauss : mmc5983_counts_to_milliGauss;
        Vector3f field {(f2 - f1) * scale * 0.5f};
        Vector3f new_offset {(f1 + f2) * scale * 0.5f};

        if (!have_initial_offset) {
            offset = new_offset;
            have_initial_offset = true;
        } else {
            // low pass changes to the offset
            offset = offset * 0.99f + new_offset * 0.01f;
        }

        accumulate_sample(field);

        if (!dev->write_register(reg_ctrl0, REG_CONTROL0_TMM)) {
            printf("failed to initiate measurement\n");
            state = MMCState::STATE_SET;
        } else {
            state = MMCState::STATE_MEASURE;
        }

        break;
    }

    // take repeated field measurements, set/reset is performed again after
    // measure_count_limit measurements
    case MMCState::STATE_MEASURE: {
        uint8_t status;
        if (!dev->read_registers(reg_status, &status, 1)) {
            state = MMCState::STATE_SET;
            break;
        }

        // check if measurement is ready
        if (!(status & status_done_bit)) {
            break;
        }

        uint8_t data1[9];
        if (!dev->read_registers(reg_xout_l, (uint8_t *)&data1[0], read_len)) {
            printf("cant read data\n");
            state = MMCState::STATE_SET;
            break;
        }

        Vector3f field;
        if (chip_variant == ChipVariant::MMC5603) {
            int32_t x1 = ((uint32_t)data1[0] << 12) | ((uint32_t)data1[1] << 4) | ((uint32_t)data1[6] >> 4);
            int32_t y1 = ((uint32_t)data1[2] << 12) | ((uint32_t)data1[3] << 4) | ((uint32_t)data1[7] >> 4);
            int32_t z1 = ((uint32_t)data1[4] << 12) | ((uint32_t)data1[5] << 4) | ((uint32_t)data1[8] >> 4);
            field = Vector3f{float(x1 - (int32_t)mmc5603_zero_offset),
                             float(y1 - (int32_t)mmc5603_zero_offset),
                             float(z1 - (int32_t)mmc5603_zero_offset)};
            field *= mmc5603_counts_to_milliGauss;
        } else {
            field = Vector3f{float((data1[0] << 8) + data1[1]) - mmc5983_zero_offset,
                             float((data1[2] << 8) + data1[3]) - mmc5983_zero_offset,
                             float((data1[4] << 8) + data1[5]) - mmc5983_zero_offset};
            field *= mmc5983_counts_to_milliGauss;
        }
        field -= offset;
        accumulate_sample(field);

        // we stay in STATE_MEASURE for measure_count_limit cycles
        if (measure_count++ >= measure_count_limit) {
            measure_count = 0;
            state = MMCState::STATE_SET;
        } else {
            if (!dev->write_register(reg_ctrl0, REG_CONTROL0_TMM)) { // Take Measurement
                state = MMCState::STATE_SET;
            }
        }
        break;
    }
    }
}

void AP_Compass_MMC5XX3::read()
{
    drain_accumulated_samples();
}

#endif  // AP_COMPASS_MMC5XX3_ENABLED
