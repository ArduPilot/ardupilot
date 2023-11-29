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

#define REG_PRODUCT_ID      0x2F
#define REG_XOUT_L          0x00
#define REG_STATUS          0x08
#define REG_CONTROL0        0x09
#define REG_CONTROL1        0x0A
#define REG_CONTROL2        0x0B

// bits in REG_CONTROL0
#define REG_CONTROL0_RESET  0x10 // Set coil for measuring offset
#define REG_CONTROL0_SET    0x08 // Reset coil for measuring offset
#define REG_CONTROL0_TMM    0x01 // Take Measurement for Magnetic field
#define REG_CONTROL0_TMT    0x02 // Take Measurement for Temperature

// bits in REG_CONTROL1
#define REG_CONTROL1_SW_RST 0x80 // Software reset
#define REG_CONTROL1_BW0    0x01
#define REG_CONTROL1_BW1    0x02

#define MMC5983_ID 0x30

AP_Compass_Backend *AP_Compass_MMC5XX3::probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                              bool force_external,
                                              enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_Compass_MMC5XX3 *sensor = new AP_Compass_MMC5XX3(std::move(dev), force_external, rotation);
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
    , rotation(_rotation)
    , have_initial_offset(false)
{
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

    // Reading REG_PRODUCT_ID fails sometimes on SPI, so we retry up to 10 times
    uint8_t whoami = 0;
    uint8_t tries = 10;
    while (whoami == 0 && tries > 0) {
        tries--;
        dev->read_registers(REG_PRODUCT_ID, &whoami, 1);
        hal.scheduler->delay(5);
    }

    if (whoami != MMC5983_ID) {
        printf("MMC5983 got unexpected product id: %d, expected: %d\n", whoami, MMC5983_ID);
        // not a MMC5983
        return false;
    }

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
    if (!register_compass(dev->get_bus_id(), compass_instance)) {
        return false;
    }

    set_dev_id(compass_instance, dev->get_bus_id());

    printf("Found a MMC5983 on 0x%x as compass %u\n", dev->get_bus_id(), compass_instance);

    set_rotation(compass_instance, rotation);

    if (force_external) {
        set_external(compass_instance, true);
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
    const uint16_t zero_offset = 32768U; // 16 bit mode
    const uint16_t sensitivity = 4096U; // counts per Gauss, 16 bit mode
    constexpr float counts_to_milliGauss = 1.0e3f / sensitivity;

    /*
      we use the SET/RESET method to remove bridge offset every
      measure_count_limit measurements. This involves a fairly complex
      state machine, but means we are much less sensitive to
      temperature changes
     */
    switch (state) {

    // perform a set operation
    case MMCState::STATE_SET: {
        if (!dev->write_register(REG_CONTROL0, REG_CONTROL0_SET)) {
            break;
        }
        // minimum time to wait after set/reset before take measurement request is 1ms
        state = MMCState::STATE_SET_MEASURE;
        break;
    }

    // request a measurement for field and offset calculation after set operation
    case MMCState::STATE_SET_MEASURE: {
        if (!dev->write_register(REG_CONTROL0, REG_CONTROL0_TMM)) {
            break;
        }
        state = MMCState::STATE_SET_WAIT;
        break;
    }

    // wait for measurement to be ready after set operation, then read the
    // measurement data and request a reset operation
    case MMCState::STATE_SET_WAIT: {
        uint8_t status;
        if (!dev->read_registers(REG_STATUS, &status, 1)) {
            state = MMCState::STATE_SET;
            break;
        }

        // check if measurement is ready
        if (!(status & 1)) {
            break;
        }

        // read measurement
        if (!dev->read_registers(REG_XOUT_L, (uint8_t *)&data0[0], 6)) {
            state = MMCState::STATE_SET;
            break;
        }

        // request set operation
        if (!dev->write_register(REG_CONTROL0, REG_CONTROL0_RESET)) {
            break;
        }
        // minimum time to wait after set/reset before take measurement request is 1ms
        state = MMCState::STATE_RESET_MEASURE;
        break;
    }

    // request a measurement for field and offset calculation after reset operation
    case MMCState::STATE_RESET_MEASURE: {
        // take measurement request
        if (!dev->write_register(REG_CONTROL0, REG_CONTROL0_TMM)) {
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
        if (!dev->read_registers(REG_STATUS, &status, 1)) {
            state = MMCState::STATE_SET;
            break;
        }
        // check if measurement is ready
        if (!(status & 1)) {
            break;
        }

        uint8_t data1[6];
        if (!dev->read_registers(REG_XOUT_L, (uint8_t *)&data1[0], 6)) {
            state = MMCState::STATE_SET;
            break;
        }

        /*
          calculate field and offset
         */
        Vector3f f1 {float((data0[0] << 8) + data0[1]) - zero_offset,
                    float((data0[2] << 8) + data0[3]) - zero_offset,
                    float((data0[4] << 8) + data0[5]) - zero_offset};
        Vector3f f2 {float((data1[0] << 8) + data1[1]) - zero_offset,
                    float((data1[2] << 8) + data1[3]) - zero_offset,
                    float((data1[4] << 8) + data1[5]) - zero_offset};

        Vector3f field {(f2 - f1) * counts_to_milliGauss * 0.5f};
        Vector3f new_offset {(f1 + f2) * counts_to_milliGauss * 0.5f};

        if (!have_initial_offset) {
            offset = new_offset;
            have_initial_offset = true;
        } else {
            // low pass changes to the offset
            offset = offset * 0.5f + new_offset * 0.5f;
        }

        accumulate_sample(field, compass_instance);

        if (!dev->write_register(REG_CONTROL0, REG_CONTROL0_TMM)) {
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
        if (!dev->read_registers(REG_STATUS, &status, 1)) {
            state = MMCState::STATE_SET;
            break;
        }

        // check if measurement is ready
        if (!(status & 1)) {
            break;
        }

        uint8_t data1[6];
        if (!dev->read_registers(REG_XOUT_L, (uint8_t *)&data1[0], 6)) {
            printf("cant read data\n");
            state = MMCState::STATE_SET;
            break;
        }

        Vector3f field {float((data1[0] << 8) + data1[1]) - zero_offset,
                       float((data1[2] << 8) + data1[3]) - zero_offset,
                       float((data1[4] << 8) + data1[5]) - zero_offset};
        field *= counts_to_milliGauss;
        field -= offset;
        accumulate_sample(field, compass_instance);

        // we stay in STATE_MEASURE for measure_count_limit cycles
        if (measure_count++ >= measure_count_limit) {
            measure_count = 0;
            state = MMCState::STATE_SET;
        } else {
            if (!dev->write_register(REG_CONTROL0, REG_CONTROL0_TMM)) { // Take Measurement
                state = MMCState::STATE_SET;
            }
        }
        break;
    }
    }
}

void AP_Compass_MMC5XX3::read()
{
    drain_accumulated_samples(compass_instance);
}

#endif  // AP_COMPASS_MMC5XX3_ENABLED

