/*
 * Copyright (C) 2019  Lucas De Marchi <lucas.de.marchi@gmail.com>
 *
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
#include "AP_RangeFinder_Benewake_TFMiniPlus.h"

#include <utility>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#define DRIVER "TFMiniPlus"

/*
 * Command format:
 *
 * uint8_t header;
 * uint8_t len;
 * uint8_t id;
 * uint8_t data[];
 * uint8_t checksum;
 */

AP_RangeFinder_Benewake_TFMiniPlus::AP_RangeFinder_Benewake_TFMiniPlus(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params,
        AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_RangeFinder_Backend(_state, _params)
    , _dev(std::move(dev))
{
}

AP_RangeFinder_Backend *AP_RangeFinder_Benewake_TFMiniPlus::detect(
        RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params,
        AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    if (!dev) {
        return nullptr;
    }

    AP_RangeFinder_Benewake_TFMiniPlus *sensor
        = new AP_RangeFinder_Benewake_TFMiniPlus(_state, _params, std::move(dev));

    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

bool AP_RangeFinder_Benewake_TFMiniPlus::init()
{
    const uint8_t CMD_FW_VERSION[] =         { 0x5A, 0x04, 0x01, 0x5F };
    const uint8_t CMD_SYSTEM_RESET[] =       { 0x5A, 0x04, 0x04, 0x62 };
    const uint8_t CMD_OUTPUT_FORMAT_CM[] =   { 0x5A, 0x05, 0x05, 0x01, 0x65 };
    const uint8_t CMD_ENABLE_DATA_OUTPUT[] = { 0x5A, 0x05, 0x07, 0x01, 0x67 };
    const uint8_t CMD_FRAME_RATE_100HZ[] =   { 0x5A, 0x06, 0x03, 0x64, 0x00, 0xC7 };
    const uint8_t CMD_SAVE_SETTINGS[] =      { 0x5A, 0x04, 0x11, 0x6F };
    const uint8_t *cmds[] = {
        CMD_OUTPUT_FORMAT_CM,
        CMD_FRAME_RATE_100HZ,
        CMD_ENABLE_DATA_OUTPUT,
        CMD_SAVE_SETTINGS,
    };
    uint8_t val[12], i;
    bool ret;

    _dev->get_semaphore()->take_blocking();

    _dev->set_retries(0);

    /*
     * Check we get a response for firmware version to detect if sensor is there
     */
    ret = _dev->transfer(CMD_FW_VERSION, sizeof(CMD_FW_VERSION), nullptr, 0);
    if (!ret) {
        goto fail;
    }

    hal.scheduler->delay(100);

    ret = _dev->transfer(nullptr, 0, val, 7);
    if (!ret || val[0] != 0x5A || val[1] != 0x07 || val[2] != 0x01 ||
        !check_checksum(val, 7)) {
        goto fail;
    }

    if (val[5] * 10000 + val[4] * 100 + val[3] < 20003) {
        hal.console->printf(DRIVER ": minimum required FW version 2.0.3, but version %u.%u.%u found\n",
                            val[5], val[4], val[3]);
        goto fail;
    }

    hal.console->printf(DRIVER ": found fw version %u.%u.%u\n",
                        val[5], val[4], val[3]);

    for (i = 0; i < ARRAY_SIZE(cmds); i++) {
        ret = _dev->transfer(cmds[i], cmds[i][1], nullptr, 0);
        if (!ret) {
            hal.console->printf(DRIVER ": Unable to set configuration register %u\n",
                                cmds[i][2]);
            goto fail;
        }
        hal.scheduler->delay(100);
    }

    _dev->transfer(CMD_SYSTEM_RESET, sizeof(CMD_SYSTEM_RESET), nullptr, 0);

    _dev->get_semaphore()->give();

    hal.scheduler->delay(100);

    _dev->register_periodic_callback(10000,
                                     FUNCTOR_BIND_MEMBER(&AP_RangeFinder_Benewake_TFMiniPlus::timer, void));

    return true;

fail:
    _dev->get_semaphore()->give();
    return false;
}

void AP_RangeFinder_Benewake_TFMiniPlus::update()
{
    WITH_SEMAPHORE(_sem);

    if (accum.count > 0) {
        state.distance_cm = accum.sum / accum.count;
        state.last_reading_ms = AP_HAL::millis();
        accum.sum = 0;
        accum.count = 0;
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > 200) {
        set_status(RangeFinder::Status::NoData);
    }
}

bool AP_RangeFinder_Benewake_TFMiniPlus::process_raw_measure(le16_t distance_raw, le16_t strength_raw,
                                                             uint16_t &output_distance_cm)
{
    uint16_t strength = le16toh(strength_raw);

    output_distance_cm = le16toh(distance_raw);

    if (strength < 100 || strength == 0xFFFF) {
        return false;
    }

    output_distance_cm = constrain_int16(output_distance_cm, 10, 1200);

    return true;
}

bool AP_RangeFinder_Benewake_TFMiniPlus::check_checksum(uint8_t *arr, int pkt_len)
{
    uint8_t checksum = 0;
    int i;

    /* sum them all except the last (the checksum) */
    for (i = 0; i < pkt_len - 1; i++) {
        checksum += arr[i];
    }

    return checksum == arr[pkt_len - 1];
}

void AP_RangeFinder_Benewake_TFMiniPlus::timer()
{
    uint8_t CMD_READ_MEASUREMENT[] = { 0x5A, 0x05, 0x00, 0x07, 0x66 };
    union {
        struct PACKED {
            uint8_t header1;
            uint8_t header2;
            le16_t distance;
            le16_t strength;
            le32_t timestamp;
            uint8_t checksum;
        } val;
        uint8_t arr[11];
    } u;
    bool ret;
    uint16_t distance;

    ret = _dev->transfer(CMD_READ_MEASUREMENT, sizeof(CMD_READ_MEASUREMENT), nullptr, 0);
    if (!ret || !_dev->transfer(nullptr, 0, (uint8_t *)&u, sizeof(u))) {
        return;
    }

    if (u.val.header1 != 0x59 || u.val.header2 != 0x59 || !check_checksum(u.arr, sizeof(u)))
        return;

    if (process_raw_measure(u.val.distance, u.val.strength, distance)) {
        WITH_SEMAPHORE(_sem);
        accum.sum += distance;
        accum.count++;
    }
}
