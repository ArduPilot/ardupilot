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
#include "AP_RangeFinder_PulsedLightLRF.h"

#include <utility>
#include <stdio.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

/* LL40LS Registers addresses */
#define LL40LS_MEASURE_REG    0x00        /* Measure range register */
#define LL40LS_DISTHIGH_REG    0x0F        /* High byte of distance register, auto increment */
#define LL40LS_COUNT        0x11
#define LL40LS_HW_VERSION    0x41
#define LL40LS_INTERVAL        0x45
#define LL40LS_SW_VERSION    0x4f

// bit values
#define LL40LS_MSRREG_RESET    0x00        /* reset to power on defaults */
#define LL40LS_AUTO_INCREMENT    0x80
#define LL40LS_COUNT_CONTINUOUS    0xff
#define LL40LS_MSRREG_ACQUIRE    0x04        /* Value to initiate a measurement, varies based on sensor revision */

// i2c address
#define LL40LS_ADDR   0x62

/*
   The constructor also initializes the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_PulsedLightLRF::AP_RangeFinder_PulsedLightLRF(uint8_t bus,
                                                             RangeFinder::RangeFinder_State &_state,
                                                             RangeFinder::RangeFinder_Type _rftype)
    : AP_RangeFinder_Backend(_state)
    , _dev(hal.i2c_mgr->get_device(bus, LL40LS_ADDR))
    , rftype(_rftype)
{
}

/*
   detect if a PulsedLight rangefinder is connected. We'll detect by
   look for the version registers
*/
AP_RangeFinder_Backend *AP_RangeFinder_PulsedLightLRF::detect(uint8_t bus,
                                                              RangeFinder::RangeFinder_State &_state,
                                                              RangeFinder::RangeFinder_Type rftype)
{
    AP_RangeFinder_PulsedLightLRF *sensor
        = new AP_RangeFinder_PulsedLightLRF(bus, _state, rftype);
    if (!sensor ||
        !sensor->init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

/*
  called at 50Hz
 */
void AP_RangeFinder_PulsedLightLRF::timer(void)
{
    if (check_reg_counter++ == 10) {
        check_reg_counter = 0;
        if (!_dev->check_next_register()) {
            // re-send the acquire. this handles the case of power
            // cycling while running in continuous mode
            _dev->write_register(LL40LS_MEASURE_REG, LL40LS_MSRREG_ACQUIRE);            
        }
    }

    switch (phase) {
    case PHASE_MEASURE:
        if (_dev->write_register(LL40LS_MEASURE_REG, LL40LS_MSRREG_ACQUIRE)) {
            phase = PHASE_COLLECT;
        }
        break;

    case PHASE_COLLECT: {
        be16_t val;
        // read the high and low byte distance registers
        if (_dev->read_registers(LL40LS_DISTHIGH_REG | LL40LS_AUTO_INCREMENT, (uint8_t*)&val, sizeof(val))) {
            uint16_t _distance_cm = be16toh(val);
            // remove momentary spikes
            if (abs(_distance_cm - last_distance_cm) < 100) {
                state.distance_cm = _distance_cm;
                update_status();                
            }
            last_distance_cm = _distance_cm;
        } else {
            set_status(RangeFinder::RangeFinder_NoData);
        }
        if (!v2_hardware) {
            // for v2 hw we use continuous mode
            phase = PHASE_MEASURE;
        }
        break;
    }
    }
}


/*
  a table of settings for a lidar
 */
struct settings_table {
    uint8_t reg;
    uint8_t value;
};

/*
  register setup table for V1 Lidar
 */
static const struct settings_table settings_v1[] = {
    { LL40LS_MEASURE_REG, LL40LS_MSRREG_RESET },
};

/*
  register setup table for V2 Lidar
 */
static const struct settings_table settings_v2[] = {
    { LL40LS_INTERVAL, 0x28 }, // 0x28 == 50Hz
    { LL40LS_COUNT, LL40LS_COUNT_CONTINUOUS },
    { LL40LS_MEASURE_REG, LL40LS_MSRREG_ACQUIRE },
};

/*
  initialise the sensor to required settings
 */
bool AP_RangeFinder_PulsedLightLRF::init(void)
{
    if (!_dev || !_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }
    _dev->set_retries(3);

    // LidarLite needs split transfers
    _dev->set_split_transfers(true);

    if (rftype == RangeFinder::RangeFinder_TYPE_PLI2CV3) {
        v2_hardware = true;
    } else {
        // auto-detect v1 vs v2
        if (!(_dev->read_registers(LL40LS_HW_VERSION, &hw_version, 1) &&
              hw_version > 0 &&
              _dev->read_registers(LL40LS_SW_VERSION, &sw_version, 1) &&
              sw_version > 0)) {
            printf("PulsedLightI2C: bad version 0x%02x 0x%02x\n", (unsigned)hw_version, (unsigned)sw_version);
            // invalid version information
            goto failed;
        }
        v2_hardware = (hw_version >= 0x15);
    }
    
    const struct settings_table *table;
    uint8_t num_settings;

    if (v2_hardware) {
        table = settings_v2;
        num_settings = sizeof(settings_v2) / sizeof(settings_table);
        phase = PHASE_COLLECT;
    } else {
        table = settings_v1;
        num_settings = sizeof(settings_v1) / sizeof(settings_table);
        phase = PHASE_MEASURE;    
    }

    _dev->setup_checked_registers(num_settings);

    for (uint8_t i = 0; i < num_settings; i++) {
        if (!_dev->write_register(table[i].reg, table[i].value, true)) {
            goto failed;
        }
    }

    printf("Found LidarLite device=0x%x v2=%d\n", _dev->get_bus_id(), (int)v2_hardware);
    
    _dev->get_semaphore()->give();

    _dev->register_periodic_callback(20000,
                                     FUNCTOR_BIND_MEMBER(&AP_RangeFinder_PulsedLightLRF::timer, void));
    return true;

failed:
    _dev->get_semaphore()->give();
    return false;
}

