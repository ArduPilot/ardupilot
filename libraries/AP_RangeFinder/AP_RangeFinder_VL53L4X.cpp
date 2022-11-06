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
  driver for ST VL53L4X lidar

  mostly based on https://github.com/adafruit/Adafruit_CircuitPython_VL53L4CD
 */
#include "AP_RangeFinder_VL53L4X.h"

#if AP_RANGEFINDER_VL53L4X_ENABLED

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const uint8_t VL53L4X_DEFAULT_CONFIGURATION[] = {
    0x12,  // 0x2d : I2C_HV__CONFIG                                 set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch
    0x01,  // 0x2e : I2C_HV__EXTSUP_CONFIG                          bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD)
    0x00,  // 0x2f : GPIO_HV_PAD__CTRL                              bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD)
    0x11,  // 0x30 : GPIO_HV_MUX__CTRL                              set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1)
    0x02,  // 0x31 : GPIO__TIO_HV_STATUS                            bit 1 = interrupt depending on the polarity
    0x00,  // 0x32 : GPIO__FIO_HV_STATUS
    0x02,  // 0x33 : ANA_CONFIG_SPAD_SEL_PSWIDTH
    0x08,  // 0x34 : ANA_CONFIG__VCSEL_PULSE_WIDTH_OFFSET
    0x00,  // 0x35 : ANA_CONFIG__FAST_OSC__CONFIG_CTRL
    0x08,  // 0x36 : SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS
    0x10,  // 0x37 : SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS
    0x01,  // 0x38 : SIGMA_ESTIMATOR__SIGMA_REF_MM
    0x01,  // 0x39 : ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM
    0x00,  // 0x3a : SPARE_HOST_CONFIG__STATIC_CONFIG_SPARE_0
    0x00,  // 0x3b : SPARE_HOST_CONFIG__STATIC_CONFIG_SPARE_1
    0x00,  // 0x3c : ALGO__RANGE_IGNORE_THRESHOLD_MCPS_HI
    0x00,  // 0x3d : ALGO__RANGE_IGNORE_THRESHOLD_MCPS_LO
    0xFF,  // 0x3e : ALGO__RANGE_IGNORE_VALID_HEIGHT_MM
    0x00,  // 0x3f : ALGO__RANGE_MIN_CLIP
    0x0F,  // 0x40 : ALGO__CONSISTENCY_CHECK__TOLERANCE
    0x00,  // 0x41 : SPARE_HOST_CONFIG__STATIC_CONFIG_SPARE_2
    0x00,  // 0x42 : SD_CONFIG__RESET_STAGES_MSB
    0x00,  // 0x43 : SD_CONFIG__RESET_STAGES_LSB
    0x00,  // 0x44 : GPH_CONFIG__STREAM_COUNT_UPDATE_VALUE
    0x00,  // 0x45 : GLOBAL_CONFIG__STREAM_DIVIDER
    0x20,  // 0x46 : SYSTEM__INTERRUPT_CONFIG_GPIO                  interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC
    0x0B,  // 0x47 : CAL_CONFIG__VCSEL_START
    0x00,  // 0x48 : CAL_CONFIG__REPEAT_RATE_HI
    0x00,  // 0x49 : CAL_CONFIG__REPEAT_RATE_LO
    0x02,  // 0x4a : GLOBAL_CONFIG__VCSEL_WIDTH
    0x14,  // 0x4b : PHASECAL_CONFIG__TIMEOUT_MACROP
    0x21,  // 0x4c : PHASECAL_CONFIG__TARGET
    0x00,  // 0x4d : PHASECAL_CONFIG__OVERRIDE
    0x00,  // 0x4e :
    0x02,  // 0x4f : DSS_CONFIG__ROI_MODE_CONTROL
    0x00,  // 0x50 : SYSTEM__THRESH_RATE_HIGH_HI
    0x00,  // 0x51 : SYSTEM__THRESH_RATE_HIGH_LO
    0x00,  // 0x52 : SYSTEM__THRESH_RATE_LOW_HI
    0x00,  // 0x53 : SYSTEM__THRESH_RATE_LOW_LO
    0xC8,  // 0x54 : DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT_HI
    0x00,  // 0x55 : DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT_LO
    0x00,  // 0x56 : DSS_CONFIG__MANUAL_BLOCK_SELECT
    0x38,  // 0x57 : DSS_CONFIG__APERTURE_ATTENUATION
    0xFF,  // 0x58 : DSS_CONFIG__MAX_SPADS_LIMIT
    0x01,  // 0x59 : DSS_CONFIG__MIN_SPADS_LIMIT
    0x00,  // 0x5a : MM_CONFIG__TIMEOUT_MACROP_A_HI
    0x08,  // 0x5b : MM_CONFIG__TIMEOUT_MACROP_A_LO
    0x00,  // 0x5c : MM_CONFIG__TIMEOUT_MACROP_B_HI
    0x00,  // 0x5d : MM_CONFIG__TIMEOUT_MACROP_B_LO
    0x01,  // 0x5e : RANGE_CONFIG__TIMEOUT_MACROP_A_HI
    0xCC,  // 0x5f : RANGE_CONFIG__TIMEOUT_MACROP_A_LO
    0x07,  // 0x60 : RANGE_CONFIG__VCSEL_PERIOD_A
    0x01,  // 0x61 : RANGE_CONFIG__TIMEOUT_MACROP_B_HI
    0xF1,  // 0x62 : RANGE_CONFIG__TIMEOUT_MACROP_B_LO
    0x05,  // 0x63 : RANGE_CONFIG__VCSEL_PERIOD_B
    0x00,  // 0x64 : RANGE_CONFIG__SIGMA_THRESH_HI                  Sigma threshold MSB (mm in 14.2 format for MSB+LSB), default value 90 mm
    0xA0,  // 0x65 : RANGE_CONFIG__SIGMA_THRESH_LO                  Sigma threshold LSB
    0x00,  // 0x66 : RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS_HI Min count Rate MSB (MCPS in 9.7 format for MSB+LSB)
    0x80,  // 0x67 : RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS_LO Min count Rate LSB
    0x08,  // 0x68 : RANGE_CONFIG__VALID_PHASE_LOW
    0x38,  // 0x69 : RANGE_CONFIG__VALID_PHASE_HIGH
    0x00,  // 0x6a :
    0x00,  // 0x6b :
    0x00,  // 0x6c : SYSTEM__INTERMEASUREMENT_PERIOD_3              Intermeasurement period MSB, 32 bits register
    0x00,  // 0x6d : SYSTEM__INTERMEASUREMENT_PERIOD_2              Intermeasurement period
    0x0F,  // 0x6e : SYSTEM__INTERMEASUREMENT_PERIOD_1              Intermeasurement period
    0x89,  // 0x6f : SYSTEM__INTERMEASUREMENT_PERIOD_0              Intermeasurement period LSB
    0x00,  // 0x70 : SYSTEM__FRACTIONAL_ENABLE
    0x00,  // 0x71 : SYSTEM__GROUPED_PARAMETER_HOLD_0
    0x00,  // 0x72 : SYSTEM__THRESH_HIGH_HI                         distance threshold high MSB (in mm, MSB+LSB)
    0x00,  // 0x73 : SYSTEM__THRESH_HIGH_LO                         distance threshold high LSB
    0x00,  // 0x74 : SYSTEM__THRESH_LOW_HI                          distance threshold low MSB ( in mm, MSB+LSB)
    0x00,  // 0x75 : SYSTEM__THRESH_LOW_LO                          distance threshold low LSB
    0x00,  // 0x76 : SYSTEM__ENABLE_XTALK_PER_QUADRANT
    0x01,  // 0x77 : SYSTEM__SEED_CONFIG
    0x07,  // 0x78 : SD_CONFIG__WOI_SD0
    0x05,  // 0x79 : SD_CONFIG__WOI_SD1
    0x06,  // 0x7a : SD_CONFIG__INITIAL_PHASE_SD0
    0x06,  // 0x7b : SD_CONFIG__INITIAL_PHASE_SD1
    0x00,  // 0x7c : SYSTEM__GROUPED_PARAMETER_HOLD_1
    0x00,  // 0x7d : SD_CONFIG__FIRST_ORDER_SELECT
    0x02,  // 0x7e : SD_CONFIG__QUANTIFIER
    0xC7,  // 0x7f : ROI_CONFIG__USER_ROI_CENTRE_SPAD
    0xFF,  // 0x80 : ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE
    0x8B,  // 0x81 : SYSTEM__SEQUENCE_CONFIG
    0x00,  // 0x82 : SYSTEM__GROUPED_PARAMETER_HOLD
    0x00,  // 0x83 : POWER_MANAGEMENT__GO1_POWER_FORCE
    0x00,  // 0x84 : SYSTEM__STREAM_COUNT_CTRL
    0x01,  // 0x85 : FIRMWARE__ENABLE
    0x00,  // 0x86 : SYSTEM__INTERRUPT_CLEAR                        clear interrupt, 0x01=clear
    0x00,  // 0x87 : SYSTEM__MODE_START                             ranging, 0x00=stop, 0x40=start
};

static const uint8_t MEASUREMENT_TIME_MS = 50; // Start continuous readings at a rate of one measurement every 50 ms

AP_RangeFinder_VL53L4X::AP_RangeFinder_VL53L4X(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev)
    : AP_RangeFinder_Backend(_state, _params)
    , dev(std::move(_dev)) {}


// detect tests if a VL53L4X is present and returns the sensor instance
AP_RangeFinder_Backend *AP_RangeFinder_VL53L4X::detect(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    if (!dev) {
        return nullptr;
    }

    AP_RangeFinder_VL53L4X *sensor = new AP_RangeFinder_VL53L4X(_state, _params, std::move(dev));

    if (!sensor) {
        delete sensor;
        return nullptr;
    }

    sensor->dev->get_semaphore()->take_blocking();

    if (!sensor->check_id() || !sensor->init()) {
        sensor->dev->get_semaphore()->give();
        delete sensor;
        return nullptr;
    }

    sensor->dev->get_semaphore()->give();
    return sensor;
}

// init writes the default configration, calibrate the sensor and starts the measurements
bool AP_RangeFinder_VL53L4X::init()
{
    // we need to do resets and delays in order to configure the sensor, don't do this if we are trying to fast boot
    if (hal.util->was_watchdog_armed()) {
        return false;
    }

    // reset the chip, we make assumptions later on that we are on a clean power on of the sensor
    if (!(reset() && wait_for_boot())) {
        return false;
    }

    for (uint16_t reg = 0x002D; reg <= 0x0087; reg++) {
        if (!write_register(reg, VL53L4X_DEFAULT_CONFIGURATION[reg - 0x002D])) {
            return false;
        }
        hal.scheduler->delay(1);
    }

    if (!read_register16(RESULT_OSC_CALIBRATE_VAL, osc_calibrate_val)) {
        return false;
    }

    if (!start_continuous()) {
        return false;
    }

    if (!write_register(SYSTEM_START, 0x00)) {
        return false;
    }

    // writing calibration VHV value
    if (!(
            write_register(VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09) &&
            write_register(VHV_CONFIG_INIT, 0x00) &&
            write_register16(DSS_CONFIG_TARGET_TOTAL_RATE_MCPS, 0x0500)
        )) {
        return false;
    }

    // start measurements regular
    if (!set_inter_measurement(MEASUREMENT_TIME_MS) || !set_timing_budget(50) || !start_continuous()) {
        return false;
    }

    // call timer() every MEASUREMENT_TIME_MS. We expect new data to be available every MEASUREMENT_TIME_MS
    dev->register_periodic_callback(MEASUREMENT_TIME_MS * 1000, FUNCTOR_BIND_MEMBER(&AP_RangeFinder_VL53L4X::timer, void));
    return true;
}

// reset softresets the sensor using i2c commands
bool AP_RangeFinder_VL53L4X::reset(void)
{
    if (dev->get_bus_id() != 0x29) {
        // if sensor is on a different port than the default do not  reset sensor otherwise we will lose the addess.
        // we assume it is already confirgured.
        return true;
    }
    if (!write_register(SOFT_RESET, 0x00)) {
        return false;
    }
    hal.scheduler->delay(1);
    if (!write_register(SOFT_RESET, 0x01)) {
        return false;
    }
    hal.scheduler->delay(1000);
    return true;
}

// wait_for_boot checks the sensor if it is ready to configure
bool AP_RangeFinder_VL53L4X::wait_for_boot(void)
{
    uint8_t res;
    for (uint16_t i = 0; i < 1000; i++) {
        if (read_register(FIRMWARE_SYSTEM_STATUS, res) && res == 0x03) {
            return true;
        }
        hal.scheduler->delay(1);
    }
    return false;
}

// check_id checks for the correct model of the VL53L4 TOF sensor
bool AP_RangeFinder_VL53L4X::check_id(void)
{
    uint16_t v;
    if (!read_register16(IDENTIFICATION_MODEL_ID, v)) {
        return false;
    }
    if (v != 0xEBAA) {
        return false;
    }
    return true;
}

// interrupt_polarity returns the interrupt polarity set in register GPIO_HV_MUX_CTRL
bool AP_RangeFinder_VL53L4X::interrupt_polarity(uint8_t *value)
{
    uint8_t v;
    if (!read_register(GPIO_HV_MUX_CTRL, v)) {
        return false;
    }
    v = v & 0x10;
    *value = !(v >> 4);

    return true;
}

// set_timing_budget sets the allowed time for meassurements
bool AP_RangeFinder_VL53L4X::set_timing_budget(uint32_t budget_ms)
{
    // budget_ms must be between 10ms and 200ms
    // budget_ms must be below inter measurement period

    uint16_t osc_freq = 0;
    if (!read_register16(OSC_MEASURED_FAST_OSC_FREQUENCY, osc_freq)) {
        return false;
    }

    uint32_t timing_budget_us = budget_ms * 1000;
    uint32_t macro_period_us = calc_macro_period(osc_freq);

    timing_budget_us -= 4300;
    timing_budget_us /= 2;

    uint16_t ms_byte = 0;
    timing_budget_us <<= 12;
    uint32_t tmp = macro_period_us * 16;
    uint32_t ls_byte = ((timing_budget_us + ((tmp >> 6) >> 1)) / (tmp >> 6)) - 1;
    while ((ls_byte & 0xFFFFFF00) > 0) {
        ls_byte >>= 1;
        ms_byte += 1;
    }
    ms_byte = (ms_byte << 8) + (ls_byte & 0xFF);
    if (!write_register16(RANGE_CONFIG_A, ms_byte)) {
        return false;
    }

    ms_byte = 0;
    tmp = macro_period_us * 12;
    ls_byte = ((timing_budget_us + ((tmp >> 6) >> 1)) / (tmp >> 6)) - 1;
    while ((ls_byte & 0xFFFFFF00) > 0) {
        ls_byte >>= 1;
        ms_byte += 1;
    }
    ms_byte = (ms_byte << 8) + (ls_byte & 0xFF);

    return write_register16(RANGE_CONFIG_B, ms_byte);
}

// set_inter_measurement determines how often the sensor takes a measurement
bool AP_RangeFinder_VL53L4X::set_inter_measurement(uint32_t period_ms)
{
    uint32_t clock_pll = osc_calibrate_val & 0x3FF;
    uint32_t int_meas = 1.055f * period_ms * clock_pll;

    if (!write_register32(INTERMEASUREMENT_MS, int_meas)) {
        return false;
    }
    return true;
}

// start_continuous starts continuous ranging measurements
bool AP_RangeFinder_VL53L4X::start_continuous()
{
    if (!write_register(SYSTEM_START, 0x40)) {
        return false;
    }

    uint16_t tries = 2500;
    while (!data_ready()) {
        tries--;
        hal.scheduler->delay(1);
        if (tries == 0) {
            return false;
        }
    }

    if (!write_register(SYSTEM_INTERRUPT_CLEAR, 0x01)) {
        return false;
    }

    return true;
}

// data_ready checks register if interrupt is triggered. it is independet from the polarity
bool AP_RangeFinder_VL53L4X::data_ready(void)
{
    uint8_t polarity;
    if (!interrupt_polarity(&polarity)) {
        return false;
    }

    uint8_t status;
    if (!read_register(GPIO_TIO_HV_STATUS, status)) {
        return false;
    }
    return (status & 0x01) == polarity;
}

// get_reading returns the distance result
bool AP_RangeFinder_VL53L4X::get_reading(uint16_t &reading_mm)
{
    uint8_t tries = 25;
    while (!data_ready()) {
        tries--;
        hal.scheduler->delay(1);
        if (tries == 0) {
            return false;
        }
    }

    uint8_t range_status = 0;

    if (!(read_register(RESULT_RANGE_STATUS, range_status) &&
          read_register16(RESULT_DISTANCE, reading_mm))) {
        return false;
    }

    if (!write_register(SYSTEM_INTERRUPT_CLEAR, 0x01)) { // sys_interrupt_clear_range
        return false;
    }

    return true;
}

uint32_t AP_RangeFinder_VL53L4X::calc_macro_period(uint16_t osc_freq) const
{
    uint32_t pll_period_us = ((uint32_t)0x40000000) / osc_freq;
    uint32_t macro_period_us = ((uint32_t)2304 )* pll_period_us;
    return macro_period_us  >> 6;
}


// timer called at 20Hz
void AP_RangeFinder_VL53L4X::timer(void)
{
    uint16_t range_mm;
    if (get_reading(range_mm)) {
        WITH_SEMAPHORE(_sem);
        sum_mm += range_mm;
        counter++;
    }
}


// update the state of the sensor and calculates the avarage
void AP_RangeFinder_VL53L4X::update(void)
{
    WITH_SEMAPHORE(_sem);
    if (counter > 0) {
        state.distance_m = (sum_mm * 0.001f) / counter;
        state.last_reading_ms = AP_HAL::millis();
        update_status();
        sum_mm = 0;
        counter = 0;
    } else if (AP_HAL::millis() - state.last_reading_ms > 200) {
        // if no updates for 0.2s set no-data
        set_status(RangeFinder::Status::NoData);
    }
}


bool AP_RangeFinder_VL53L4X::read_register(uint16_t reg, uint8_t &value)
{
    uint8_t b[2] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF) };
    return dev->transfer(b, 2, &value, 1);
}

bool AP_RangeFinder_VL53L4X::read_register16(uint16_t reg, uint16_t & value)
{
    uint16_t v = 0;
    uint8_t b[2] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF) };
    if (!dev->transfer(b, 2, (uint8_t *)&v, 2)) {
        return false;
    }
    value = be16toh(v);
    return true;
}

bool AP_RangeFinder_VL53L4X::write_register(uint16_t reg, uint8_t value)
{
    uint8_t b[3] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF), value };
    return dev->transfer(b, 3, nullptr, 0);
}

bool AP_RangeFinder_VL53L4X::write_register16(uint16_t reg, uint16_t value)
{
    uint8_t b[4] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF), uint8_t(value >> 8), uint8_t(value & 0xFF) };
    return dev->transfer(b, 4, nullptr, 0);
}

bool AP_RangeFinder_VL53L4X::write_register32(uint16_t reg, uint32_t value)
{
    uint8_t b[6] = { uint8_t(reg >> 8),
                     uint8_t(reg & 0xFF),
                     uint8_t((value >> 24) & 0xFF),
                     uint8_t((value >> 16) & 0xFF),
                     uint8_t((value >>  8) & 0xFF),
                     uint8_t((value)       & 0xFF)
                   };
    return dev->transfer(b, 6, nullptr, 0);
}

#endif  // AP_RANGEFINDER_VL53L4X_ENABLED