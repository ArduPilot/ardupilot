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
  driver for ST VL53L1X lidar

  Many thanks to Pololu, https://github.com/pololu/vl53l1x-arduino and
  the ST example code
 */
#include "AP_RangeFinder_VL53L1X.h"

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initializes the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_VL53L1X::AP_RangeFinder_VL53L1X(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev)
    : AP_RangeFinder_Backend(_state, _params)
    , dev(std::move(_dev)) {}

/*
   detect if a VL53L1X rangefinder is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
AP_RangeFinder_Backend *AP_RangeFinder_VL53L1X::detect(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    if (!dev) {
        return nullptr;
    }

    AP_RangeFinder_VL53L1X *sensor
        = new AP_RangeFinder_VL53L1X(_state, _params, std::move(dev));

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

// check sensor ID registers
bool AP_RangeFinder_VL53L1X::check_id(void)
{
    uint8_t v1, v2;
    if (!(read_register(0x010F, v1) &&
          read_register(0x0110, v2))) {
        return false;
    }

    if ((v1 != 0xEA) ||
        (v2 != 0xCC)) {
        return false;
    }
    printf("Detected VL53L1X on bus 0x%x\n", dev->get_bus_id());
    return true;
}

/*
  initialise sensor
 */
bool AP_RangeFinder_VL53L1X::init()
{
    uint8_t pad_i2c_hv_extsup_config = 0;
    uint16_t mm_config_outer_offset_mm = 0;
    if (!(// setup for 2.8V operation
          read_register(PAD_I2C_HV__EXTSUP_CONFIG, pad_i2c_hv_extsup_config) &&
          write_register(PAD_I2C_HV__EXTSUP_CONFIG,
                         pad_i2c_hv_extsup_config | 0x01) &&

          // store oscillator info for later use
          read_register16(OSC_MEASURED__FAST_OSC__FREQUENCY, fast_osc_frequency) &&
          read_register16(RESULT__OSC_CALIBRATE_VAL, osc_calibrate_val) &&

          // static config
          write_register16(DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, TargetRate) && // should already be this value after reset
          write_register(GPIO__TIO_HV_STATUS, 0x02) &&
          write_register(SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8) && // tuning parm default
          write_register(SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16) && // tuning parm default
          write_register(ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01) &&
          write_register(ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF) &&
          write_register(ALGO__RANGE_MIN_CLIP, 0) && // tuning parm default
          write_register(ALGO__CONSISTENCY_CHECK__TOLERANCE, 2) && // tuning parm default

          // general config
          write_register16(SYSTEM__THRESH_RATE_HIGH, 0x0000) &&
          write_register16(SYSTEM__THRESH_RATE_LOW, 0x0000) &&
          write_register(DSS_CONFIG__APERTURE_ATTENUATION, 0x38) &&

          // timing config
          write_register16(RANGE_CONFIG__SIGMA_THRESH, 360) && // tuning parm default
          write_register16(RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192) && // tuning parm default

          // dynamic config
          write_register(SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01) &&
          write_register(SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01) &&
          write_register(SD_CONFIG__QUANTIFIER, 2) && // tuning parm default

          // from VL53L1_preset_mode_timed_ranging_*
          // GPH is 0 after reset, but writing GPH0 and GPH1 above seem to set GPH to 1,
          // and things don't seem to work if we don't set GPH back to 0 (which the API
          // does here).
          write_register(SYSTEM__GROUPED_PARAMETER_HOLD, 0x00) &&
          write_register(SYSTEM__SEED_CONFIG, 1) && // tuning parm default

          // from VL53L1_config_low_power_auto_mode
          write_register(SYSTEM__SEQUENCE_CONFIG, 0x8B) && // VHV, PHASECAL, DSS1, RANGE
          write_register16(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8) &&
          write_register(DSS_CONFIG__ROI_MODE_CONTROL, 2) && // REQUESTED_EFFFECTIVE_SPADS
          read_register16(MM_CONFIG__OUTER_OFFSET_MM, mm_config_outer_offset_mm) &&
          setDistanceMode(Long) &&
          setMeasurementTimingBudget(40000) &&
          // the API triggers this change in VL53L1_init_and_start_range() once a
          // measurement is started; assumes MM1 and MM2 are disabled
          write_register16(ALGO__PART_TO_PART_RANGE_OFFSET_MM, mm_config_outer_offset_mm * 4) &&
          // set continuous mode
          startContinuous(50)
          )) {
              return false;
          }

    // call timer() every 50ms. We expect new data to be available every 50ms
    dev->register_periodic_callback(50000,
                                    FUNCTOR_BIND_MEMBER(&AP_RangeFinder_VL53L1X::timer, void));

    return true;
}

// set distance mode to Short, Medium, or Long
// based on VL53L1_SetDistanceMode()
bool AP_RangeFinder_VL53L1X::setDistanceMode(DistanceMode distance_mode)
{
    // save existing timing budget
    uint32_t budget_us = 0;
    if (!getMeasurementTimingBudget(budget_us)) {
        return false;
    }

    switch (distance_mode) {
        case Short:
            // from VL53L1_preset_mode_standard_ranging_short_range()

            if (!(// timing config
                  write_register(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07) &&
                  write_register(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05) &&
                  write_register(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38) &&

                  // dynamic config
                  write_register(SD_CONFIG__WOI_SD0, 0x07) &&
                  write_register(SD_CONFIG__WOI_SD1, 0x05) &&
                  write_register(SD_CONFIG__INITIAL_PHASE_SD0, 6) && // tuning parm default
                  write_register(SD_CONFIG__INITIAL_PHASE_SD1, 6))) { // tuning parm default
                return false;
            }

            break;

        case Medium:
            // from VL53L1_preset_mode_standard_ranging()

            if (!(// timing config
                  write_register(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B) &&
                  write_register(RANGE_CONFIG__VCSEL_PERIOD_B, 0x09) &&
                  write_register(RANGE_CONFIG__VALID_PHASE_HIGH, 0x78) &&

                  // dynamic config
                  write_register(SD_CONFIG__WOI_SD0, 0x0B) &&
                  write_register(SD_CONFIG__WOI_SD1, 0x09) &&
                  write_register(SD_CONFIG__INITIAL_PHASE_SD0, 10) && // tuning parm default
                  write_register(SD_CONFIG__INITIAL_PHASE_SD1, 10))) { // tuning parm default
                return false;
            }

            break;

        case Long:
            // from VL53L1_preset_mode_standard_ranging_long_range()

            if (!(// timing config
                  write_register(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F) &&
                  write_register(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D) &&
                  write_register(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8) &&

                  // dynamic config
                  write_register(SD_CONFIG__WOI_SD0, 0x0F) &&
                  write_register(SD_CONFIG__WOI_SD1, 0x0D) &&
                  write_register(SD_CONFIG__INITIAL_PHASE_SD0, 14) && // tuning parm default
                  write_register(SD_CONFIG__INITIAL_PHASE_SD1, 14))) { // tuning parm default
                return false;
            }

            break;

        default:
            // unrecognized mode - do nothing
            return false;
    }

    // reapply timing budget
    return setMeasurementTimingBudget(budget_us);
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement. A longer timing budget allows for more accurate measurements.
// based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
bool AP_RangeFinder_VL53L1X::setMeasurementTimingBudget(uint32_t budget_us)
{
    // assumes PresetMode is LOWPOWER_AUTONOMOUS
    if (budget_us <= TimingGuard) {
        return false;
    }

    uint32_t range_config_timeout_us = budget_us - TimingGuard;
    if (range_config_timeout_us > 1100000) {
        return false; // FDA_MAX_TIMING_BUDGET_US * 2
    }

    range_config_timeout_us /= 2;

    // VL53L1_calc_timeout_register_values() begin

    uint8_t range_config_vcsel_period = 0;
    if (!read_register(RANGE_CONFIG__VCSEL_PERIOD_A, range_config_vcsel_period)) {
        return false;
    }

    // "Update Macro Period for Range A VCSEL Period"
    uint32_t macro_period_us = calcMacroPeriod(range_config_vcsel_period);

    // "Update Phase timeout - uses Timing A"
    // Timeout of 1000 is tuning parm default (TIMED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT)
    // via VL53L1_get_preset_mode_timing_cfg().
    uint32_t phasecal_timeout_mclks = timeoutMicrosecondsToMclks(1000, macro_period_us);
    if (phasecal_timeout_mclks > 0xFF) {
        phasecal_timeout_mclks = 0xFF;
    }

    if (!( write_register(PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks) &&

          // "Update MM Timing A timeout"
          // Timeout of 1 is tuning parm default (LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US_DEFAULT)
          // via VL53L1_get_preset_mode_timing_cfg(). With the API, the register
          // actually ends up with a slightly different value because it gets assigned,
          // retrieved, recalculated with a different macro period, and reassigned,
          // but it probably doesn't matter because it seems like the MM ("mode
          // mitigation"?) sequence steps are disabled in low power auto mode anyway.
          write_register16(MM_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(
              timeoutMicrosecondsToMclks(1, macro_period_us))) &&

          // "Update Range Timing A timeout"
          write_register16(RANGE_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(
              timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us))) &&

          // "Update Macro Period for Range B VCSEL Period"
          read_register(RANGE_CONFIG__VCSEL_PERIOD_B, range_config_vcsel_period)
         )) {
        return false;
    }

    // "Update Macro Period for Range B VCSEL Period"
    macro_period_us = calcMacroPeriod(range_config_vcsel_period);

    // "Update MM Timing B timeout"
    // (See earlier comment about MM Timing A timeout.)
    return write_register16(MM_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(
               timeoutMicrosecondsToMclks(1, macro_period_us))) &&

           // "Update Range Timing B timeout"
           write_register16(RANGE_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(
               timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));
}

// Get the measurement timing budget in microseconds
// based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
bool AP_RangeFinder_VL53L1X::getMeasurementTimingBudget(uint32_t &budget)
{
    // assumes PresetMode is LOWPOWER_AUTONOMOUS and these sequence steps are
    // enabled: VHV, PHASECAL, DSS1, RANGE

    // "Update Macro Period for Range A VCSEL Period"
    uint8_t range_config_vcsel_period_a = 0;
    if (!read_register(RANGE_CONFIG__VCSEL_PERIOD_A, range_config_vcsel_period_a)) {
        return false;
    }

    uint32_t macro_period_us = calcMacroPeriod(range_config_vcsel_period_a);

    uint16_t timeout_macrop_a = 0;
    if (!read_register16(RANGE_CONFIG__TIMEOUT_MACROP_A, timeout_macrop_a)) {
        return false;
    }

    // "Get Range Timing A timeout"
    uint32_t range_config_timeout_us = timeoutMclksToMicroseconds(decodeTimeout(timeout_macrop_a), macro_period_us);

    budget = 2 * range_config_timeout_us + TimingGuard;
    return true;
}

// Start continuous ranging measurements, with the given inter-measurement
// period in milliseconds determining how often the sensor takes a measurement.
bool AP_RangeFinder_VL53L1X::startContinuous(uint32_t period_ms)
{
    // fix for actual measurement period shorter than set
    uint32_t adjusted_period_ms = period_ms + (period_ms * 64 / 1000);

    // from VL53L1_set_inter_measurement_period_ms()
    return write_register32(SYSTEM__INTERMEASUREMENT_PERIOD, adjusted_period_ms * osc_calibrate_val) &&
           write_register(SYSTEM__INTERRUPT_CLEAR, 0x01) && // sys_interrupt_clear_range
           write_register(SYSTEM__MODE_START, 0x40); // mode_range__timed
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L1_decode_timeout()
uint32_t AP_RangeFinder_VL53L1X::decodeTimeout(uint16_t reg_val)
{
    return ((uint32_t)(reg_val & 0xFF) << (reg_val >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L1_encode_timeout()
uint16_t AP_RangeFinder_VL53L1X::encodeTimeout(uint32_t timeout_mclks)
{
    // encoded format: "(LSByte * 2^MSByte) + 1"
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks > 0) {
        ls_byte = timeout_mclks - 1;
        while ((ls_byte & 0xFFFFFF00) > 0) {
            ls_byte >>= 1;
            ms_byte++;
        }
        return (ms_byte << 8) | (ls_byte & 0xFF);
    }
    else {
        return 0;
    }
}

// Convert sequence step timeout from macro periods to microseconds with given
// macro period in microseconds (12.12 format)
// based on VL53L1_calc_timeout_us()
uint32_t AP_RangeFinder_VL53L1X::timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us)
{
    return ((uint64_t)timeout_mclks * macro_period_us + 0x800) >> 12;
}

// Convert sequence step timeout from microseconds to macro periods with given
// macro period in microseconds (12.12 format)
// based on VL53L1_calc_timeout_mclks()
uint32_t AP_RangeFinder_VL53L1X::timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us)
{
    return (((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
}

// Calculate macro period in microseconds (12.12 format) with given VCSEL period
// assumes fast_osc_frequency has been read and stored
// based on VL53L1_calc_macro_period_us()
uint32_t AP_RangeFinder_VL53L1X::calcMacroPeriod(uint8_t vcsel_period)
{
    // from VL53L1_calc_pll_period_us()
    // fast osc frequency in 4.12 format; PLL period in 0.24 format
    uint32_t pll_period_us = ((uint32_t)0x01 << 30) / fast_osc_frequency;

    // from VL53L1_decode_vcsel_period()
    uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;

    // VL53L1_MACRO_PERIOD_VCSEL_PERIODS = 2304
    uint32_t macro_period_us = (uint32_t)2304 * pll_period_us;
    macro_period_us >>= 6;
    macro_period_us *= vcsel_period_pclks;
    macro_period_us >>= 6;

    return macro_period_us;
}

// "Setup ranges after the first one in low power auto mode by turning off
// FW calibration steps and programming static values"
// based on VL53L1_low_power_auto_setup_manual_calibration()
bool AP_RangeFinder_VL53L1X::setupManualCalibration(void)
{
    uint8_t saved_vhv_init = 0;
    uint8_t saved_vhv_timeout = 0;
    uint8_t phasecal_result_vcsel_start = 0;

    return // "save original vhv configs"
           read_register(VHV_CONFIG__INIT, saved_vhv_init) &&
           read_register(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, saved_vhv_timeout) &&

           // "disable VHV init"
           write_register(VHV_CONFIG__INIT, saved_vhv_init & 0x7F) &&

          // "set loop bound to tuning param"
          write_register(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
                         (saved_vhv_timeout & 0x03) + (3 << 2)) && // tuning parm default (LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT)

          // "override phasecal"
          write_register(PHASECAL_CONFIG__OVERRIDE, 0x01) &&
          read_register(PHASECAL_RESULT__VCSEL_START, phasecal_result_vcsel_start) &&
          write_register(CAL_CONFIG__VCSEL_START, phasecal_result_vcsel_start);
}

// check if sensor has new reading available
// assumes interrupt is active low (GPIO_HV_MUX__CTRL bit 4 is 1)
bool AP_RangeFinder_VL53L1X::dataReady(void)
{
    uint8_t gpio_tio_hv_status = 0;

    return read_register(GPIO__TIO_HV_STATUS, gpio_tio_hv_status) &&
           ((gpio_tio_hv_status & 0x01) == 0);
}

// read - return last value measured by sensor
bool AP_RangeFinder_VL53L1X::get_reading(uint16_t &reading_mm)
{
    uint8_t tries = 10;
    while (!dataReady()) {
        tries--;
        hal.scheduler->delay(1);
        if (tries == 0) {
            return false;
        }
    }

    uint8_t range_status = 0;

    if (!(read_register(RESULT__RANGE_STATUS, range_status) &&
          read_register16(RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, reading_mm))) {
        return false;
    }

    // "apply correction gain"
    // gain factor of 2011 is tuning parm default (VL53L1_TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT)
    // Basically, this appears to scale the result by 2011/2048, or about 98%
    // (with the 1024 added for proper rounding).
    reading_mm = ((uint32_t)reading_mm * 2011 + 0x0400) / 0x0800;

    if (!write_register(SYSTEM__INTERRUPT_CLEAR, 0x01)) { // sys_interrupt_clear_range
        return false;
    }

    switch ((DeviceError)range_status) {
      case RANGECOMPLETE:
        break;

      default:
#ifdef VL53L1X_DEBUG
        hal.console->printf("VL53L1X: %d ms status %d\n", AP_HAL::millis(), (int)range_status);
#endif // VL53L1X_DEBUG
        return false;
    }

    if (!calibrated) {
        calibrated = setupManualCalibration();
    }

    return calibrated;
}

bool AP_RangeFinder_VL53L1X::read_register(uint16_t reg, uint8_t &value)
{
    uint8_t b[2] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF) };
    return dev->transfer(b, 2, &value, 1);
}

bool AP_RangeFinder_VL53L1X::read_register16(uint16_t reg, uint16_t & value)
{
    uint16_t v = 0;
    uint8_t b[2] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF) };
    if (!dev->transfer(b, 2, (uint8_t *)&v, 2)) {
        return false;
    }
    value = be16toh(v);
    return true;
}

bool AP_RangeFinder_VL53L1X::write_register(uint16_t reg, uint8_t value)
{
    uint8_t b[3] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF), value };
    return dev->transfer(b, 3, nullptr, 0);
}

bool AP_RangeFinder_VL53L1X::write_register16(uint16_t reg, uint16_t value)
{
    uint8_t b[4] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF), uint8_t(value >> 8), uint8_t(value & 0xFF) };
    return dev->transfer(b, 4, nullptr, 0);
}

bool AP_RangeFinder_VL53L1X::write_register32(uint16_t reg, uint32_t value)
{
    uint8_t b[6] = { uint8_t(reg >> 8),
                     uint8_t(reg & 0xFF),
                     uint8_t((value >> 24) & 0xFF),
                     uint8_t((value >> 16) & 0xFF),
                     uint8_t((value >>  8) & 0xFF),
                     uint8_t((value)       & 0xFF) };
    return dev->transfer(b, 6, nullptr, 0);
}

/*
  timer called at 20Hz
*/
void AP_RangeFinder_VL53L1X::timer(void)
{
    uint16_t range_mm;
    if ((get_reading(range_mm)) && (range_mm <= 4000)) {
        WITH_SEMAPHORE(_sem);
        sum_mm += range_mm;
        counter++;
    }
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_VL53L1X::update(void)
{
    WITH_SEMAPHORE(_sem);
    if (counter > 0) {
        state.distance_cm = sum_mm / (10*counter);
        state.last_reading_ms = AP_HAL::millis();
        update_status();
        sum_mm = 0;
        counter = 0;
    } else if (AP_HAL::millis() - state.last_reading_ms > 200) {
        // if no updates for 0.2s set no-data
        set_status(RangeFinder::RangeFinder_NoData);
    }
}
