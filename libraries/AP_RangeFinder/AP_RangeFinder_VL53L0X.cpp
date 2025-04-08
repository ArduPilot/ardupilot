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
  driver for ST VL53L0X lidar

  Many thanks to Pololu, https://github.com/pololu/vl53l0x-arduino and
  the ST example code
 */
#include "AP_RangeFinder_VL53L0X.h"

#if AP_RANGEFINDER_VL53L0X_ENABLED

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

enum regAddr
{
      SYSRANGE_START                              = 0x00,

      SYSTEM_THRESH_HIGH                          = 0x0C,
      SYSTEM_THRESH_LOW                           = 0x0E,

      SYSTEM_SEQUENCE_CONFIG                      = 0x01,
      SYSTEM_RANGE_CONFIG                         = 0x09,
      SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04,

      SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A,

      GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84,

      SYSTEM_INTERRUPT_CLEAR                      = 0x0B,

      RESULT_INTERRUPT_STATUS                     = 0x13,
      RESULT_RANGE_STATUS                         = 0x14,

      RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC,
      RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0,
      RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0,
      RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4,
      RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6,

      ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28,

      I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A,

      MSRC_CONFIG_CONTROL                         = 0x60,

      PRE_RANGE_CONFIG_MIN_SNR                    = 0x27,
      PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56,
      PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57,
      PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64,

      FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67,
      FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47,
      FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48,
      FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

      PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61,
      PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62,

      PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50,
      PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51,
      PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52,

      SYSTEM_HISTOGRAM_BIN                        = 0x81,
      HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33,
      HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55,

      FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70,
      FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71,
      FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72,
      CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20,

      MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46,

      SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF,
      IDENTIFICATION_MODEL_ID                     = 0xC0,
      IDENTIFICATION_REVISION_ID                  = 0xC2,

      OSC_CALIBRATE_VAL                           = 0xF8,

      GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0xB1,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0xB2,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0xB3,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0xB4,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0xB5,

      GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6,
      DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E,
      DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F,
      POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80,

      VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89,

      ALGO_PHASECAL_LIM                           = 0x30,
      ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30,
};

// tuning register settings
const AP_RangeFinder_VL53L0X::RegData AP_RangeFinder_VL53L0X::tuning_data[] =
{
    { 0xFF, 0x01 },
    { 0x00, 0x00 },

    { 0xFF, 0x00 },
    { 0x09, 0x00 },
    { 0x10, 0x00 },
    { 0x11, 0x00 },

    { 0x24, 0x01 },
    { 0x25, 0xFF },
    { 0x75, 0x00 },

    { 0xFF, 0x01 },
    { 0x4E, 0x2C },
    { 0x48, 0x00 },
    { 0x30, 0x20 },

    { 0xFF, 0x00 },
    { 0x30, 0x09 },
    { 0x54, 0x00 },
    { 0x31, 0x04 },
    { 0x32, 0x03 },
    { 0x40, 0x83 },
    { 0x46, 0x25 },
    { 0x60, 0x00 },
    { 0x27, 0x00 },
    { 0x50, 0x06 },
    { 0x51, 0x00 },
    { 0x52, 0x96 },
    { 0x56, 0x08 },
    { 0x57, 0x30 },
    { 0x61, 0x00 },
    { 0x62, 0x00 },
    { 0x64, 0x00 },
    { 0x65, 0x00 },
    { 0x66, 0xA0 },

    { 0xFF, 0x01 },
    { 0x22, 0x32 },
    { 0x47, 0x14 },
    { 0x49, 0xFF },
    { 0x4A, 0x00 },

    { 0xFF, 0x00 },
    { 0x7A, 0x0A },
    { 0x7B, 0x00 },
    { 0x78, 0x21 },

    { 0xFF, 0x01 },
    { 0x23, 0x34 },
    { 0x42, 0x00 },
    { 0x44, 0xFF },
    { 0x45, 0x26 },
    { 0x46, 0x05 },
    { 0x40, 0x40 },
    { 0x0E, 0x06 },
    { 0x20, 0x1A },
    { 0x43, 0x40 },

    { 0xFF, 0x00 },
    { 0x34, 0x03 },
    { 0x35, 0x44 },

    { 0xFF, 0x01 },
    { 0x31, 0x04 },
    { 0x4B, 0x09 },
    { 0x4C, 0x05 },
    { 0x4D, 0x04 },

    { 0xFF, 0x00 },
    { 0x44, 0x00 },
    { 0x45, 0x20 },
    { 0x47, 0x08 },
    { 0x48, 0x28 },
    { 0x67, 0x00 },
    { 0x70, 0x04 },
    { 0x71, 0x01 },
    { 0x72, 0xFE },
    { 0x76, 0x00 },
    { 0x77, 0x00 },

    { 0xFF, 0x01 },
    { 0x0D, 0x01 },

    { 0xFF, 0x00 },
    { 0x80, 0x01 },
    { 0x01, 0xF8 },

    { 0xFF, 0x01 },
    { 0x8E, 0x01 },
    { 0x00, 0x01 },
    { 0xFF, 0x00 },
    { 0x80, 0x00 },
};

AP_RangeFinder_VL53L0X::AP_RangeFinder_VL53L0X(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev)
    : AP_RangeFinder_Backend(_state, _params)
    , dev(std::move(_dev)) {}


/*
   detect if a VL53L0X rangefinder is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
AP_RangeFinder_Backend *AP_RangeFinder_VL53L0X::detect(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
	if (!dev) {
		return nullptr;
	}
    AP_RangeFinder_VL53L0X *sensor
        = NEW_NOTHROW AP_RangeFinder_VL53L0X(_state, _params, std::move(dev));

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
bool AP_RangeFinder_VL53L0X::check_id(void)
{
    uint8_t v1, v2;
    if (!dev->read_registers(0xC0, &v1, 1) ||
        !dev->read_registers(0xC1, &v2, 1) ||
        v1 != 0xEE ||
        v2 != 0xAA) {
        return false;
    }
    printf("Detected VL53L0X on bus 0x%x\n", unsigned(dev->get_bus_id()));
    return true;
}

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
bool AP_RangeFinder_VL53L0X::get_SPAD_info(uint8_t * count, bool *type_is_aperture)
{
    uint8_t tmp;

    write_register(0x80, 0x01);
    write_register(0xFF, 0x01);
    write_register(0x00, 0x00);

    write_register(0xFF, 0x06);
    write_register(0x83, read_register(0x83) | 0x04);
    write_register(0xFF, 0x07);
    write_register(0x81, 0x01);

    write_register(0x80, 0x01);

    write_register(0x94, 0x6b);
    write_register(0x83, 0x00);

    uint8_t tries = 50;
    while (read_register(0x83) == 0x00) {
        tries--;
        if (tries == 0) {
            return false;
        }
        hal.scheduler->delay(1);
    }
    write_register(0x83, 0x01);
    tmp = read_register(0x92);

    *count = tmp & 0x7f;
    *type_is_aperture = (tmp >> 7) & 0x01;

    write_register(0x81, 0x00);
    write_register(0xFF, 0x06);
    write_register(0x83, read_register(0x83) & ~0x04);
    write_register(0xFF, 0x01);
    write_register(0x00, 0x01);

    write_register(0xFF, 0x00);
    write_register(0x80, 0x00);

    return true;
}

// Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
void AP_RangeFinder_VL53L0X::getSequenceStepEnables(SequenceStepEnables * enables)
{
    uint8_t sequence_config = read_register(SYSTEM_SEQUENCE_CONFIG);

    enables->tcc          = (sequence_config >> 4) & 0x1;
    enables->dss          = (sequence_config >> 3) & 0x1;
    enables->msrc         = (sequence_config >> 2) & 0x1;
    enables->pre_range    = (sequence_config >> 6) & 0x1;
    enables->final_range  = (sequence_config >> 7) & 0x1;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
uint8_t AP_RangeFinder_VL53L0X::getVcselPulsePeriod(vcselPeriodType _type)
{
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)
    if (_type == VcselPeriodPreRange) {
        return decodeVcselPeriod(read_register(PRE_RANGE_CONFIG_VCSEL_PERIOD));
    } else if (_type == VcselPeriodFinalRange) {
        return decodeVcselPeriod(read_register(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
    }
    return 255;
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
uint32_t AP_RangeFinder_VL53L0X::timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

    return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
uint16_t AP_RangeFinder_VL53L0X::decodeTimeout(uint16_t reg_val)
{
    // format: "(LSByte * 2^MSByte) + 1"
    return (uint16_t)((reg_val & 0x00FF) <<
                      (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
void AP_RangeFinder_VL53L0X::getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts)
{
    timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodPreRange);

    timeouts->msrc_dss_tcc_mclks = read_register(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
    timeouts->msrc_dss_tcc_us =
        timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                                   timeouts->pre_range_vcsel_period_pclks);

    timeouts->pre_range_mclks =
        decodeTimeout(read_register16(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    timeouts->pre_range_us =
        timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                                   timeouts->pre_range_vcsel_period_pclks);

    timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodFinalRange);

    timeouts->final_range_mclks =
        decodeTimeout(read_register16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

    if (enables->pre_range) {
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;
    }

    timeouts->final_range_us =
        timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                                   timeouts->final_range_vcsel_period_pclks);
}


// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us
uint32_t AP_RangeFinder_VL53L0X::getMeasurementTimingBudget(void)
{
    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;

    uint16_t const StartOverhead     = 1910; // note that this is different than the value in set_
    uint16_t const EndOverhead        = 960;
    uint16_t const MsrcOverhead       = 660;
    uint16_t const TccOverhead        = 590;
    uint16_t const DssOverhead        = 690;
    uint16_t const PreRangeOverhead   = 660;
    uint16_t const FinalRangeOverhead = 550;

    // "Start and end overhead times always present"
    uint32_t budget_us = StartOverhead + EndOverhead;

    getSequenceStepEnables(&enables);
    getSequenceStepTimeouts(&enables, &timeouts);

    if (enables.tcc) {
        budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables.dss) {
        budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    } else if (enables.msrc) {
        budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables.pre_range) {
        budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range) {
        budget_us += (timeouts.final_range_us + FinalRangeOverhead);
    }

    measurement_timing_budget_us = budget_us; // store for internal reuse
    return budget_us;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
uint32_t AP_RangeFinder_VL53L0X::timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

    return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
uint16_t AP_RangeFinder_VL53L0X::encodeTimeout(uint16_t timeout_mclks)
{
    // format: "(LSByte * 2^MSByte) + 1"

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
    return 0;
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
bool AP_RangeFinder_VL53L0X::setMeasurementTimingBudget(uint32_t budget_us)
{
    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;

    uint16_t const StartOverhead      = 1320; // note that this is different than the value in get_
    uint16_t const EndOverhead        = 960;
    uint16_t const MsrcOverhead       = 660;
    uint16_t const TccOverhead        = 590;
    uint16_t const DssOverhead        = 690;
    uint16_t const PreRangeOverhead   = 660;
    uint16_t const FinalRangeOverhead = 550;

    uint32_t const MinTimingBudget = 20000;

    if (budget_us < MinTimingBudget) { return false; }

    uint32_t used_budget_us = StartOverhead + EndOverhead;

    getSequenceStepEnables(&enables);
    getSequenceStepTimeouts(&enables, &timeouts);

    if (enables.tcc) {
        used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables.dss) {
        used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    } else if (enables.msrc) {
        used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables.pre_range) {
        used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range) {
        used_budget_us += FinalRangeOverhead;

        // "Note that the final range timeout is determined by the timing
        // budget and the sum of all other timeouts within the sequence.
        // If there is no room for the final range timeout, then an error
        // will be set. Otherwise the remaining time will be applied to
        // the final range."

        if (used_budget_us > budget_us) {
            // "Requested timeout too big."
            return false;
        }

        uint32_t final_range_timeout_us = budget_us - used_budget_us;

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

        // "For the final range timeout, the pre-range timeout
        //  must be added. To do this both final and pre-range
        //  timeouts must be expressed in macro periods MClks
        //  because they have different vcsel periods."

        uint16_t final_range_timeout_mclks =
            timeoutMicrosecondsToMclks(final_range_timeout_us,
                                       timeouts.final_range_vcsel_period_pclks);

        if (enables.pre_range) {
            final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        write_register16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                         encodeTimeout(final_range_timeout_mclks));

        // set_sequence_step_timeout() end
        measurement_timing_budget_us = budget_us; // store for internal reuse
    }
    return true;
}

bool AP_RangeFinder_VL53L0X::init()
{
    // setup for 2.8V operation
    write_register(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
                   read_register(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);

    // "Set I2C standard mode"
    write_register(0x88, 0x00);

    write_register(0x80, 0x01);
    write_register(0xFF, 0x01);
    write_register(0x00, 0x00);
    stop_variable = read_register(0x91);
    write_register(0x00, 0x01);
    write_register(0xFF, 0x00);
    write_register(0x80, 0x00);

    // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
    write_register(MSRC_CONFIG_CONTROL, read_register(MSRC_CONFIG_CONTROL) | 0x12);

    // set final range signal rate limit to 0.25 MCPS (million counts per second)
    write_register16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, uint16_t(0.25 * (1 << 7)));

    write_register(SYSTEM_SEQUENCE_CONFIG, 0xFF);

    uint8_t spad_count;
    bool spad_type_is_aperture;
    if (!get_SPAD_info(&spad_count, &spad_type_is_aperture)) {
        printf("VL53L0X: Failed to get SPAD info\n");
        return false;
    }

    // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
    // the API, but the same data seems to be more easily readable from
    // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
    uint8_t ref_spad_map[6];
    if (!dev->read_registers(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6)) {
        printf("VL53L0X: Failed to read SPAD map\n");
        return false;
    }

    // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)
    write_register(0xFF, 0x01);
    write_register(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    write_register(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    write_register(0xFF, 0x00);
    write_register(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

    uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
    uint8_t spads_enabled = 0;

    for (uint8_t i = 0; i < 48; i++) {
        if (i < first_spad_to_enable || spads_enabled == spad_count) {
            // This bit is lower than the first one that should be enabled, or
            // (reference_spad_count) bits have already been enabled, so zero this bit
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1) {
            spads_enabled++;
        }
    }

    uint8_t reg_spad_map[7] = { GLOBAL_CONFIG_SPAD_ENABLES_REF_0, };
    memcpy(&reg_spad_map[1], ref_spad_map, 6);
    dev->transfer(reg_spad_map, 7, nullptr, 0);

    for (uint16_t i=0; i<ARRAY_SIZE(tuning_data); i++) {
        write_register(tuning_data[i].reg, tuning_data[i].value);
    }

    write_register(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    write_register(GPIO_HV_MUX_ACTIVE_HIGH, read_register(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
    write_register(SYSTEM_INTERRUPT_CLEAR, 0x01);

    // -- VL53L0X_SetGpioConfig() end

    measurement_timing_budget_us = getMeasurementTimingBudget();

    // "Disable MSRC and TCC by default"
    // MSRC = Minimum Signal Rate Check
    // TCC = Target CentreCheck
    // -- VL53L0X_SetSequenceStepEnable() begin

    write_register(SYSTEM_SEQUENCE_CONFIG, 0xE8);

    // -- VL53L0X_SetSequenceStepEnable() end

    // "Recalculate timing budget"
    setMeasurementTimingBudget(measurement_timing_budget_us);

    // VL53L0X_StaticInit() end

    // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

    // -- VL53L0X_perform_vhv_calibration() begin

    write_register(SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!performSingleRefCalibration(0x40)) {
        printf("VL53L0X: Failed SingleRefCalibration1\n");
        return false;
    }

    // -- VL53L0X_perform_vhv_calibration() end

    // -- VL53L0X_perform_phase_calibration() begin

    write_register(SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!performSingleRefCalibration(0x00)) {
        printf("VL53L0X: Failed SingleRefCalibration2\n");
        return false;
    }

    // -- VL53L0X_perform_phase_calibration() end

    // "restore the previous Sequence Config"
    write_register(SYSTEM_SEQUENCE_CONFIG, 0xE8);

    start_continuous();

    // call timer() every 33ms. We expect new data to be available every 33ms
    dev->register_periodic_callback(33000,
                                    FUNCTOR_BIND_MEMBER(&AP_RangeFinder_VL53L0X::timer, void));
    return true;
}


// based on VL53L0X_perform_single_ref_calibration()
bool AP_RangeFinder_VL53L0X::performSingleRefCalibration(uint8_t vhv_init_byte)
{
    write_register(SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

    uint8_t tries = 200;
    while ((read_register(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (tries-- == 0) {
            return false;
        }
        hal.scheduler->delay(1);
    }

    write_register(SYSTEM_INTERRUPT_CLEAR, 0x01);

    write_register(SYSRANGE_START, 0x00);

    return true;
}


// Start continuous ranging measurements
void AP_RangeFinder_VL53L0X::start_continuous(void)
{
    write_register(0x80, 0x01);
    write_register(0xFF, 0x01);
    write_register(0x00, 0x00);
    write_register(0x91, stop_variable);
    write_register(0x00, 0x01);
    write_register(0xFF, 0x00);
    write_register(0x80, 0x00);

    // continuous back-to-back mode
    write_register(SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK

    start_ms = AP_HAL::millis();
}

// read - return last value measured by sensor
bool AP_RangeFinder_VL53L0X::get_reading(uint16_t &reading_mm)
{
    if ((read_register(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (AP_HAL::millis() - start_ms > 200) {
            start_continuous();
        }
        return false;
    }

    // assumptions: Linearity Corrective Gain is 1000 (default);
    // fractional ranging is not enabled
    reading_mm = read_register16(RESULT_RANGE_STATUS + 10);
    write_register(SYSTEM_INTERRUPT_CLEAR, 0x01);

    return true;
}

void AP_RangeFinder_VL53L0X::write_register16(uint8_t reg, uint16_t value)
{
    uint8_t b[3] = { reg, uint8_t(value>>8), uint8_t(value) };
    dev->transfer(b, 3, nullptr, 0);
}

void AP_RangeFinder_VL53L0X::write_register(uint8_t reg, uint8_t value)
{
    dev->write_register(reg, value);
}

uint8_t AP_RangeFinder_VL53L0X::read_register(uint8_t reg)
{
    uint8_t v = 0;
    dev->read_registers(reg, &v, 1);
    return v;
}

uint16_t AP_RangeFinder_VL53L0X::read_register16(uint8_t reg)
{
    uint16_t v = 0;
    dev->transfer(&reg, 1, (uint8_t *)&v, 2);
    return be16toh(v);
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_VL53L0X::update(void)
{
    if (counter > 0) {
        state.distance_m = (sum_mm * 0.001f) / counter;
        state.last_reading_ms = AP_HAL::millis();
        sum_mm = 0;
        counter = 0;
        update_status();
    } else {
        set_status(RangeFinder::Status::NoData);
    }
}

void AP_RangeFinder_VL53L0X::timer(void)
{
    uint16_t range_mm;
    if (get_reading(range_mm) && range_mm < 8000) {
        sum_mm += range_mm;
        counter++;
    }
}

#endif  // AP_RANGEFINDER_VL53L0X_ENABLED
