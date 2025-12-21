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
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */
#include <AP_HAL/AP_HAL.h>
#include "ch.h"
#include "hal.h"
#include <AP_Math/AP_Math.h>

#if HAL_USE_ADC == TRUE && !defined(HAL_DISABLE_ADC_DRIVER)

#include "AnalogIn.h"

#if HAL_WITH_IO_MCU
#include <AP_IOMCU/AP_IOMCU.h>
extern AP_IOMCU iomcu;
#endif

#include "hwdef/common/stm32_util.h"

// MAVLink is included as we send a mavlink message as part of debug,
// and also use the MAV_POWER flags below in update_power_flags
#include <GCS_MAVLink/GCS_MAVLink.h>

#ifndef STM32_ADC_DUAL_MODE
#define STM32_ADC_DUAL_MODE                 FALSE
#endif

#define ANLOGIN_DEBUGGING 0

// base voltage scaling for 12 bit 3.3V ADC
#define VOLTAGE_SCALING (3.3f / ((1 << 12) - 1))

// voltage divider is usually 1/(10/(20+10))
#ifndef HAL_IOMCU_VSERVO_SCALAR
  #define HAL_IOMCU_VSERVO_SCALAR 3
#endif

// voltage divider is usually not present
#ifndef HAL_IOMCU_VRSSI_SCALAR
  #define HAL_IOMCU_VRSSI_SCALAR 1
#endif

#if ANLOGIN_DEBUGGING
 # define Debug(fmt, args ...)  do {printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
 # define Debug(fmt, args ...)
#endif

extern const AP_HAL::HAL& hal;

using namespace ChibiOS;

// special pins
#define ANALOG_SERVO_VRSSI_PIN 103

/*
  scaling table between ADC count and actual input voltage, to account
  for voltage dividers on the board.
 */
const AnalogIn::pin_info AnalogIn::pin_config[] = { HAL_ANALOG_PINS };

#ifdef HAL_ANALOG2_PINS
    const AnalogIn::pin_info AnalogIn::pin_config_2[] = { HAL_ANALOG2_PINS };
    #define ADC2_GRP1_NUM_CHANNELS ARRAY_SIZE(AnalogIn::pin_config_2)
#endif

#if defined(HAL_ANALOG3_PINS) || HAL_WITH_MCU_MONITORING
#if HAL_WITH_MCU_MONITORING
    // internal ADC channels (from H7 reference manual)
    #define ADC3_VSENSE_CHAN 18
    #define ADC3_VREFINT_CHAN 19
    #define ADC3_VBAT4_CHAN 17
    #define HAL_MCU_MONITORING_PINS {ADC3_VBAT4_CHAN, 252, 3.30/4096}, {ADC3_VSENSE_CHAN, 253, 3.30/4096}, {ADC3_VREFINT_CHAN, 254, 3.30/4096}
#else
    #define HAL_MCU_MONITORING_PINS
#endif
#ifndef HAL_ANALOG3_PINS
    #define HAL_ANALOG3_PINS
#endif
    const AnalogIn::pin_info AnalogIn::pin_config_3[] = { HAL_ANALOG3_PINS HAL_MCU_MONITORING_PINS};
    #define ADC3_GRP1_NUM_CHANNELS ARRAY_SIZE(AnalogIn::pin_config_3)
#endif

#define ADC_GRP1_NUM_CHANNELS   ARRAY_SIZE(AnalogIn::pin_config)

#if defined(ADC_CFGR_RES_16BITS)
// on H7 we use 16 bit ADC transfers, giving us more resolution. We
// need to scale by 1/16 to match the 12 bit scale factors in hwdef.dat
#define ADC_BOARD_SCALING (1.0/16)
#else
#define ADC_BOARD_SCALING 1
#endif

// samples filled in by ADC DMA engine
adcsample_t *AnalogIn::samples[];
uint32_t *AnalogIn::sample_sum[];
uint32_t AnalogIn::sample_count[];

AnalogSource::AnalogSource(int16_t pin) :
    _pin(pin)
{
}


float AnalogSource::read_average()
{
    WITH_SEMAPHORE(_semaphore);

    if (_sum_count == 0) {
        return _value;
    }
    _value = _sum_value / _sum_count;
    _value_ratiometric = _sum_ratiometric / _sum_count;
    _sum_value = 0;
    _sum_ratiometric = 0;
    _sum_count = 0;

    return _value;
}

float AnalogSource::read_latest()
{
    return _latest_value;
}

bool AnalogIn::valid_analog_pin(uint16_t pin) const
{
    if (pin == ANALOG_INPUT_NONE) {
        return false;
    }
    for (uint8_t i=0; i<ADC_GRP1_NUM_CHANNELS; i++) {
        if (AnalogIn::pin_config[i].analog_pin == pin) {
            return true;
        }
    }
#ifdef HAL_ANALOG2_PINS
    for (uint8_t i=0; i<ADC2_GRP1_NUM_CHANNELS; i++) {
        if (AnalogIn::pin_config_2[i].analog_pin == pin) {
            return true;
        }
    }
#endif
#ifdef HAL_ANALOG3_PINS
    for (uint8_t i=0; i<ADC3_GRP1_NUM_CHANNELS; i++) {
        if (AnalogIn::pin_config_3[i].analog_pin == pin) {
            return true;
        }
    }
#endif

    return false;
}
/*
  return scaling from ADC count to Volts
 */
float AnalogSource::_pin_scaler(void)
{
    float scaling = VOLTAGE_SCALING;
    for (uint8_t i=0; i<ADC_GRP1_NUM_CHANNELS; i++) {
        if (AnalogIn::pin_config[i].analog_pin == _pin && (_pin != ANALOG_INPUT_NONE)) {
            scaling = AnalogIn::pin_config[i].scaling;
            break;
        }
    }
#ifdef HAL_ANALOG2_PINS
    for (uint8_t i=0; i<ADC2_GRP1_NUM_CHANNELS; i++) {
        if (AnalogIn::pin_config_2[i].analog_pin == _pin && (_pin != ANALOG_INPUT_NONE)) {
            scaling = AnalogIn::pin_config_2[i].scaling;
            break;
        }
    }
#endif
#ifdef HAL_ANALOG3_PINS
    for (uint8_t i=0; i<ADC3_GRP1_NUM_CHANNELS; i++) {
        if (AnalogIn::pin_config_3[i].analog_pin == _pin && (_pin != ANALOG_INPUT_NONE)) {
            scaling = AnalogIn::pin_config_3[i].scaling;
            break;
        }
    }
#endif
    return scaling;
}

/*
  return voltage in Volts
 */
float AnalogSource::voltage_average()
{
    return _pin_scaler() * read_average();
}

/*
  return voltage in Volts, assuming a ratiometric sensor powered by
  the 5V rail
 */
float AnalogSource::voltage_average_ratiometric()
{
    voltage_average();
    return _pin_scaler() * _value_ratiometric;
}

/*
  return voltage in Volts
 */
float AnalogSource::voltage_latest()
{
    return _pin_scaler() * read_latest();
}

bool AnalogSource::set_pin(uint8_t pin)
{
    if (pin == ANALOG_INPUT_NONE) {
        return false;
    }
    if (_pin == pin) {
        return true;
    }
    bool found_pin = false;
    if (pin == ANALOG_SERVO_VRSSI_PIN) {
        found_pin = true;
    } else {
        for (uint8_t i=0; i<ADC_GRP1_NUM_CHANNELS; i++) {
            if (AnalogIn::pin_config[i].analog_pin == pin) {
                found_pin = true;
                break;
            }
        }
#ifdef HAL_ANALOG2_PINS
        for (uint8_t i=0; i<ADC2_GRP1_NUM_CHANNELS; i++) {
            if (AnalogIn::pin_config_2[i].analog_pin == pin) {
                found_pin = true;
                break;
            }
        }
#endif
#ifdef HAL_ANALOG3_PINS
        for (uint8_t i=0; i<ADC3_GRP1_NUM_CHANNELS; i++) {
            if (AnalogIn::pin_config_3[i].analog_pin == pin) {
                found_pin = true;
                break;
            }
        }
#endif
    }
    if (!found_pin) {
        return false;
    }

    WITH_SEMAPHORE(_semaphore);
    _pin = pin;
    _sum_value = 0;
    _sum_ratiometric = 0;
    _sum_count = 0;
    _latest_value = 0;
    _value = 0;
    _value_ratiometric = 0;
    return true;
}

/*
  apply a reading in ADC counts
 */
void AnalogSource::_add_value(float v, float vcc5V)
{
    WITH_SEMAPHORE(_semaphore);

    _latest_value = v;
    _sum_value += v;
    if (vcc5V < 3.0f) {
        _sum_ratiometric += v;
    } else {
        // this compensates for changes in the 5V rail relative to the
        // 3.3V reference used by the ADC.
        _sum_ratiometric += v * 5.0f / vcc5V;
    }
    _sum_count++;
    if (_sum_count == 254) {
        _sum_value /= 2;
        _sum_ratiometric /= 2;
        _sum_count /= 2;
    }
}


/*
  return the channel number
 */
uint8_t AnalogIn::get_pin_channel(uint8_t adc_index, uint8_t pin_index)
{
    switch(adc_index) {
    case 0:
        osalDbgAssert(pin_index < ADC_GRP1_NUM_CHANNELS, "invalid pin_index");
        return pin_config[pin_index].channel;
#if defined(HAL_ANALOG2_PINS)
    case 1:
        osalDbgAssert(pin_index < ADC2_GRP1_NUM_CHANNELS, "invalid pin_index");
        return pin_config_2[pin_index].channel;
#endif
#if defined(HAL_ANALOG3_PINS)
    case 2:
        osalDbgAssert(pin_index < ADC3_GRP1_NUM_CHANNELS, "invalid pin_index");
        return pin_config_3[pin_index].channel;
#endif
    };
    osalDbgAssert(false, "invalid adc_index");
    return 255;
}

uint8_t AnalogIn::get_analog_pin(uint8_t adc_index, uint8_t pin_index)
{
    switch(adc_index) {
    case 0:
        return pin_config[pin_index].analog_pin;
#if defined(HAL_ANALOG2_PINS)
    case 1:
        return pin_config_2[pin_index].analog_pin;
#endif
#if defined(HAL_ANALOG3_PINS)
    case 2:
        return pin_config_3[pin_index].analog_pin;
#endif
    };
    osalDbgAssert(false, "invalid adc_index");
    return 255;
}
/*
  return scaling
 */
float AnalogIn::get_pin_scaling(uint8_t adc_index, uint8_t pin_index)
{
    switch(adc_index) {
    case 0:
        return pin_config[pin_index].scaling;
#if defined(HAL_ANALOG2_PINS)
    case 1:
        return pin_config_2[pin_index].scaling;
#endif
#if defined(HAL_ANALOG3_PINS)
    case 2:
        return pin_config_3[pin_index].scaling;
#endif
    };
    osalDbgAssert(false, "invalid adc_index");
    return 0;
}

/*
  callback from ADC driver when sample buffer is filled
 */
#if HAL_WITH_MCU_MONITORING
static uint16_t min_vrefint, max_vrefint;
#endif
void AnalogIn::adccallback(ADCDriver *adcp)
{
    uint8_t index = get_adc_index(adcp);
    if (index >= HAL_NUM_ANALOG_INPUTS) {
        return;
    }
    const uint16_t *buffer = (uint16_t*)samples[index];
    uint8_t num_grp_channels = get_num_grp_channels(index);
#if STM32_ADC_DUAL_MODE
    if (index == 0) {
        stm32_cacheBufferInvalidate(buffer, sizeof(uint16_t)*ADC_DMA_BUF_DEPTH*num_grp_channels*2);
    } else
#endif
    {
        stm32_cacheBufferInvalidate(buffer, sizeof(uint16_t)*ADC_DMA_BUF_DEPTH*num_grp_channels);
    }
    for (uint8_t i = 0; i < ADC_DMA_BUF_DEPTH; i++) {
        for (uint8_t j = 0; j < num_grp_channels; j++) {
            sample_sum[index][j] += *buffer;

#if HAL_WITH_MCU_MONITORING
            if (j == (num_grp_channels-1) && index == 2) {
                // record min/max for MCU Vcc
                if (min_vrefint == 0 ||
                    min_vrefint > *buffer) {
                    min_vrefint = *buffer;
                }
                if (max_vrefint == 0 ||
                    max_vrefint < *buffer) {
                    max_vrefint = *buffer;
                }
            }
#endif
            buffer++;
#if STM32_ADC_DUAL_MODE
            if (index == 0) {
                // also sum the second ADC for dual mode
                sample_sum[1][j] += *buffer;
                buffer++;
            } 
#endif
        }
    }
#if STM32_ADC_DUAL_MODE
    if (index == 0) {
        // also set the sample count for the second ADC for dual mode
        sample_count[1] += ADC_DMA_BUF_DEPTH;
    }
#endif
    sample_count[index] += ADC_DMA_BUF_DEPTH;
}

uint8_t AnalogIn::get_adc_index(ADCDriver* adcp)
{
    if (adcp == &ADCD1) {
        return 0;
    }
#if defined(HAL_ANALOG2_PINS) && !STM32_ADC_DUAL_MODE
    if (adcp == &ADCD2) {
        return 1;
    }
#endif
#if defined(HAL_ANALOG3_PINS)
    if (adcp == &ADCD3) {
        return 2;
    }
#endif
    osalDbgAssert(false, "invalid ADC");
    return 255;
}

uint8_t AnalogIn::get_num_grp_channels(uint8_t index)
{
    switch (index) {
    case 0:
        return ADC_GRP1_NUM_CHANNELS;
#if defined(HAL_ANALOG2_PINS)
    case 1:
        return ADC2_GRP1_NUM_CHANNELS;
#endif
#if defined(HAL_ANALOG3_PINS)
    case 2:
        return ADC3_GRP1_NUM_CHANNELS;
#endif
    };
    osalDbgAssert(false, "invalid adc_index");
    return 0;
}

/*
  setup adc peripheral to capture samples with DMA into a buffer
 */
void AnalogIn::init()
{
#if STM32_ADC_DUAL_MODE
    static_assert(sizeof(uint32_t) == sizeof(adcsample_t), "adcsample_t must be uint32_t");
#else
    static_assert(sizeof(uint16_t) == sizeof(adcsample_t), "adcsample_t must be uint16_t");
#endif
    setup_adc(0);
#if defined(HAL_ANALOG2_PINS)
    setup_adc(1);
#endif
#if defined(HAL_ANALOG3_PINS)
    setup_adc(2);
#endif
}

void AnalogIn::setup_adc(uint8_t index)
{
    uint8_t num_grp_channels = get_num_grp_channels(index);
    if (num_grp_channels == 0) {
        return;
    }

    ADCDriver *adcp;
    switch (index) {
    case 0:
        adcp = &ADCD1;
        break;
// if we are in dual mode then ADC2 is setup along with ADC1
// so we don't need to setup ADC2 separately
#if defined(HAL_ANALOG2_PINS) && !STM32_ADC_DUAL_MODE
    case 1:
        adcp = &ADCD2;
        break;
#endif
#if defined(HAL_ANALOG3_PINS)
    case 2:
        adcp = &ADCD3;
        break;
#endif
    default:
        return;
    };

#if STM32_ADC_DUAL_MODE
    // In ADC Dual mode we use ADC_SAMPLE_SIZE as 32 bits for ADCs doing shared sampling, then we need to
    // split the samples into two buffers at read time for other ADCs ChibiOS HAL uses 16 bit sample size
    // so ADC_SAMPLE_SIZE is actually 16 bits for ADCs even though its set to 32 bits
    if (index == 0) {
        // we need to setup the second ADC as well
        num_grp_channels = num_grp_channels * 2;
        samples[0] = (adcsample_t *)hal.util->malloc_type(sizeof(uint16_t)*ADC_DMA_BUF_DEPTH*num_grp_channels, AP_HAL::Util::MEM_DMA_SAFE);
        if (samples[0] == nullptr) {
            // panic if we can't allocate memory
            goto failed_alloc;
        }
        sample_sum[0] = (uint32_t *)malloc(sizeof(uint32_t)*num_grp_channels);
        if (sample_sum[0] == nullptr) {
            // panic if we can't allocate memory
            goto failed_alloc;
        }
        sample_sum[1] = (uint32_t *)malloc(sizeof(uint32_t)*num_grp_channels);  
        if (sample_sum[1] == nullptr) {
            // panic if we can't allocate memory
            goto failed_alloc;
        }      
    } else {
        samples[index] = (adcsample_t *)hal.util->malloc_type(sizeof(uint16_t)*ADC_DMA_BUF_DEPTH*num_grp_channels, AP_HAL::Util::MEM_DMA_SAFE);
        if (samples[index] == nullptr) {
            // panic if we can't allocate memory
            goto failed_alloc;
        }
        sample_sum[index] = (uint32_t *)malloc(sizeof(uint32_t)*num_grp_channels);
        if (sample_sum[index] == nullptr) {
            // panic if we can't allocate memory
            goto failed_alloc;
        }
    }
#else
    samples[index] = (adcsample_t *)hal.util->malloc_type(sizeof(adcsample_t)*ADC_DMA_BUF_DEPTH*num_grp_channels, AP_HAL::Util::MEM_DMA_SAFE);
    if (samples[index] == nullptr) {
        // panic if we can't allocate memory
            goto failed_alloc;
    }
    sample_sum[index] = (uint32_t *)malloc(sizeof(uint32_t)*num_grp_channels);
    if (sample_sum[index] == nullptr) {
        // panic if we can't allocate memory
            goto failed_alloc;
    }
#endif

    adcStart(adcp, NULL);
#if HAL_WITH_MCU_MONITORING
    if (index == 2) {
        adcSTM32EnableVREF(&ADCD3);
        adcSTM32EnableTS(&ADCD3);
        adcSTM32EnableVBAT(&ADCD3);
    }
#endif
    memset(&adcgrpcfg[index], 0, sizeof(adcgrpcfg[index]));
    adcgrpcfg[index].circular = true;
    adcgrpcfg[index].num_channels = num_grp_channels;
    adcgrpcfg[index].end_cb = adccallback;
#if defined(ADC_CFGR_RES_16BITS)
    // use 16 bit resolution
    adcgrpcfg[index].cfgr = ADC_CFGR_CONT | ADC_CFGR_RES_16BITS;
#elif defined(ADC_CFGR_RES_12BITS)
    // use 12 bit resolution
    adcgrpcfg[index].cfgr = ADC_CFGR_CONT | ADC_CFGR_RES_12BITS;
#else
    // use 12 bit resolution with ADCv1 or ADCv2
    adcgrpcfg[index].sqr1 = ADC_SQR1_NUM_CH(num_grp_channels);
    adcgrpcfg[index].cr2 = ADC_CR2_SWSTART;
#endif

#if STM32_ADC_DUAL_MODE
    adcgrpcfg[index].ccr = 0;
    if (index == 0) {
        num_grp_channels /= 2;
    }
#endif
    for (uint8_t i=0; i<num_grp_channels; i++) {
        uint8_t chan = get_pin_channel(index, i);
        // setup cycles per sample for the channel
#if defined(STM32H7)
        adcgrpcfg[index].pcsel |= (1<<chan);
        adcgrpcfg[index].smpr[chan/10] |= ADC_SMPR_SMP_384P5 << (3*(chan%10));
        if (i < 4) {
            adcgrpcfg[index].sqr[0] |= chan << (6*(i+1));
        } else if (i < 9) {
            adcgrpcfg[index].sqr[1] |= chan << (6*(i-4));
        } else {
            adcgrpcfg[index].sqr[2] |= chan << (6*(i-9));
        }
#elif defined(STM32F3) || defined(STM32G4) || defined(STM32L4) || defined(STM32L4PLUS)
#if defined(STM32G4) || defined(STM32L4) || defined(STM32L4PLUS)
        adcgrpcfg[index].smpr[chan/10] |= ADC_SMPR_SMP_640P5 << (3*(chan%10));
#else
        adcgrpcfg[index].smpr[chan/10] |= ADC_SMPR_SMP_601P5 << (3*(chan%10));
#endif
        // setup channel sequence
        if (i < 4) {
            adcgrpcfg[index].sqr[0] |= chan << (6*(i+1));
        } else if (i < 9) {
            adcgrpcfg[index].sqr[1] |= chan << (6*(i-4));
        } else {
            adcgrpcfg[index].sqr[2] |= chan << (6*(i-9));
        }
#else
        if (chan < 10) {
            adcgrpcfg[index].smpr2 |= ADC_SAMPLE_480 << (3*chan);
        } else {
            adcgrpcfg[index].smpr1 |= ADC_SAMPLE_480 << (3*(chan-10));
        }
        // setup channel sequence
        if (i < 6) {
            adcgrpcfg[index].sqr3 |= chan << (5*i);
        } else if (i < 12) {
            adcgrpcfg[index].sqr2 |= chan << (5*(i-6));
        } else {
            adcgrpcfg[index].sqr1 |= chan << (5*(i-12));
        }
#endif
    }
#if STM32_ADC_DUAL_MODE
    // assert that ADC1 and ADC2 have the same number of channels
    static_assert(ARRAY_SIZE(AnalogIn::pin_config) == ARRAY_SIZE(AnalogIn::pin_config_2), "ADC1 and ADC2 must have same num of channels");

    if (index == 0) {
        for (uint8_t i=0; i<num_grp_channels; i++) {
            uint8_t chan = get_pin_channel(1, i);
            // setup cycles per sample for the channel
            adcgrpcfg[0].pcsel |= (1<<chan);
            adcgrpcfg[0].ssmpr[chan/10] |= ADC_SMPR_SMP_384P5 << (3*(chan%10));
            if (i < 4) {
                adcgrpcfg[0].ssqr[0] |= chan << (6*(i+1));
            } else if (i < 9) {
                adcgrpcfg[0].ssqr[1] |= chan << (6*(i-4));
            } else {
                adcgrpcfg[0].ssqr[2] |= chan << (6*(i-9));
            }
        }
    }
#endif

    adcStartConversion(adcp, &adcgrpcfg[index], samples[index], ADC_DMA_BUF_DEPTH);
    return;
failed_alloc:
    AP_HAL::panic("Failed to allocate ADC DMA buffer");
}

/*
  calculate average sample since last read for all channels
 */
void AnalogIn::read_adc(uint8_t index, uint32_t *val)
{
    chSysLock();
    uint8_t num_grp_channels = get_num_grp_channels(index);
    if (num_grp_channels == 0) {
        chSysUnlock();
        return;
    }
    for (uint8_t i = 0; i < num_grp_channels; i++) {
        val[i] = sample_sum[index][i] / sample_count[index];
    }
    memset(sample_sum[index], 0, sizeof(uint32_t) * num_grp_channels);
    sample_count[index] = 0;
#if HAL_WITH_MCU_MONITORING
    if (index == 2) {
        // copy the min/max values of vrefint if we are reading ADC3
        if (_mcu_vrefint_min == 0 ||
            _mcu_vrefint_min > min_vrefint) {
            _mcu_vrefint_min = min_vrefint;
        }
        if (_mcu_vrefint_max == 0 ||
            _mcu_vrefint_max < max_vrefint) {
            _mcu_vrefint_max = max_vrefint;
        }
        // also reset the min/max values
        min_vrefint = 0;
        max_vrefint = 0;
        // accumulate temperature and Vcc readings
        _mcu_monitor_temperature_accum += val[num_grp_channels - 2];
        _mcu_monitor_voltage_accum += val[num_grp_channels - 1];
        _mcu_monitor_sample_count++;
    }
#endif
    chSysUnlock();
}

/*
  read the data from an ADC index
 */
void AnalogIn::timer_tick_adc(uint8_t index)
{
    const uint8_t num_grp_channels = get_num_grp_channels(index);
    uint32_t buf_adc[num_grp_channels];

    /* read all channels available on index ADC*/
    read_adc(index, buf_adc);

    // match the incoming channels to the currently active pins
    for (uint8_t i=0; i < num_grp_channels; i++) {
#ifdef ANALOG_VCC_5V_PIN
        if (get_analog_pin(index, i) == ANALOG_VCC_5V_PIN) {
            // record the Vcc value for later use in
            // voltage_average_ratiometric()
            _board_voltage = buf_adc[i] * get_pin_scaling(index, i) * ADC_BOARD_SCALING;
        }
#endif
#ifdef FMU_SERVORAIL_ADC_PIN
        if (get_analog_pin(index, i) == FMU_SERVORAIL_ADC_PIN) {
            _servorail_voltage = buf_adc[i] * get_pin_scaling(index, i) * ADC_BOARD_SCALING;
        }
#endif
    }

    for (uint8_t i=0; i<num_grp_channels; i++) {
        Debug("adc%u chan %u value=%f\n",
              (unsigned)index+1,
              (unsigned)get_pin_channel(index, i),
              (float)buf_adc[i] * ADC_BOARD_SCALING * VOLTAGE_SCALING);
        for (uint8_t j=0; j < ANALOG_MAX_CHANNELS; j++) {
            ChibiOS::AnalogSource *c = _channels[j];
            if (c != nullptr) {
                if ((get_analog_pin(index, i) == c->_pin) && (c->_pin != ANALOG_INPUT_NONE)) {
                    // add a value
                    c->_add_value(buf_adc[i] * ADC_BOARD_SCALING, _board_voltage);
                } else if (c->_pin == ANALOG_SERVO_VRSSI_PIN) {
                    c->_add_value(_rssi_voltage / VOLTAGE_SCALING, 0);
                }
            }
        }
    }
}

/*
  called at 1kHz
 */
void AnalogIn::_timer_tick(void)
{
    // read adc at 100Hz
    uint32_t now = AP_HAL::micros();
    uint32_t delta_t = now - _last_run;
    if (delta_t < 10000) {
        return;
    }
    _last_run = now;

    // update power status flags
    update_power_flags();

#if HAL_WITH_IO_MCU
    // handle special inputs from IOMCU
    _rssi_voltage = iomcu.get_vrssi_adc_count() * (VOLTAGE_SCALING *  HAL_IOMCU_VRSSI_SCALAR);
#endif

    /*
      update each of our ADCs
     */
    timer_tick_adc(0);
#if defined(HAL_ANALOG2_PINS)
    timer_tick_adc(1);
#endif
#if defined(HAL_ANALOG3_PINS)
    timer_tick_adc(2);
#endif

#if HAL_WITH_IO_MCU
    _servorail_voltage = iomcu.get_vservo_adc_count() * (VOLTAGE_SCALING * HAL_IOMCU_VSERVO_SCALAR);
#endif

#if HAL_WITH_MCU_MONITORING
    // 20Hz temperature and ref voltage
    static uint32_t last_mcu_temp_us;
    if (now - last_mcu_temp_us > 50000 &&
        hal.scheduler->is_system_initialized()) {
        last_mcu_temp_us = now;

        // factory calibration values
        const float TS_CAL1 = *(const volatile uint16_t *)0x1FF1E820;
        const float TS_CAL2 = *(const volatile uint16_t *)0x1FF1E840;
        const float VREFINT_CAL = *(const volatile uint16_t *)0x1FF1E860;

        _mcu_temperature = ((110 - 30) / (TS_CAL2 - TS_CAL1)) * (float(_mcu_monitor_temperature_accum/_mcu_monitor_sample_count) - TS_CAL1) + 30;
        _mcu_voltage = 3.3 * VREFINT_CAL / float((_mcu_monitor_voltage_accum/_mcu_monitor_sample_count)+0.001);
        _mcu_monitor_voltage_accum = 0;
        _mcu_monitor_temperature_accum = 0;
        _mcu_monitor_sample_count = 0;

        // note min/max swap due to inversion
        _mcu_voltage_min = 3.3 * VREFINT_CAL / float(_mcu_vrefint_max+0.001);
        _mcu_voltage_max = 3.3 * VREFINT_CAL / float(_mcu_vrefint_min+0.001);
        
        // reset min and max
        _mcu_vrefint_max = 0;
        _mcu_vrefint_min = 0;
    }
#endif
}

AP_HAL::AnalogSource* AnalogIn::channel(int16_t pin)
{
    WITH_SEMAPHORE(_semaphore);
    for (uint8_t j=0; j<ANALOG_MAX_CHANNELS; j++) {
        if (_channels[j] == nullptr) {
            _channels[j] = NEW_NOTHROW AnalogSource(pin);
            return _channels[j];
        }
    }
    osalDbgAssert(false, "Out of analog channels");
    DEV_PRINTF("Out of analog channels\n");
    return nullptr;
}

/*
  update power status flags
 */
void AnalogIn::update_power_flags(void)
{
    uint16_t flags = 0;

    /*
      primary "brick" power supply valid pin. Some boards have this
      active high, some active low. Use nVALID for active low, VALID
      for active high
    */
#if defined(HAL_GPIO_PIN_VDD_BRICK_VALID)
    if (palReadLine(HAL_GPIO_PIN_VDD_BRICK_VALID) == 1) {
        flags |= MAV_POWER_STATUS_BRICK_VALID;
    }
#elif defined(HAL_GPIO_PIN_VDD_BRICK_nVALID)
    if (palReadLine(HAL_GPIO_PIN_VDD_BRICK_nVALID) == 0) {
        flags |= MAV_POWER_STATUS_BRICK_VALID;
    }
#endif

    /*
      secondary "brick" power supply valid pin. This is servo rail
      power valid on some boards. Some boards have this active high,
      some active low. Use nVALID for active low, VALID for active
      high. This maps to the MAV_POWER_STATUS_SERVO_VALID in mavlink
      (as this was first added for older boards that used servo rail
      for backup power)
    */
#if defined(HAL_GPIO_PIN_VDD_BRICK2_VALID)
    if (palReadLine(HAL_GPIO_PIN_VDD_BRICK2_VALID) == 1) {
        flags |= MAV_POWER_STATUS_SERVO_VALID;
    }
#elif defined(HAL_GPIO_PIN_VDD_BRICK2_nVALID)
    if (palReadLine(HAL_GPIO_PIN_VDD_BRICK2_nVALID) == 0) {
        flags |= MAV_POWER_STATUS_SERVO_VALID;
    }
#endif

    /*
      USB power. This can be VBUS_VALID, VBUS_nVALID or just
      VBUS. Some boards have both a valid pin and VBUS. The VBUS pin
      is an analog pin that could be used to read USB voltage.
     */
#if defined(HAL_GPIO_PIN_VBUS_VALID)
    if (palReadLine(HAL_GPIO_PIN_VBUS_VALID) == 1) {
        flags |= MAV_POWER_STATUS_USB_CONNECTED;
    }
#elif defined(HAL_GPIO_PIN_VBUS_nVALID)
    if (palReadLine(HAL_GPIO_PIN_VBUS_nVALID) == 0) {
        flags |= MAV_POWER_STATUS_USB_CONNECTED;
    }
#elif defined(HAL_GPIO_PIN_VBUS)
    if (palReadLine(HAL_GPIO_PIN_VBUS) == 1) {
        flags |= MAV_POWER_STATUS_USB_CONNECTED;
    }
#endif

    /*
      overcurrent on "high power" peripheral rail.
     */
#if defined(HAL_GPIO_PIN_VDD_5V_HIPOWER_OC)
    if (palReadLine(HAL_GPIO_PIN_VDD_5V_HIPOWER_OC) == 1) {
        flags |= MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT;
    }
#elif defined(HAL_GPIO_PIN_VDD_5V_HIPOWER_nOC)
    if (palReadLine(HAL_GPIO_PIN_VDD_5V_HIPOWER_nOC) == 0) {
        flags |= MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT;
    }
#endif

    /*
      overcurrent on main peripheral rail.
     */
#if defined(HAL_GPIO_PIN_VDD_5V_PERIPH_OC)
    if (palReadLine(HAL_GPIO_PIN_VDD_5V_PERIPH_OC) == 1) {
        flags |= MAV_POWER_STATUS_PERIPH_OVERCURRENT;
    }
#elif defined(HAL_GPIO_PIN_VDD_5V_PERIPH_nOC)
    if (palReadLine(HAL_GPIO_PIN_VDD_5V_PERIPH_nOC) == 0) {
        flags |= MAV_POWER_STATUS_PERIPH_OVERCURRENT;
    }
#endif

#if defined(HAL_GPIO_PIN_VDD_SERVO_VALID)
#error "building with old hwdef.dat"
#endif

#if 0
    /*
      this bit of debug code is useful when testing the polarity of
      VALID pins for power sources. It allows you to see the change on
      USB with a 3s delay, so you can see USB changes by unplugging
      and re-inserting USB power
     */
    static uint32_t last_change_ms;
    uint32_t now = AP_HAL::millis();
    if (_power_flags != flags) {
        if (last_change_ms == 0) {
            last_change_ms = now;
        } else if (now - last_change_ms > 3000) {
            last_change_ms = 0;
            hal.console->printf("POWR: 0x%02x -> 0x%02x\n", _power_flags, flags);
            _power_flags = flags;
        }
        if (hal.util->get_soft_armed()) {
            // the power status has changed while armed
            flags |= MAV_POWER_STATUS_CHANGED;
        }
        return;
    }
#endif

    if (_power_flags != 0 &&
        (_power_flags&~MAV_POWER_STATUS_CHANGED) != (flags&~MAV_POWER_STATUS_CHANGED) &&
        hal.util->get_soft_armed()) {
        // the power status has changed while armed
        flags |= MAV_POWER_STATUS_CHANGED;
    }
    _accumulated_power_flags |= flags;
    _power_flags = flags;
}
#endif // HAL_USE_ADC
