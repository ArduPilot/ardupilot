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
const AnalogIn::pin_info AnalogIn::pin_config[] = HAL_ANALOG_PINS;

#define ADC_GRP1_NUM_CHANNELS   ARRAY_SIZE(AnalogIn::pin_config)

#if defined(ADC_CFGR_RES_16BITS)
// on H7 we use 16 bit ADC transfers, giving us more resolution. We
// need to scale by 1/16 to match the 12 bit scale factors in hwdef.dat
#define ADC_BOARD_SCALING (1.0/16)
#else
#define ADC_BOARD_SCALING 1
#endif

// samples filled in by ADC DMA engine
adcsample_t *AnalogIn::samples;
uint32_t AnalogIn::sample_sum[ADC_GRP1_NUM_CHANNELS];
uint32_t AnalogIn::sample_count;

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

/*
  return scaling from ADC count to Volts
 */
float AnalogSource::_pin_scaler(void)
{
    float scaling = VOLTAGE_SCALING;
    for (uint8_t i=0; i<ADC_GRP1_NUM_CHANNELS; i++) {
        if (AnalogIn::pin_config[i].channel == _pin) {
            scaling = AnalogIn::pin_config[i].scaling;
            break;
        }
    }
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
    if (_pin == pin) {
        return true;
    }
    bool found_pin = false;
    if (pin == ANALOG_SERVO_VRSSI_PIN) {
        found_pin = true;
    } else {
        for (uint8_t i=0; i<ADC_GRP1_NUM_CHANNELS; i++) {
            if (AnalogIn::pin_config[i].channel == pin) {
                found_pin = true;
                break;
            }
        }
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
  callback from ADC driver when sample buffer is filled
 */
void AnalogIn::adccallback(ADCDriver *adcp)
{
    const adcsample_t *buffer = samples;

    stm32_cacheBufferInvalidate(buffer, sizeof(adcsample_t)*ADC_DMA_BUF_DEPTH*ADC_GRP1_NUM_CHANNELS);
    for (uint8_t i = 0; i < ADC_DMA_BUF_DEPTH; i++) {
        for (uint8_t j = 0; j < ADC_GRP1_NUM_CHANNELS; j++) {
            sample_sum[j] += *buffer++;
        }
    }
    sample_count += ADC_DMA_BUF_DEPTH;
}

/*
  setup adc peripheral to capture samples with DMA into a buffer
 */
void AnalogIn::init()
{
    static_assert(sizeof(uint16_t) == sizeof(adcsample_t), "adcsample_t must be uint16_t");

    if (ADC_GRP1_NUM_CHANNELS == 0) {
        return;
    }

    samples = (adcsample_t *)hal.util->malloc_type(sizeof(adcsample_t)*ADC_DMA_BUF_DEPTH*ADC_GRP1_NUM_CHANNELS, AP_HAL::Util::MEM_DMA_SAFE);

    adcStart(&ADCD1, NULL);
    memset(&adcgrpcfg, 0, sizeof(adcgrpcfg));
    adcgrpcfg.circular = true;
    adcgrpcfg.num_channels = ADC_GRP1_NUM_CHANNELS;
    adcgrpcfg.end_cb = adccallback;
#if defined(ADC_CFGR_RES_16BITS)
    // use 16 bit resolution
    adcgrpcfg.cfgr = ADC_CFGR_CONT | ADC_CFGR_RES_16BITS;
#elif defined(ADC_CFGR_RES_12BITS)
    // use 12 bit resolution
    adcgrpcfg.cfgr = ADC_CFGR_CONT | ADC_CFGR_RES_12BITS;
#else
    // use 12 bit resolution with ADCv1 or ADCv2
    adcgrpcfg.sqr1 = ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS);
    adcgrpcfg.cr2 = ADC_CR2_SWSTART;
#endif

    for (uint8_t i=0; i<ADC_GRP1_NUM_CHANNELS; i++) {
        uint8_t chan = pin_config[i].channel;
        // setup cycles per sample for the channel
#if defined(STM32H7)
        adcgrpcfg.pcsel |= (1<<chan);
        adcgrpcfg.smpr[chan/10] |= ADC_SMPR_SMP_384P5 << (3*(chan%10));
        if (i < 4) {
            adcgrpcfg.sqr[0] |= chan << (6*(i+1));
        } else if (i < 9) {
            adcgrpcfg.sqr[1] |= chan << (6*(i-4));
        } else {
            adcgrpcfg.sqr[2] |= chan << (6*(i-9));
        }
#elif defined(STM32F3) || defined(STM32G4) || defined(STM32L4)
#if defined(STM32G4) || defined(STM32L4)
        adcgrpcfg.smpr[chan/10] |= ADC_SMPR_SMP_640P5 << (3*(chan%10));
#else
        adcgrpcfg.smpr[chan/10] |= ADC_SMPR_SMP_601P5 << (3*(chan%10));
#endif
        // setup channel sequence
        if (i < 4) {
            adcgrpcfg.sqr[0] |= chan << (6*(i+1));
        } else if (i < 9) {
            adcgrpcfg.sqr[1] |= chan << (6*(i-4));
        } else {
            adcgrpcfg.sqr[2] |= chan << (6*(i-9));
        }
#else
        if (chan < 10) {
            adcgrpcfg.smpr2 |= ADC_SAMPLE_480 << (3*chan);
        } else {
            adcgrpcfg.smpr1 |= ADC_SAMPLE_480 << (3*(chan-10));
        }
        // setup channel sequence
        if (i < 6) {
            adcgrpcfg.sqr3 |= chan << (5*i);
        } else if (i < 12) {
            adcgrpcfg.sqr2 |= chan << (5*(i-6));
        } else {
            adcgrpcfg.sqr1 |= chan << (5*(i-12));
        }
#endif
    }
    adcStartConversion(&ADCD1, &adcgrpcfg, samples, ADC_DMA_BUF_DEPTH);

#if HAL_WITH_MCU_MONITORING
    setup_adc3();
#endif
}

/*
  calculate average sample since last read for all channels
 */
void AnalogIn::read_adc(uint32_t *val)
{
    chSysLock();
    for (uint8_t i = 0; i < ADC_GRP1_NUM_CHANNELS; i++) {
        val[i] = sample_sum[i] / sample_count;
    }
    memset(sample_sum, 0, sizeof(sample_sum));
    sample_count = 0;
    chSysUnlock();
}


#if HAL_WITH_MCU_MONITORING
/*
  on H7 we can support monitoring MCU temperature and voltage using ADC3
 */
#define ADC3_GRP1_NUM_CHANNELS 3

// internal ADC channels (from H7 reference manual)
#define ADC3_VSENSE_CHAN 18
#define ADC3_VREFINT_CHAN 19
#define ADC3_VBAT4_CHAN 17

// samples filled in by ADC DMA engine
adcsample_t *AnalogIn::samples_adc3;
uint32_t AnalogIn::sample_adc3_sum[ADC3_GRP1_NUM_CHANNELS];
// we also keep min and max so we can report the range of voltages
// seen, to give an idea of supply stability
uint16_t AnalogIn::sample_adc3_max[ADC3_GRP1_NUM_CHANNELS];
uint16_t AnalogIn::sample_adc3_min[ADC3_GRP1_NUM_CHANNELS];
uint32_t AnalogIn::sample_adc3_count;

/*
  callback from ADC3 driver when sample buffer is filled
 */
void AnalogIn::adc3callback(ADCDriver *adcp)
{
    const adcsample_t *buffer = samples_adc3;

    stm32_cacheBufferInvalidate(buffer, sizeof(adcsample_t)*ADC_DMA_BUF_DEPTH*ADC3_GRP1_NUM_CHANNELS);
    for (uint8_t i = 0; i < ADC_DMA_BUF_DEPTH; i++) {
        for (uint8_t j = 0; j < ADC3_GRP1_NUM_CHANNELS; j++) {
            const uint16_t v = *buffer++;
            sample_adc3_sum[j] += v;
            if (sample_adc3_min[j] == 0 ||
                sample_adc3_min[j] > v) {
                sample_adc3_min[j] = v;
            }
            if (sample_adc3_max[j] == 0 ||
                sample_adc3_max[j] < v) {
                sample_adc3_max[j] = v;
            }
        }
    }
    sample_adc3_count += ADC_DMA_BUF_DEPTH;
}

/*
  setup ADC3 for internal temperature and voltage monitoring
 */
void AnalogIn::setup_adc3(void)
{
    samples_adc3 = (adcsample_t *)hal.util->malloc_type(sizeof(adcsample_t)*ADC_DMA_BUF_DEPTH*ADC3_GRP1_NUM_CHANNELS, AP_HAL::Util::MEM_DMA_SAFE);
    if (samples_adc3 == nullptr) {
        // not likely, but can't setup ADC3
        return;
    }

    adcStart(&ADCD3, NULL);

    adcSTM32EnableVREF(&ADCD3);
    adcSTM32EnableTS(&ADCD3);
    adcSTM32EnableVBAT(&ADCD3);

    memset(&adc3grpcfg, 0, sizeof(adc3grpcfg));
    adc3grpcfg.circular = true;
    adc3grpcfg.num_channels = ADC3_GRP1_NUM_CHANNELS;
    adc3grpcfg.end_cb = adc3callback;
#if defined(ADC_CFGR_RES_16BITS)
    // use 16 bit resolution
    adc3grpcfg.cfgr = ADC_CFGR_CONT | ADC_CFGR_RES_16BITS;
#elif defined(ADC_CFGR_RES_12BITS)
    // use 12 bit resolution
    adc3grpcfg.cfgr = ADC_CFGR_CONT | ADC_CFGR_RES_12BITS;
#else
    // use 12 bit resolution with ADCv1 or ADCv2
    adc3grpcfg.sqr1 = ADC_SQR1_NUM_CH(ADC3_GRP1_NUM_CHANNELS);
    adc3grpcfg.cr2 = ADC_CR2_SWSTART;
#endif

    const uint8_t channels[ADC3_GRP1_NUM_CHANNELS] = { ADC3_VBAT4_CHAN, ADC3_VSENSE_CHAN, ADC3_VREFINT_CHAN };

    for (uint8_t i=0; i<ADC3_GRP1_NUM_CHANNELS; i++) {
        uint8_t chan = channels[i];
        // setup cycles per sample for the channel
        adc3grpcfg.pcsel |= (1<<chan);
        adc3grpcfg.smpr[chan/10] |= ADC_SMPR_SMP_384P5 << (3*(chan%10));
        if (i < 4) {
            adc3grpcfg.sqr[0] |= chan << (6*(i+1));
        } else if (i < 9) {
            adc3grpcfg.sqr[1] |= chan << (6*(i-4));
        } else {
            adc3grpcfg.sqr[2] |= chan << (6*(i-9));
        }
    }
    adcStartConversion(&ADCD3, &adc3grpcfg, samples_adc3, ADC_DMA_BUF_DEPTH);
}

/*
  calculate average sample since last read for all channels
 */
void AnalogIn::read_adc3(uint32_t *val, uint16_t *min, uint16_t *max)
{
    chSysLock();
    for (uint8_t i = 0; i < ADC3_GRP1_NUM_CHANNELS; i++) {
        val[i] = sample_adc3_sum[i] / sample_adc3_count;
        min[i] = sample_adc3_min[i];
        max[i] = sample_adc3_max[i];
    }
    memset(sample_adc3_sum, 0, sizeof(sample_adc3_sum));
    memset(sample_adc3_min, 0, sizeof(sample_adc3_min));
    memset(sample_adc3_max, 0, sizeof(sample_adc3_max));
    sample_adc3_count = 0;
    chSysUnlock();
}

#endif // HAL_WITH_MCU_MONITORING

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

    uint32_t buf_adc[ADC_GRP1_NUM_CHANNELS];

    /* read all channels available */
    read_adc(buf_adc);

    // update power status flags
    update_power_flags();

    // match the incoming channels to the currently active pins
    for (uint8_t i=0; i < ADC_GRP1_NUM_CHANNELS; i++) {
#ifdef ANALOG_VCC_5V_PIN
        if (pin_config[i].channel == ANALOG_VCC_5V_PIN) {
            // record the Vcc value for later use in
            // voltage_average_ratiometric()
            _board_voltage = buf_adc[i] * pin_config[i].scaling * ADC_BOARD_SCALING;
        }
#endif
#ifdef FMU_SERVORAIL_ADC_CHAN
        if (pin_config[i].channel == FMU_SERVORAIL_ADC_CHAN) {
           _servorail_voltage = buf_adc[i] * pin_config[i].scaling * ADC_BOARD_SCALING;
        }
#endif
    }

#if HAL_WITH_IO_MCU
    // now handle special inputs from IOMCU
    _servorail_voltage = iomcu.get_vservo_adc_count() * (VOLTAGE_SCALING * HAL_IOMCU_VSERVO_SCALAR);
    _rssi_voltage = iomcu.get_vrssi_adc_count() * (VOLTAGE_SCALING *  HAL_IOMCU_VRSSI_SCALAR);
#endif

    for (uint8_t i=0; i<ADC_GRP1_NUM_CHANNELS; i++) {
        Debug("chan %u value=%u\n",
              (unsigned)pin_config[i].channel,
              (unsigned)buf_adc[i]);
        for (uint8_t j=0; j < ANALOG_MAX_CHANNELS; j++) {
            ChibiOS::AnalogSource *c = _channels[j];
            if (c != nullptr) {
                if (pin_config[i].channel == c->_pin) {
                    // add a value
                    c->_add_value(buf_adc[i] * ADC_BOARD_SCALING, _board_voltage);
                } else if (c->_pin == ANALOG_SERVO_VRSSI_PIN) {
                    c->_add_value(_rssi_voltage / VOLTAGE_SCALING, 0);
                }
            }
        }
    }

#if HAL_WITH_MCU_MONITORING
    // 20Hz temperature and ref voltage
    static uint32_t last_mcu_temp_us;
    if (now - last_mcu_temp_us > 50000 &&
        hal.scheduler->is_system_initialized()) {
        last_mcu_temp_us = now;

        uint32_t buf_adc3[ADC3_GRP1_NUM_CHANNELS];
        uint16_t min_adc3[ADC3_GRP1_NUM_CHANNELS];
        uint16_t max_adc3[ADC3_GRP1_NUM_CHANNELS];

        read_adc3(buf_adc3, min_adc3, max_adc3);

        // factory calibration values
        const float TS_CAL1 = *(const volatile uint16_t *)0x1FF1E820;
        const float TS_CAL2 = *(const volatile uint16_t *)0x1FF1E840;
        const float VREFINT_CAL = *(const volatile uint16_t *)0x1FF1E860;

        _mcu_temperature = ((110 - 30) / (TS_CAL2 - TS_CAL1)) * (float(buf_adc3[1]) - TS_CAL1) + 30;
        _mcu_voltage = 3.3 * VREFINT_CAL / float(buf_adc3[2]+0.001);
        // note min/max swap due to inversion
        _mcu_voltage_min = 3.3 * VREFINT_CAL / float(max_adc3[2]+0.001);
        _mcu_voltage_max = 3.3 * VREFINT_CAL / float(min_adc3[2]+0.001);
    }
#endif
}

AP_HAL::AnalogSource* AnalogIn::channel(int16_t pin)
{
    WITH_SEMAPHORE(_semaphore);
    for (uint8_t j=0; j<ANALOG_MAX_CHANNELS; j++) {
        if (_channels[j] == nullptr) {
            _channels[j] = new AnalogSource(pin);
            return _channels[j];
        }
    }
    hal.console->printf("Out of analog channels\n");
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
    if (palReadLine(HAL_GPIO_PIN_VDD_BRICK_VALID) == 1) {
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
        _power_flags != flags &&
        hal.util->get_soft_armed()) {
        // the power status has changed while armed
        flags |= MAV_POWER_STATUS_CHANGED;
    }
    _accumulated_power_flags |= flags;
    _power_flags = flags;
}
#endif // HAL_USE_ADC
