#include <AP_HAL.h>

#include "AP_ADC_ADS1115.h"

#define ADS1115_ADDRESS_ADDR_GND    0x48 // address pin low (GND)
#define ADS1115_ADDRESS_ADDR_VDD    0x49 // address pin high (VCC)
#define ADS1115_ADDRESS_ADDR_SDA    0x4A // address pin tied to SDA pin
#define ADS1115_ADDRESS_ADDR_SCL    0x4B // address pin tied to SCL pin
#define ADS1115_DEFAULT_ADDRESS     ADS1115_ADDRESS_ADDR_GND

#define ADS1115_RA_CONVERSION       0x00
#define ADS1115_RA_CONFIG           0x01
#define ADS1115_RA_LO_THRESH        0x02
#define ADS1115_RA_HI_THRESH        0x03

#define ADS1115_OS_SHIFT            15
#define ADS1115_OS_INACTIVE         0x00 << ADS1115_OS_SHIFT
#define ADS1115_OS_ACTIVE           0x01 << ADS1115_OS_SHIFT

#define ADS1115_CHANNELS_COUNT           8
#define ADS1115_MUX_SHIFT           12
#define ADS1115_MUX_P0_N1           0x00 << ADS1115_MUX_SHIFT /* default */
#define ADS1115_MUX_P0_N3           0x01 << ADS1115_MUX_SHIFT
#define ADS1115_MUX_P1_N3           0x02 << ADS1115_MUX_SHIFT
#define ADS1115_MUX_P2_N3           0x03 << ADS1115_MUX_SHIFT
#define ADS1115_MUX_P0_NG           0x04 << ADS1115_MUX_SHIFT
#define ADS1115_MUX_P1_NG           0x05 << ADS1115_MUX_SHIFT
#define ADS1115_MUX_P2_NG           0x06 << ADS1115_MUX_SHIFT
#define ADS1115_MUX_P3_NG           0x07 << ADS1115_MUX_SHIFT

#define ADS1115_PGA_SHIFT           9
#define ADS1115_PGA_6P144           0x00 << ADS1115_PGA_SHIFT
#define ADS1115_PGA_4P096           0x01 << ADS1115_PGA_SHIFT
#define ADS1115_PGA_2P048           0x02 << ADS1115_PGA_SHIFT // default
#define ADS1115_PGA_1P024           0x03 << ADS1115_PGA_SHIFT
#define ADS1115_PGA_0P512           0x04 << ADS1115_PGA_SHIFT
#define ADS1115_PGA_0P256           0x05 << ADS1115_PGA_SHIFT
#define ADS1115_PGA_0P256B          0x06 << ADS1115_PGA_SHIFT
#define ADS1115_PGA_0P256C          0x07 << ADS1115_PGA_SHIFT

#define ADS1115_MV_6P144            0.187500
#define ADS1115_MV_4P096            0.125000
#define ADS1115_MV_2P048            0.062500 // default
#define ADS1115_MV_1P024            0.031250
#define ADS1115_MV_0P512            0.015625
#define ADS1115_MV_0P256            0.007813
#define ADS1115_MV_0P256B           0.007813
#define ADS1115_MV_0P256C           0.007813

#define ADS1115_MODE_SHIFT          8
#define ADS1115_MODE_CONTINUOUS     0x00 << ADS1115_MODE_SHIFT
#define ADS1115_MODE_SINGLESHOT     0x01 << ADS1115_MODE_SHIFT // default

#define ADS1115_RATE_SHIFT          5
#define ADS1115_RATE_8              0x00 << ADS1115_RATE_SHIFT
#define ADS1115_RATE_16             0x01 << ADS1115_RATE_SHIFT
#define ADS1115_RATE_32             0x02 << ADS1115_RATE_SHIFT
#define ADS1115_RATE_64             0x03 << ADS1115_RATE_SHIFT
#define ADS1115_RATE_128            0x04 << ADS1115_RATE_SHIFT // default
#define ADS1115_RATE_250            0x05 << ADS1115_RATE_SHIFT
#define ADS1115_RATE_475            0x06 << ADS1115_RATE_SHIFT
#define ADS1115_RATE_860            0x07 << ADS1115_RATE_SHIFT

#define ADS1115_COMP_MODE_SHIFT         4
#define ADS1115_COMP_MODE_HYSTERESIS    0x00 << ADS1115_COMP_MODE_SHIFT        // default
#define ADS1115_COMP_MODE_WINDOW        0x01 << ADS1115_COMP_MODE_SHIFT

#define ADS1115_COMP_POL_SHIFT          3
#define ADS1115_COMP_POL_ACTIVE_LOW     0x00 << ADS1115_COMP_POL_SHIFT     // default
#define ADS1115_COMP_POL_ACTIVE_HIGH    0x01 << ADS1115_COMP_POL_SHIFT

#define ADS1115_COMP_LAT_SHIFT          2
#define ADS1115_COMP_LAT_NON_LATCHING   0x00 << ADS1115_COMP_LAT_SHIFT    // default
#define ADS1115_COMP_LAT_LATCHING       0x01 << ADS1115_COMP_LAT_SHIFT

#define ADS1115_COMP_QUE_SHIFT      0
#define ADS1115_COMP_QUE_ASSERT1    0x00 << ADS1115_COMP_SHIFT
#define ADS1115_COMP_QUE_ASSERT2    0x01 << ADS1115_COMP_SHIFT
#define ADS1115_COMP_QUE_ASSERT4    0x02 << ADS1115_COMP_SHIFT
#define ADS1115_COMP_QUE_DISABLE    0x03 // default

#define ADS1115_DEBUG 1
#if ADS1115_DEBUG
#include <cstdio>
#define debug(fmt, args ...)  do {hal.console->printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#define error(fmt, args ...)  do {fprintf(stderr,"%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)  
#define error(fmt, args ...)  
#endif

extern const AP_HAL::HAL& hal;
static const uint16_t mux_table[8] = 
{
    ADS1115_MUX_P0_N1,
    ADS1115_MUX_P0_N3,
    ADS1115_MUX_P1_N3,
    ADS1115_MUX_P2_N3,
    ADS1115_MUX_P0_NG,
    ADS1115_MUX_P1_NG,
    ADS1115_MUX_P2_NG,
    ADS1115_MUX_P3_NG
};


AP_ADC_ADS1115::AP_ADC_ADS1115():
    _i2c_sem(NULL),
    _channel_to_read(0)
{
}

void AP_ADC_ADS1115::Init()
{
    hal.scheduler->suspend_timer_procs();

    _gain = ADS1115_PGA_4P096; 
    _i2c_sem = hal.i2c->get_semaphore();

    hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&AP_ADC_ADS1115::_update));
    hal.scheduler->resume_timer_procs();
}

bool AP_ADC_ADS1115::_start_conversion(uint8_t channel)
{
    union {
        uint16_t w;
        uint8_t b[2];
    } config;

    config.w = ADS1115_OS_ACTIVE | _gain | mux_table[channel] | 
    ADS1115_MODE_SINGLESHOT | ADS1115_COMP_QUE_DISABLE | ADS1115_RATE_250;

    config.w = config.b[0] << 8 | config.b[1];

    if (hal.i2c->writeRegisters((uint8_t)ADS1115_ADDRESS_ADDR_GND, ADS1115_RA_CONFIG, 2, (uint8_t *) &config) != 0) {
        return false;
    }

    return true;
}

float AP_ADC_ADS1115::Ch(uint8_t ch_num)
{
    return _samples[ch_num].data / 1000;
}


ssize_t AP_ADC_ADS1115::read(char *buffer, size_t len)
{
    adc_report_s *report = (adc_report_s *)buffer;
    for (int i = 0; i < ADS1115_CHANNELS_COUNT; i++) {
        report[i].data = _samples[i].data;
        report[i].id = _samples[i].id;
    }
    return ADS1115_CHANNELS_COUNT;
}

bool AP_ADC_ADS1115::new_data_available(const uint8_t *channel_numbers)
{
    return true;
}

uint32_t AP_ADC_ADS1115::Ch6(const uint8_t *channel_numbers, float *buffer)
{
    adc_report_s *report = (adc_report_s *)buffer;
    for (int i = 0; i < ADS1115_CHANNELS_COUNT; i++) {
        report[i].data = _samples[i].data;
        report[i].id = _samples[i].id;
    }
    return ADS1115_CHANNELS_COUNT;
}

uint16_t AP_ADC_ADS1115::num_samples_available(const uint8_t *channel_numbers)
{
    return 0;
}


float AP_ADC_ADS1115::_convert_register_data_to_mv(int16_t word)
{

    float pga;

    switch (_gain) {
        case ADS1115_PGA_6P144:  
            pga = ADS1115_MV_6P144;
            break;
        case ADS1115_PGA_4P096:  
            pga = ADS1115_MV_4P096;
            break;
        case ADS1115_PGA_2P048:  
            pga = ADS1115_MV_2P048;
            break;
        case ADS1115_PGA_1P024:  
            pga = ADS1115_MV_1P024;
            break;
        case ADS1115_PGA_0P512:  
            pga = ADS1115_MV_0P512;
            break;
        case ADS1115_PGA_0P256:  
            pga = ADS1115_MV_0P256;
            break;
        case ADS1115_PGA_0P256B: 
            pga = ADS1115_MV_0P256B;
            break;
        case ADS1115_PGA_0P256C:
            pga = ADS1115_MV_0P256C;
            break;
        default:
            hal.console->printf("Wrong gain");
            hal.scheduler->panic("ADS1115: wrong gain selected");
            break;
    }

    return (float) word * pga;
}

void AP_ADC_ADS1115::_update() 
{
    /* TODO: make update rate configurable */
    if (hal.scheduler->micros() - _last_update_timestamp < 100000) {
        return;
    }

    union {
        uint8_t b[2];
        int16_t w;
    } word, config;

    if (!_i2c_sem->take_nonblocking()) {
        return;
    }

    if ( hal.i2c->readRegisters(ADS1115_ADDRESS_ADDR_GND, ADS1115_RA_CONFIG, 2, config.b) != 0 ) {
        error("i2c->readRegisters failed in ADS1115");
        _i2c_sem->give();
        return;
    }

    /* check rdy bit */
    if ((config.b[1] & 0x80) != 0x80 ) {
        _i2c_sem->give();
        return;
    }

    if ( hal.i2c->readRegisters(ADS1115_ADDRESS_ADDR_GND, ADS1115_RA_CONVERSION, 2, word.b) != 0 ) {
        _i2c_sem->give();
        return;
    }

    word.w = word.b[0] << 8 | word.b[1]; /* Exchange MSB and LSB */

    float sample = _convert_register_data_to_mv(word.w);

    _samples[_channel_to_read].data = sample;
    _samples[_channel_to_read].id = _channel_to_read;

    /* select next channel */
    _channel_to_read = (_channel_to_read + 1) % ADS1115_CHANNELS_COUNT;
    _start_conversion(_channel_to_read);

    _i2c_sem->give();

    _last_update_timestamp = hal.scheduler->micros();
}
