/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef __AP_ADC_ADS1115_H__
#define __AP_ADC_ADS1115_H__


#include <inttypes.h>
#include "AP_ADC.h"
#include <AP_HAL.h>

struct adc_report_s 
{
    uint8_t id;
    float data;
};

class AP_ADC_ADS1115 : public AP_ADC
{
public:
    AP_ADC_ADS1115();      // Constructor
    void      Init();

    // Read 1 sensor value
    float               Ch(unsigned char ch_num);

    // Read 6 sensors at once
    uint32_t            Ch6(const uint8_t *channel_numbers, float *result);
    ssize_t             read(char *buffer, size_t len);

    // check if Ch6 would block
    bool                new_data_available(const uint8_t *channel_numbers);

    // Get minimum number of samples read from the sensors
    uint16_t            num_samples_available(const uint8_t *channel_numbers);

private:
    uint32_t            _last_update_timestamp;
    uint16_t            _gain;
    int                 _channel_to_read;
    adc_report_s        _samples[8];

    AP_HAL::Semaphore*  _i2c_sem;
    void _update();
    bool _start_conversion(uint8_t channel);
    float _convert_register_data_to_mv(int16_t word);
};

#endif
