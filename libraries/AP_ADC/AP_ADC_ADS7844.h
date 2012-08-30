/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_ADC_ADS7844_H
#define AP_ADC_ADS7844_H

#define bit_set(p,m)   ((p) |= ( 1<<m))
#define bit_clear(p,m) ((p) &= ~(1<<m))

// We use Serial Port 2 in SPI Mode
#define ADC_DATAOUT     51    // MOSI
#define ADC_DATAIN      50    // MISO
#define ADC_SPICLOCK    52    // SCK
#define ADC_CHIP_SELECT 33    // PC4   9 // PH6  Puerto:0x08 Bit mask : 0x40

// DO NOT CHANGE FROM 8!!
#define ADC_ACCEL_FILTER_SIZE 8

#include "AP_ADC.h"
#include "../AP_PeriodicProcess/AP_PeriodicProcess.h"
#include <inttypes.h>

class AP_ADC_ADS7844 : public AP_ADC
{
public:
    AP_ADC_ADS7844();      // Constructor
    void                Init( AP_PeriodicProcess * scheduler );

    // Read 1 sensor value
    float               Ch(unsigned char ch_num);

    // Read 6 sensors at once
    uint32_t            Ch6(const uint8_t *channel_numbers, float *result);

    // check if Ch6 would block
    bool                new_data_available(const uint8_t *channel_numbers);

    // Get minimum number of samples read from the sensors
    uint16_t            num_samples_available(const uint8_t *channel_numbers);

private:
    static void         read(uint32_t);

};

#endif
