/*
   Parser for the dedicated ATAL ESP32 depth controller's serial stream.
   The ESP32 sends DEPTH:pressure_mbar,temp_c,depth_m at 20 Hz.
 */

#pragma once

#include <stdint.h>

class AP_Baro_ATAL_Depth_Parser {
public:
    struct Sample {
        float pressure_pa;
        float temperature_c;
        float diagnostic_depth_m;
    };

    // Returns true only for a complete, valid DEPTH line.
    bool feed(uint8_t byte, Sample &sample);

private:
    bool parse(Sample &sample);

    char line[96] {};
    uint8_t length = 0;
    bool discarding = false;
};
