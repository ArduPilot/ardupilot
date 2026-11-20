/*
   Xsens MTData2 decoder for the ATAL MTi-300 integration.
   Protocol field selection follows AUV-IITK/ATAL_controller's MIT-licensed
   xsens_mti_driver_node.py. This parser accepts only complete NED float32
   samples; no serial device or actuator is accessed here.
 */

#pragma once

#include <stdint.h>

class AP_ExternalAHRS_Xsens_Parser {
public:
    struct Sample {
        float quaternion_wxyz[4];
        float accel_mps2[3];
        float gyro_rad_s[3];
        uint16_t counter;
    };

    // Returns true only after a complete, valid MTData2 packet.
    bool feed(uint8_t byte, Sample &sample);

private:
    bool decode(Sample &sample) const;
    uint8_t packet[260] {};
    uint16_t count = 0;
    uint16_t expected = 0;
};
