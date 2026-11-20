/* Dedicated ATAL ESP32 Bar30 serial backend. No actuator interface. */

#pragma once

#include "AP_Baro_config.h"

#if AP_BARO_ATAL_DEPTH_ENABLED

#include "AP_Baro_ATAL_Depth_Parser.h"
#include "AP_Baro_Backend.h"

class AP_Baro_ATAL_Depth : public AP_Baro_Backend {
public:
    AP_Baro_ATAL_Depth(AP_Baro &baro, AP_HAL::UARTDriver *uart, uint8_t serial_port);
    void update() override;

private:
    AP_HAL::UARTDriver *uart;
    AP_Baro_ATAL_Depth_Parser parser;
    uint8_t instance;
};

#endif // AP_BARO_ATAL_DEPTH_ENABLED
