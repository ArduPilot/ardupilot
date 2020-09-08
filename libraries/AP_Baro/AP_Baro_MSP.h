/*
  MSP backend barometer
 */
#pragma once

#include "AP_Baro_Backend.h"

#define MOVING_AVERAGE_WEIGHT 0.20f // a 5 samples moving average

#if HAL_MSP_BARO_ENABLED

class AP_Baro_MSP : public AP_Baro_Backend
{
public:
    AP_Baro_MSP(AP_Baro &baro, uint8_t msp_instance);
    void update(void) override;
    void handle_msp(const MSP::msp_baro_data_message_t &pkt) override;

private:
    uint8_t instance;
    uint8_t msp_instance;
    float sum_pressure;
    float sum_temp;
    uint16_t count;
};

#endif // HAL_MSP_BARO_ENABLED
