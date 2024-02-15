/*
  ExternalAHRS backend barometer
 */
#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_EXTERNALAHRS_ENABLED

class AP_Baro_ExternalAHRS : public AP_Baro_Backend
{
public:
    AP_Baro_ExternalAHRS(AP_Baro &baro, uint8_t serial_port);
    void update(void) override;
    void handle_external(const AP_ExternalAHRS::baro_data_message_t &pkt) override;

private:
    uint8_t instance;
    float sum_pressure;
    float sum_temp;
    uint16_t count;
};

#endif // AP_BARO_EXTERNALAHRS_ENABLED
