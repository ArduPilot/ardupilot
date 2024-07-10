#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_DDS_ENABLED

class AP_Baro_DDS : public AP_Baro_Backend
{
public:
    AP_Baro_DDS(AP_Baro &baro);
    virtual ~AP_Baro_DDS();
    void update() override;
    void handle_external(const AP_ExternalAHRS::baro_data_message_t &pkt) override;

protected:
    void update_healthy_flag(uint8_t instance) override;

private:
    bool healthy(uint8_t instance);

    uint8_t _instance;
    uint32_t _last_sample_time;
    float _sum_pressure;
    float _sum_temp;
    uint16_t _count;
};

#endif  // AP_BARO_DDS_ENABLED
