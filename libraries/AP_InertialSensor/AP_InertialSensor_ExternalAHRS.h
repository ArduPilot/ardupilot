#pragma once

#include <AP_ExternalAHRS/AP_ExternalAHRS.h>

#if HAL_EXTERNAL_AHRS_ENABLED

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_ExternalAHRS : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_ExternalAHRS(AP_InertialSensor &imu, uint8_t serial_port);

    /* update accel and gyro state */
    bool update() override;
    void start() override;
    void accumulate() override;

    void handle_external(const AP_ExternalAHRS::ins_data_message_t &pkt) override;
    bool get_output_banner(char* banner, uint8_t banner_len) override;

private:
    uint8_t gyro_instance;
    uint8_t accel_instance;
    const uint8_t serial_port;
    bool started;
};
#endif // HAL_EXTERNAL_AHRS_ENABLED

