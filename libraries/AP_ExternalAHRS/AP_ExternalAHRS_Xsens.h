/*
   Xsens MTi-300 serial ExternalAHRS backend for ATAL.
   This backend publishes IMU samples only. It has no actuator interface.
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_XSENS_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include "AP_ExternalAHRS_Xsens_Parser.h"

class AP_ExternalAHRS_Xsens : public AP_ExternalAHRS_backend {
public:
    AP_ExternalAHRS_Xsens(AP_ExternalAHRS *frontend_ptr, AP_ExternalAHRS::state_t &state);
    int8_t get_port() const override { return port_num; }
    const char *get_name() const override { return "Xsens MTi-300"; }
    bool healthy() const override;
    bool initialised() const override { return last_sample_ms != 0; }
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void update() override;

protected:
    uint8_t num_gps_sensors() const override { return 0; }

private:
    void send_command(uint8_t mid, const uint8_t *payload, uint8_t length);
    void publish(const AP_ExternalAHRS_Xsens_Parser::Sample &sample);

    AP_HAL::UARTDriver *uart;
    AP_ExternalAHRS_Xsens_Parser parser;
    uint32_t baudrate;
    uint32_t last_command_ms;
    uint32_t last_sample_ms;
    uint16_t last_counter;
    int8_t port_num;
    uint8_t setup_step;
    bool have_counter;
    bool port_open;
};

#endif // AP_EXTERNAL_AHRS_XSENS_ENABLED
