#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include "AP_SmartAudioDevice.h"

class AP_SmartAudioDeviceV2_OneWay : public AP_SmartAudioDevice
{

public:
    AP_SmartAudioDeviceV2_OneWay(AP_HAL::UARTDriver* _port, bool _use_trailing_zero) 
        : AP_SmartAudioDevice(_port, _use_trailing_zero) { };

protected:
    void send_update_power_command();
    void send_update_channel_command();
    bool read_response();

private:
    uint32_t last_action_time_ms;

    uint8_t CMD[8] = { 0xAA, 0x55 };
};