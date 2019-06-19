#include <AP_HAL/AP_HAL.h>
#include "AP_SmartAudioDeviceV2_OneWay.h"

#define SMART_AUDIO_DELAY_MS 500

#define CMD_HEADER 0xAA, 0x55
#define CMD_POWER 0x05
#define CMD_CHANNEL 0x07

void AP_SmartAudioDeviceV2_OneWay::send_update_power_command()
{
    uint8_t cmd[] = { CMD_HEADER, CMD_POWER, 0x01, (uint8_t)desired_state.power };

    send_command(cmd, sizeof(cmd));

    actual_state.power = desired_state.power;

    last_action_time_ms = AP_HAL::millis();
}

void AP_SmartAudioDeviceV2_OneWay::send_update_channel_command()
{
    uint8_t cmd[] = { CMD_HEADER, CMD_CHANNEL, 0x01, (uint8_t)desired_state.channel };

    send_command(cmd, sizeof(cmd));

    actual_state.channel = desired_state.channel;

    last_action_time_ms = AP_HAL::millis();
}

bool AP_SmartAudioDeviceV2_OneWay::read_response()
{
    // just ensure a delay to not to sent a next command too quickly
    return AP_HAL::millis() - last_action_time_ms > SMART_AUDIO_DELAY_MS;
}