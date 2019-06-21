#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_SerialManager/AP_SerialManager.h>

#define SMARTAUDIO_VALUE_UNKNOWN -1

class AP_SmartAudioDevice
{

public:
    AP_SmartAudioDevice(AP_HAL::UARTDriver* _port, bool _use_trailing_zero);

    int8_t get_power() const { return actual_state.power; }
    void set_power(int8_t power) { desired_state.power = power; }

    int8_t get_channel() const { return actual_state.channel; }
    void set_channel(int8_t channel) { desired_state.channel = channel; }

    void update();

protected:
    struct {
        int8_t power = SMARTAUDIO_VALUE_UNKNOWN;
        int8_t channel = SMARTAUDIO_VALUE_UNKNOWN;
    } actual_state, desired_state;

    virtual void send_update_power_command() = 0;
    virtual void send_update_channel_command() = 0;

    void send_command(uint8_t* command, uint8_t size);

    // must return true when response reading complete
    virtual bool read_response() = 0;

private:
    enum DeviceState {
        State_READY,
        State_READING_RESPONSE,
    } state;

    AP_HAL::UARTDriver* port;

    bool use_trailing_zero;

    bool update_power();
    bool update_channel();
};