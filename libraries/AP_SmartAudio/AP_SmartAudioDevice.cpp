#include <AP_HAL/AP_HAL.h>
#include "AP_SmartAudioDevice.h"

AP_SmartAudioDevice::AP_SmartAudioDevice(AP_HAL::UARTDriver* _port, bool _use_trailing_zero) :
    port(_port),
    use_trailing_zero(_use_trailing_zero)
{
}

void AP_SmartAudioDevice::update()
{
    switch(state) {
        case State_READY:
            if (update_power() || update_channel())
                state = State_READING_RESPONSE;
            break;

        case State_READING_RESPONSE:
            if (read_response())
                state = State_READY;
            break;
    }
}

bool AP_SmartAudioDevice::update_power()
{
    if (actual_state.power == desired_state.power) {
        return false;
    }

    send_update_power_command();

    return true;
}

bool AP_SmartAudioDevice::update_channel()
{
    if (actual_state.channel == desired_state.channel) {
        return false;
    }

    send_update_channel_command();

    return true;
}

void AP_SmartAudioDevice::send_command(uint8_t* command, uint8_t size)
{
    // pull line low
    port->write((uint8_t)0);

    // write command
    for(int i = 0; i < size; ++i) {
	    port->write(command[i]);
    }

    // write crc
    port->write(get_crc8(command, size));

    // some VTX need this
    if (use_trailing_zero) {
        port->write((uint8_t)0);
    }
}

uint8_t AP_SmartAudioDevice::get_crc8(uint8_t* command, uint8_t size)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < size; i++) {
        crc = crc8tab[crc ^ *command++];
    }
    return crc;
}