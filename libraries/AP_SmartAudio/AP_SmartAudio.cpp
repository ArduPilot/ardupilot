#include "AP_SmartAudio.h"
#include "AP_SmartAudioDeviceV2_OneWay.h"
#include <AP_HAL/AP_HAL.h>

#define SMARTAUDIO_BAUDRATE 4800

const AP_Param::GroupInfo AP_SmartAudio::var_info[] = {

    // @Param: PROTOCOL
    // @DisplayName: SmartAudio protocol version
    // @Description: 0: None, 3: Version 2.0 One way
    // @User: Standard
    AP_GROUPINFO("PROTOCOL", 1, AP_SmartAudio, protocol_version, 0),

    // @Param: TRAILZERO
    // @DisplayName: Enable trailing zero
    // @Description: If enabled the trailing zero will be sent after SA frame. Some VTX requires that.
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("TRAILZERO", 2, AP_SmartAudio, use_trailing_zero, false),

    // @Param: CHANNEL
    // @DisplayName: VTX Channel
    // @Description: Channel to be selected. 0-7: Band A, 8-15: Band B, 16-23: Band E, 24-31: Band Air, 32-39: Band Race
    // @Range: 0 39
    // @User: Standard
    AP_GROUPINFO("CHANNEL", 3, AP_SmartAudio, channel, 0),

    // @Param: PWR_MODE
    // @DisplayName: Power mode 
    // @Description: 0-4: Using of PWR_0 - PWR_4 value, 5: Use Auto power mode based on home distanse 
    // @User: Standard
    AP_GROUPINFO("PWR_MODE", 4, AP_SmartAudio, power_mode, 0),

    // @Param: PWR_0
    // @DisplayName: Power #0
    // @Description: First (minimal) VTX power level
    // @User: Standard
    AP_GROUPINFO("PWR_0", 5, AP_SmartAudio, power_0, -1),

    // @Param: PWR_1
    // @DisplayName: Power #1
    // @Description: Second power level
    // @User: Standard
    AP_GROUPINFO("PWR_1", 6, AP_SmartAudio, power_1, -1),

    // @Param: PWR_2
    // @DisplayName: Power #2
    // @Description: Third power level
    // @User: Standard
    AP_GROUPINFO("PWR_2", 7, AP_SmartAudio, power_2, -1),

    // @Param: PWR_3
    // @DisplayName: Power #3
    // @Description: Fourth power level
    // @User: Standard
    AP_GROUPINFO("PWR_3", 8, AP_SmartAudio, power_3, -1),

    // @Param: PWR_4
    // @DisplayName: Power #4
    // @Description: Fifth power level
    // @User: Standard
    AP_GROUPINFO("PWR_4", 9, AP_SmartAudio, power_4, -1),

    AP_GROUPEND
};

AP_SmartAudio::AP_SmartAudio()
{
    AP_Param::setup_object_defaults(this, var_info);
    singleton = this;
}

void AP_SmartAudio::init()
{
    AP_HAL::UARTDriver *port = AP_SerialManager::get_singleton()->find_serial(AP_SerialManager::SerialProtocol_SmartAudio, 0);

    if (port != nullptr) {
        port->begin(SMARTAUDIO_BAUDRATE, 16, 16);
        port->set_stop_bits(2);
        port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);

        switch(protocol_version) {
            case SmartAudioProtocol_V2_OneWay:
                device = new AP_SmartAudioDeviceV2_OneWay(port, use_trailing_zero);
                break;
        }
    }
}

void AP_SmartAudio::update()
{
    if (device == nullptr) {
        return;
    }

    // update power
    int8_t power = -1;
    switch ((SmartAudio_PowerMode)(int)power_mode) {
        case SmartAudio_Power_AUTO:
            // TODO: calculate power basing on home distance
            break;

        case SmartAudio_Power_0: power = power_0; break;

        case SmartAudio_Power_1: power = power_1; break;

        case SmartAudio_Power_2: power = power_2; break;

        case SmartAudio_Power_3: power = power_3; break;

        case SmartAudio_Power_4: power = power_4; break;
    }
    if (power >= 0) {
        device->set_power(power);
    }

    // update channel
    device->set_channel(channel);

    // update device
    device->update();
}

AP_SmartAudio *AP_SmartAudio::singleton;