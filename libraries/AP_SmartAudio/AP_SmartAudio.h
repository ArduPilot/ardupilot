#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include "AP_SmartAudioDevice.h"

class AP_SmartAudio
{

public:
    AP_SmartAudio();

    /* Do not allow copies */
    AP_SmartAudio(const AP_SmartAudio &other) = delete;
    AP_SmartAudio &operator=(const AP_SmartAudio&) = delete;

    static const struct AP_Param::GroupInfo var_info[];

    enum SmartAudioProtocol {
        SmartAudioProtocol_NONE         = 0,
        SmartAudioProtocol_V1_OneWay    = 1,
        SmartAudioProtocol_V1           = 2,        
        SmartAudioProtocol_V2_OneWay    = 3,
        SmartAudioProtocol_V2           = 4,
        SmartAudioProtocol_V21_OneWay   = 5,
        SmartAudioProtocol_V21          = 6
    };

    enum SmartAudio_PowerMode {        
        SmartAudio_Power_0,
        SmartAudio_Power_1,
        SmartAudio_Power_2,
        SmartAudio_Power_3,
        SmartAudio_Power_4,
        SmartAudio_Power_AUTO
    };

    void init();
    void update();

    void set_power_mode(SmartAudio_PowerMode mode) { power_mode = (int8_t)mode; }
    int8_t get_power() { return (device != nullptr) ? device->get_power() : SMARTAUDIO_VALUE_UNKNOWN; }

    void set_channel(uint8_t channel);
    int8_t get_channel() const { return (device != nullptr) ? device->get_channel() : SMARTAUDIO_VALUE_UNKNOWN; }

    static AP_SmartAudio *get_singleton(void) { return singleton; }

private:
    static AP_SmartAudio* singleton;

    AP_Int8 protocol_version;
    AP_Int8 use_trailing_zero;
    AP_Int8 channel;
    AP_Int8 power_mode;
    AP_Int8 power_value[5];

    AP_SmartAudioDevice* device;
};