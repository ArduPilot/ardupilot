#pragma once

/// UTILS
#include <stdio.h>
#include <iostream>

#include "smartaudio_protocol.h"


//
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include "SPEC_SmartAudioDevice.h"

#define SA_IO_TIMEOUT_MS 200 // DOUBLE THAN RECOMMENDED ON THE TBS SMARTAUDIO DOCUMENT <100ms
#define SA_BUFFER_SIZE 16    // Not used to init uart ?
#define DBM_MAX_ARRAY_SIZE 6    // Max side for the array containing the power modes based in dbm

#define SA_SPEC_PROTOCOL_v1  0
#define SA_SPEC_PROTOCOL_v2  1
#define SA_SPEC_PROTOCOL_v21 2

/// Internal auto discovery behaviours
#define AUTO_DISCOVERY_PENDING  0x00
#define AUTO_DISCOVERY_SUCCESS  0x01
#define AUTO_DISCOVERY_FAILED   0x02

/// COMMANDS
#define SA_SPEC_CMD_GET_SETTINGS     0x01
#define SA_SPEC_CMD_SET_POWER        0x02
#define SA_SPEC_CMD_SET_CHANNEL      0x03
#define SA_SPEC_CMD_SET_FRECUENCY    0x04

#define SA_SPEC_V2_CMD_SET_MODE         0x05        // Available only in v2

#define LOG_TAG "AP_SmartAudio"



/// MODE FLAGS
#define SA_SPEC_V21_MODE_PIT_IN_RANGE   0x01
#define SA_SPEC_V21_MODE_PIT_OUT_RANGE  0x02
#define SA_SPEC_V21_MODE_PIT__DISABLE   0x04
#define SA_SPEC_V21_MODE_LOCK_TOGGLE    0x08



class AP_SmartAudio
{

public:
    AP_SmartAudio();

    /* Do not allow copies */
    AP_SmartAudio(const AP_SmartAudio &other) = delete;
    AP_SmartAudio &operator=(const AP_SmartAudio&) = delete;

    static const struct AP_Param::GroupInfo var_info[];

    void init();
    void update();


    void set_power_level(uint8_t level) { power_level_request = level; }
    uint8_t get_power_level() const { return power_level; }

    void refresh_settings() { get_settings_request = true; }

    static AP_SmartAudio *get_singleton(void) { return singleton; }

    // SPEC FEATURES
    void getSettings();
    void setChannel(uint8_t channel);
    uint8_t getChannel();
    uint16_t getPower();
    void setPower(uint8_t powerLevel);
    void setFrecuency(uint16_t frec,bool isPitModeFrec);
    uint16_t getFrecuency();


private:
    // Pointer to singleton
    static AP_SmartAudio* singleton;
    AP_HAL::UARTDriver* port;

    // Device specification ( or setup )
    SPEC_SmartAudioDevice* device;

    // CURRENT COMMAND FUNCTION POINTER
     uint8_t *currentCommand;
     uint8_t *lastExecutedCommand;

    const uint8_t header_size = 4; // header bytes number of the request and response
    uint8_t buffer[SA_BUFFER_SIZE];
    uint8_t buffer_len;
    uint32_t last_io_time;

    void initPort();
    void beforeSend();
    void autoDiscoverSpec();

    void parseSettings(smartaudioSettingsResponseFrame_s response);

    void send_frame(uint8_t frameBuffer[],uint8_t size);
    void parseFrameResponse(uint8_t frameBuffer[]);

    void read_response();


    bool is_waiting_response() const { return AP_HAL::millis() - last_io_time < SA_IO_TIMEOUT_MS; }

    void logToStdOut(char *message){std::cout << "AP_SmartAudio -> "<< message << std::endl;}




    // desired values
    bool get_settings_request;
    int8_t power_level_request;

    // actual values
    uint8_t protocol;
    uint8_t channel;
    uint8_t power_level;
    uint8_t power_dBm;
    uint8_t power_levels_count;

    uint8_t operation_mode;
    uint8_t frequency;

    //AP_Int8 protocol_version;
    AP_Int8 enabled;
    AP_Int8 use_trailing_zero;


void print_bytes_to_hex_string(uint8_t buf[],uint8_t x)
{
  int i;
    for (i = 0; i < x; i++)
        {
         if (i > 0) printf(":");
            printf("%02X", buf[i]);
        }
    printf("\n");
    }

};