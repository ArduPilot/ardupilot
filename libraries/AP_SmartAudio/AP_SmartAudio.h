#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Param/AP_Param.h>
#include <stdio.h>

#ifndef HAL_SMARTAUDIO_ENABLED
#define HAL_SMARTAUDIO_ENABLED !HAL_MINIMIZE_FEATURES  && !APM_BUILD_TYPE(APM_BUILD_Replay) && !APM_BUILD_TYPE(APM_BUILD_AntennaTracker)
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#define BOARD_IS_SITL
#endif

#if HAL_SMARTAUDIO_ENABLED
#define SMARTAUDIO_BUFFER_CAPACITY 5

// SmartAudio Serial Protocol
#define AP_SMARTAUDIO_UART_BAUD            4800
#define AP_SMARTAUDIO_UART_BUFSIZE_RX      16
#define AP_SMARTAUDIO_UART_BUFSIZE_TX      16

#include "smartaudio_protocol.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_RCTelemetry/AP_VideoTX.h>


class AP_SmartAudio
{

public:

    // request packet to be processed
    struct Packet{
        smartaudioFrame_t frame;
        uint8_t frame_siz;
        uint32_t sended_at;
    } PACKED;

    // configured external params
    struct configured_smartaudio_params{
        AP_Int8  enabled;
        AP_Int8  protocol_version;
        AP_Int8  setup_defaults;
    } smartaudio_params;

    AP_SmartAudio();

    static AP_SmartAudio *get_singleton(void)
    {
        return singleton;
    }

    /* Do not allow copies */
    AP_SmartAudio(const AP_SmartAudio &other) = delete;

    AP_SmartAudio &operator=(const AP_SmartAudio&) = delete;

    static const struct AP_Param::GroupInfo var_info[];

    // init threads and lookup for io uart.
    bool init();

    // looping over request loop
    void loop();

    // updates the smartaudio state in sync whith AP_VideoTX
    bool update(bool force);

    // RingBuffer to store outgoing request.
    ObjectBuffer<Packet> requests_queue{SMARTAUDIO_BUFFER_CAPACITY};

    // time the last_request is process
    uint32_t last_request_sended_at;
    // loops is waiting a response after a request
    bool is_waiting_response=false;

    // sends a frame over the wire
    void send_request(smartaudioFrame_t requestFrame,uint8_t size);

    // receives a frame response over the wire
    void read_response(uint8_t *response_buffer,uint8_t inline_buffer_length);

    // parses the response and updates the AP_VTX readings
    bool parse_frame_response(const uint8_t *buffer);

    // to prevent use of semaphores use an vtx_states_queue with 2 elements, so one buffered reading
    ObjectBuffer<smartaudioSettings_s> vtx_states_queue{2};

    // get last reading from the fifo queue
    bool get_readings(AP_VideoTX *vtx_dest);

    // enqueue a get settings request
    void request_settings();

    void set_operation_mode(uint8_t mode);

    // enqueue a set frecuency request, specifiying if the setted frequency is for pit mode or not
    void set_frequency(uint16_t frecuency,bool isPitModeFreq);

    // enqueue a set channel request
    void set_channel(uint8_t chan);

    // enqueue a get pit mode frequency request
    void get_pit_mode_frequency();

    // enqueue a set power request using dbm
    void set_power_dbm(uint8_t power);

    void set_power_mw(uint16_t power_mw);

    /**
     *
     * */
    void print_bytes_to_hex_string(uint8_t buf[],uint8_t x);
    // {
    //     int i;
    //     for (i = 0; i < x; i++) {
    //         if (i > 0) {
    //             printf(":");
    //         }
    //         printf("%02X", buf[i]);
    //     }
    //     printf("\n");
    // }

       // utility method to get power in dbm unit from the settled power level
    static uint8_t _get_power_in_dbm_from_vtx_power_level(uint8_t power_level, uint8_t& protocol_version, uint8_t& power_in_dbm);

        // utility method to get power in dbm unit from the settled power level
    static uint8_t _get_power_in_dbm_from_vtx_power_level(uint8_t power_level, uint8_t& protocol_version);

    // utility method to get power level which corresponds to a dbm power defined
    static uint8_t _get_power_level_from_dbm(uint8_t sma_version,uint8_t power);

private:
    // serial interface
    AP_HAL::UARTDriver *_port;                  // UART used to send data to SmartAudio VTX

    //Pointer to singleton
    static AP_SmartAudio* singleton;

    // get current state, first on fifo queue
    smartaudioSettings_t* _get_current_state(smartaudioSettings_t *stateStorage){


         if(!vtx_states_queue.is_empty()){

            vtx_states_queue.peek(stateStorage,1);
         }else{
             stateStorage=nullptr;
         }

        return stateStorage;
    }

    void _print_state(smartaudioSettings_t *state);



static uint16_t _get_power_in_mw_from_dbm(uint8_t power){
     switch (power) {
    case 14:
        return  25;

    case 20:
        return  100;

    case 23:
        return  200;

    case 26:
        return  400;

    case 27:
        return  500;

    case 29:
        return  800;

    default:
        return uint16_t(roundf(powf(10, power / 10.0f)));
    }
}

static uint16_t _get_power_in_dbm_from_mw(uint16_t power){
     switch (power) {
    case 25:
        return  14;

    case 100:
        return  20;

    case 200:
        return  23;

    case 400:
        return  26;

    case 500:
        return  27;

    case 800:
        return  29;

    default:
        return uint16_t(roundf(powf(10, power / 10.0f)));
    }
}

};
#endif