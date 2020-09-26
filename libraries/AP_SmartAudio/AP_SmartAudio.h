#pragma once

#include <AP_HAL/AP_HAL.h>


#ifndef HAL_SMARTAUDIO_ENABLED
#define HAL_SMARTAUDIO_ENABLED !HAL_MINIMIZE_FEATURES
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


#include <AP_Param/AP_Param.h>
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
        uint8_t frame_size;
        uint32_t sended_at;
    } PACKED;

    // Enable SmartAudio Protocol by param
    AP_Int8 _smart_audio_param_enabled;
    // Set protocol version NOT USED YET
    AP_Int8 _smart_audio_param_protocol_version;
    // When enabled the settings returned from hw vtx are settled into ap_videoTx params NOT USED YET
    AP_Int8 _smart_audio_param_setup_defaults;

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
    void send_request(smartaudioFrame_t requestFrame, uint8_t size);

    // receives a frame response over the wire
    void read_response(uint8_t *response_buffer, uint8_t inline_buffer_length);

    // parses the response and updates the AP_VTX readings
    bool parse_frame_response(const uint8_t *buffer);

    // to prevent use of semaphores use an vtx_states_queue with 2 elements, so one buffered reading
    ObjectBuffer<smartaudioSettings_s> vtx_states_queue{2};

    // get last reading from the fifo queue
    bool get_readings(AP_VideoTX *vtx_dest);

    // enqueue a get settings request
    void request_settings();

    void set_operation_mode(uint8_t mode);

    // enqueue a set frequency request, specifiying if the setted frequency is for pit mode or not
    void set_frequency(uint16_t frequency, bool isPitModeFreq);

    // enqueue a set channel request
    void set_channel(uint8_t chan);

    // enqueue a get pit mode frequency request
    void get_pit_mode_frequency();

    // enqueue a set power request using dbm
    void set_power_dbm(uint8_t power);

    // enqueue a set power request using mw
    void set_power_mw(uint16_t power_mw);

    // utility method to get power in dbm unit from the settled power level
    static uint8_t _get_power_in_dbm_from_vtx_power_level(uint8_t power_level, uint8_t& protocol_version, uint8_t& power_in_dbm);

    // utility method to get power in dbm unit from the settled power level
    static uint8_t _get_power_in_dbm_from_vtx_power_level(uint8_t power_level, uint8_t& protocol_version);

    // utility method to get power level which corresponds to a dbm power defined
    static uint8_t _get_power_level_from_dbm(uint8_t sma_version, uint8_t power);

private:
    // serial interface
    AP_HAL::UARTDriver *_port;                  // UART used to send data to SmartAudio VTX

    //Pointer to singleton
    static AP_SmartAudio* singleton;

    // get current state, first on fifo queue
    smartaudioSettings_t* _get_current_state(smartaudioSettings_t *stateStorage){


         if(!vtx_states_queue.is_empty()){

            vtx_states_queue.peek(stateStorage, 1);
         }else{
             stateStorage=nullptr;
         }

        return stateStorage;
    }

    // utility method for debugging
    void _print_state(smartaudioSettings_t *state);

    // utility method for debugging.
    void _print_bytes_to_hex_string(uint8_t buf[], uint8_t x);


    // utility method to get dbm transformation from power
    static uint16_t _get_power_in_mw_from_dbm(uint8_t power){
        for(int i=0;i<7;i++){
            if (POW_MW_DBM_REL_TABLE[0][i]==uint16_t(power)){
                return POW_MW_DBM_REL_TABLE[0][i];
            }
        }
        return uint16_t(roundf(powf(10, power / 10.0f)));
    }


    // utility method to get mw transformation from power
    static uint16_t _get_power_in_dbm_from_mw(uint16_t power){
        for(int i=0;i<7;i++){
            if (POW_MW_DBM_REL_TABLE[1][i]==uint16_t(power)){
                return POW_MW_DBM_REL_TABLE[1][i];
            }
        }
            return uint16_t(roundf(powf(10, power / 10.0f)));
    }

};
#endif
