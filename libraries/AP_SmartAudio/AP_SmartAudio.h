#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Param/AP_Param.h>

#ifndef SMARTAUDIO_ENABLED
#define SMARTAUDIO_ENABLED !HAL_MINIMIZE_FEATURES && !APM_BUILD_TYPE(APM_BUILD_Replay)
#endif

#if SMARTAUDIO_ENABLED
#define SMARTAUDIO_BUFFER_CAPACITY 5

#include "smartaudio_protocol.h"
#include "AP_Vtx_Params.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_RCTelemetry/AP_VideoTX.h>


class AP_SmartAudio
{

public:
    /** Operation modes types **/
    enum OperationMode {
        in_range_pit_mode=0,
        out_range_pit_mode,
        disabled_pit_mode,
        locked,
        unlocked
    };

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

    // request or response packet to be processed
    struct packet{
        smartaudioFrame_t frame;
        uint8_t frame_size=0;
        uint32_t sended_at=0;
    } PACKED;

    // looping over
    void loop();

    // RingBuffer to store outgoing request.
    ObjectBuffer<packet> requests_queue{SMARTAUDIO_BUFFER_CAPACITY};

    // time the last_request is process
    uint32_t last_request_sended_at=0;
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


    /**
     * Get settings from vtx.
     * Depends on protocol version.
     * Features:
     *  -> Retrieve frec info.
     *  -> Retrive chan info.
     *  -> Retrieve power info.
     *  -> Retrieve mode info.
    */
    void request_settings();




    void update();

    void update_readings();



    /**
     * Set operation mode.
     * Only works with smaraudio protocols since 2.0
     * Features:
     *  -> Disable pit mode;
     *  -> Unlock/lock vtx power levels.
     *  -> Enable pit mode out of band, setting  5584 MHz via hardware
     *  -> Enable pit mode in band. Frecuency is not changed
     */
    void set_operation_mode(OperationMode mode,bool locked);
    /**
     *
     **/


    /**
     * Sets the frecuency to transmit in the vtx.
     * When isPitModeFreq active the frec will be set to be used when in pitmode (in range)
     *
     * Features:
     *  - Set frequency
     *  - Set pit mode frequency
     */
    void set_frequency(uint16_t frecuency,bool isPitModeFreq);


    /**
     * Set the power to the vtx device returning the dbm.
     *
     *
     *
     */
    void set_power_dbm(uint8_t power);

    /**
     *
     * */
    void static print_bytes_to_hex_string(uint8_t buf[],uint8_t x)
    {
        int i;
        for (i = 0; i < x; i++) {
            if (i > 0) {
                printf(":");
            }
            printf("%02X", buf[i]);
        }
        printf("\n");
    }



protected:
    AP_Vtx_Params vtx_params;

private:
    AP_HAL::UARTDriver *_port;                  // UART used to send data to SmartAudio VTX
    //Pointer to singleton
    static AP_SmartAudio* singleton;

    bool _get_power_in_dbm_from_vtx_power_level(uint8_t power_level, uint8_t& protocol_version, uint8_t& power_in_dbm);

    void monkey_testing()
    {
        uint8_t response_settings_v1[10]={0xAA,0x55,0x01,0x06,0x00,0x00,0x01,0x16,0xE9,0x4D};
        uint8_t response_settings_v2[10]={0xAA,0x55,0x09,0x06,0x01,0x00,0x1A,0x16,0xE9,0x0A};
        uint8_t response_settings_v3[16]={0xAA,0x55,0x11,0x0C,0x00,0x00,0x00,0x16,0xE9,0x0E,0x03,0x00,0x0E,0x14,0x1A,0x01};

        const int operation=rand() % 20;
        printf("Free mem: %d\n",AP_HAL::get_HAL().util->available_memory());

        switch (operation) {
        case SMARTAUDIO_CMD_GET_SETTINGS:
            printf("\n");
            request_settings();
            parse_frame_response(response_settings_v1);
            get_readings(AP_VideoTX::get_singleton());
            // asserts
            if(AP_VideoTX::get_singleton()->get_band()!=0){
                AP_HAL::panic("Band is not correct with response package");
            }
            if(AP_VideoTX::get_singleton()->get_channel()!=1){
                AP_HAL::panic("Channel is not correct with response package ");
            }
            if(AP_VideoTX::get_singleton()->get_frequency_mhz()!=5865){
                AP_HAL::panic("Frequency is not correct with response package  %d vs %d",5865,AP_VideoTX::get_singleton()->get_frequency_mhz());
            }
            if(AP_VideoTX::get_singleton()->get_locking()!=1){
                AP_HAL::panic("Locking is not correct with response package");
            }
            if(AP_VideoTX::get_singleton()->get_power_mw()!=25){
                AP_HAL::panic("Power is not correct with response package");
            }
            if(AP_VideoTX::get_singleton()->get_options()!=0){
                AP_HAL::panic("Options is not correct with response package");
            }

            request_settings();
            parse_frame_response(response_settings_v2);
            get_readings(AP_VideoTX::get_singleton());

            request_settings();
            parse_frame_response(response_settings_v3);
            get_readings(AP_VideoTX::get_singleton());


            break;
        case SMARTAUDIO_CMD_SET_MODE:
            switch (rand() % 4) {
            case 0:
                set_operation_mode(OperationMode::in_range_pit_mode,rand() %1==1);
                break;
            case 1:
                set_operation_mode(OperationMode::out_range_pit_mode,rand() %1==1);
                break;
            case 2:
                set_operation_mode(OperationMode::disabled_pit_mode,rand() %1==1);
                break;
            case 3:
                set_operation_mode(OperationMode::locked,rand() %1==1);
                break;
            case 4:
                set_operation_mode(OperationMode::unlocked,rand() %1==1);
                break;
            };
            break;

        case SMARTAUDIO_CMD_SET_FREQUENCY:
            printf("\n");
            set_frequency(rand() % 5865,rand() %1==1);
            break;
        case SMARTAUDIO_CMD_SET_POWER:
            printf("\n");
            set_power_dbm(23);
            break;

        case 18:
            get_readings(AP_VideoTX::get_singleton());
            break;
        // case 19:
        //     backend->set_io_status(AP_Vtx_SerialBackend::RqStatus::idle);
        //     backend->set_status(AP_Vtx_Backend::Status::idle);
        //     break;
         }
    }

};

#endif