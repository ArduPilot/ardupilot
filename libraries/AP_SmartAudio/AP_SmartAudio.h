#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#ifndef SMARTAUDIO_ENABLED
#define SMARTAUDIO_ENABLED !HAL_MINIMIZE_FEATURES && !APM_BUILD_TYPE(APM_BUILD_Replay)
#endif

#if SMARTAUDIO_ENABLED
#define SMARTAUDIO_REQUEST_BUFFER_CAPACITY 5

#include "smartaudio_protocol.h"
#include "AP_Vtx_Params.h"
#include "AP_SmartAudioBackend.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/RingBuffer.h>


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

    // Subclass from the Vtx_State at the deep end to return the readings from frontend.
    struct SmartAudioState : public AP_Vtx_SerialBackend::Vtx_Serial_State {
        bool         initialized=false;
        bool         is_in_range_pit_mode=false;
        bool         is_out_range_pit_mode=false;
        bool         is_pit_mode_disabled=false;
        bool         is_locked=false;

        uint8_t      version=0;
        uint8_t      current_band=0;
        uint8_t      current_channel=0;
        uint32_t     current_frequency=0;
        uint8_t      current_power_db=0;
        uint16_t     current_power_mw=0;
    };




    AP_Int8 setting_low_race;     // Low race setting hv race can use it.
    AP_Int8 ov_settings_enable; // Enable overrides
    AP_Int8 ov_version;          // Protocol version overrides

    AP_SmartAudio();
    static AP_SmartAudio *get_singleton(void)
    {
        return singleton;
    }

    /* Do not allow copies */
    AP_SmartAudio(const AP_SmartAudio &other) = delete;
    AP_SmartAudio &operator=(const AP_SmartAudio&) = delete;
    static const struct AP_Param::GroupInfo var_info[];

    AP_Vtx_Params params;

    bool init();
    // looping over
    void loop();
    // RingBuffer to store incoming request, which will be consumed when the response is received or the timeout is triggered
    ObjectBuffer<smartaudioCommandOnlyFrame_t> queue{SMARTAUDIO_REQUEST_BUFFER_CAPACITY};

    void update();
    SmartAudioState get_readings();
    void update_readings();


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




protected:
    AP_Vtx_Params vtx_params;
    SmartAudioState vtx_state;
    AP_SmartAudioBackend *backend;
private:
    AP_HAL::UARTDriver *_port;                  // UART used to send data to SmartAudio VTX
    //Pointer to singleton
    static AP_SmartAudio* singleton;


    void monkey_testing()
    {
        const int operation=rand() % 20;
        switch (operation) {
        case SMARTAUDIO_CMD_GET_SETTINGS:
            printf("\n");
            request_settings();
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
            get_readings();
            break;
        case 19:
            backend->set_io_status(AP_Vtx_SerialBackend::RqStatus::idle);
            backend->set_status(AP_Vtx_Backend::Status::idle);
            break;
        }
    }

};

#endif