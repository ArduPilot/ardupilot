#pragma once


#include "AP_SmartAudio.h"

#include <stdio.h>
#define AP_VTX_DEV_MODE     // DEV TESTS ACTIVATE
#ifdef AP_VTX_DEV_MODE
# define debug(fmt, args...)	hal.console->printf("\t':-), " fmt "\n", ##args)
#else
# define debug(fmt, args...)	do {} while(0)
#endif

extern const AP_HAL::HAL &hal;
const char *TAG="VTX-SMARTAUDIO ";


// table of user settable parameters
const AP_Param::GroupInfo AP_SmartAudio::var_info[] = {

    // @Param: LOW_RACE
    // @DisplayName: Use Low Race band
    // @Description: Use low race band
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("LOW_RACE",     11, AP_SmartAudio, setting_low_race, 0),

    // @Param: ENABLED
    // @DisplayName: Enable protocol settings override
    // @Description: Override the info from vtx to work with custom data
    // @Values: 0:Disabled,1:Enabled, 3:V2.1
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLED",     1, AP_SmartAudio, ov_settings_enable, 0,AP_PARAM_FLAG_ENABLE),

    // @Param: VERSION
    // @DisplayName: Overrided protocol version
    // @Description: Override the info from vtx to work with this version.
    // @Values: 1:V1,2:V2, 3:V2.1
    // @User: Advanced
    AP_GROUPINFO("VERSION",     2, AP_SmartAudio, ov_version, 1),




};
/**
 * Constructor
 */
AP_SmartAudio::AP_SmartAudio()
{
    AP_Param::setup_object_defaults(this, var_info);
    singleton = this;

}

AP_SmartAudio *AP_SmartAudio::singleton;

// initialization start making a request settings to the vtx
bool AP_SmartAudio::init()
{
    // init uart
    _port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_SmartAudio, 0);
    if(_port!=nullptr){

          if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_SmartAudio::loop, void),
                                          "SmartAudio",
                                          1024, AP_HAL::Scheduler::PRIORITY_UART, 60)) {
            return false;
            }

        // setup port options
        _port->set_stop_bits(AP_SERIALMANAGER_SMARTAUDIO_STOP_BITS);
        _port->set_flow_control(AP_SERIALMANAGER_SMARTAUDIO_FLOW_CONTROL);
        _port->set_options(AP_SERIALMANAGER_SMARTAUDIO_OPTIONS);
        return true;
    }
    return false;
    this->backend=new AP_SmartAudioBackend(vtx_state,params,0);
    backend->init();
    request_settings();
}

void AP_SmartAudio::loop(){
    // initialise uart (this must be called from within tick b/c the UART begin must be called from the same thread as it is used from)
    //_port->begin(AP_SERIALMANAGER_SMARTAUDIO_BAUD, AP_SERIALMANAGER_SMARTAUDIO_BUFSIZE_RX, AP_SERIALMANAGER_SMARTAUDIO_BUFSIZE_TX);
     while (true) {
         // spec says that
        hal.scheduler->delay(100);
        printf("b %d",AP_HAL::millis());
     }

}


/**
  *    |io_status        |vtx_status     |main loop action|
  *    | idle            | idle          | exit           |
  *    | idle            | processing    | error          |
  *    | idle            | unsynced      | resync         |
  *    | idle            | not_connected | exit           |
  *
  *    | requesting      | idle          | error          |
  *    | requesting      | processing    | sending_request|
  *    | requesting      | unsynced      | sending_request|
  *    | requesting      | not_connected | error          |
  *
  *    | waiting response| idle          | error          |
  *    | waiting response| processing    | read_response  |
  *    | waiting response| unsynced      | error          |
  *    | waiting response| not_connected | error          |
  *
  *    | reading response| idle          | error          |
  *    | reading response| processing    | read_response  |
  *    | reading response| unsynced      | error          |
  *    | reading response| not_connected | exit           |
 *
 * */

void AP_SmartAudio::update()
{
#ifdef AP_VTX_DEV_MODE
    monkey_testing();
#endif
    /// NOT CONNECTED FREE MAIN LOOP
    if (backend->status()==AP_Vtx_Backend::Status::not_connected) {
        return;
    }

    // NO IO ACTIVITY, NO VTX ACTIVITY OR SINCRO STABLISHED
    if (backend->get_io_status()==AP_Vtx_SerialBackend::RqStatus::idle  && backend->status()==AP_Vtx_Backend::Status::idle) {
        printf("\n DO NOTHING:%d ms ",AP_HAL::millis());
        return;
    }

    // When Unsynced try to resync data
    if (backend->status()==AP_Vtx_Backend::Status::desynchronized) {
        printf("\nUNSYNCED:%d  ",AP_HAL::millis());
        backend->set_io_status(AP_Vtx_SerialBackend::RqStatus::idle);
        this->request_settings();
    }

    // SLOT TIME FINISHED FREE THE MAIN LOOP
    if (backend->is_io_timeout() && backend->get_io_status()!=AP_Vtx_SerialBackend::RqStatus::idle) {
        printf("\nTIMEOUT CONSUMED:%d milliseconds ",AP_HAL::millis());
        backend->set_status(AP_Vtx_Backend::Status::desynchronized);    // DO SYNCRO IN THE NEXT LOOP KEEP AN EYE TO SOURCE BLOCK POSITION UNDER RESYNC
        return;
    }

    // IF RESPONSE IS PENDING OR IN PROCESS BECAUSE LOOP IS BREAKED READ_RESPONSE
    if (backend->status()==AP_Vtx_Backend::Status::processing
        && (backend->get_io_status()==AP_Vtx_SerialBackend::RqStatus::waiting_response
            ||
            backend->get_io_status()==AP_Vtx_SerialBackend::RqStatus::reading_response
           )
       ) {
        printf("\nREADING RESPONSE IN :%d milliseconds ",AP_HAL::millis());
        backend->read_response();
    }

    printf("\nBACKEND->STATE LAST I/O: %d-%d ms",backend->get_read_time(),backend->get_write_time());
}

/**
 * Sends get settings command.
 * */
void AP_SmartAudio::request_settings()
{
    printf("%80s::request_settings()\t",TAG);
    smartaudioFrame_t request;
    uint8_t frameSize=smartaudioFrameGetSettings(&request);
    backend->send_request(request,frameSize);
}

/**
 * Set operation mode.
 * Only works with smaraudio protocols since 2.0
 * Features:
 *  -> Disable pit mode;
 *  -> Unlock/lock vtx power levels.
 *  -> Enable pit mode out of band, setting  5584 MHz via hardware
 *  -> Enable pit mode in band. Frecuency is not changed
 */
void AP_SmartAudio::set_operation_mode(OperationMode mode,bool locked)
{
    printf("%80s::set_operation_mode(%d,%d)\t",TAG,mode,locked);
    if (ov_settings_enable.get()==1 && ov_version.get()<2) {
        debug("%s HW: %s Device can't change operation mode. Spec protocol not supported",TAG,"set_operation_mode");
        return;
    } else {
        if (backend->version()<2) {
            debug("%s HW: %s",TAG,"Device can't change operation mode. Spec protocol not supported");
            AP::logger().Write_MessageF("%s HW: %s Device can't change operation mode. Spec protocol not supported",TAG,"set_operation_mode");
        }
    }


    smartaudioSettings_t settings;
    switch (mode) {
    case OperationMode::in_range_pit_mode:
        settings.pitmodeInRangeActive=true;
        settings.pitmodeOutRangeActive=false;
        settings.pitmodeDisabled=false;
        settings.unlocked=!locked;
        break;
    case OperationMode::out_range_pit_mode:
        settings.pitmodeInRangeActive=false;
        settings.pitmodeOutRangeActive=true;
        settings.pitmodeDisabled=false;
        settings.unlocked=!locked;
        settings.pitmodeOutRangeActive=true;
        break;
    case OperationMode::disabled_pit_mode:
        settings.pitmodeInRangeActive=false;
        settings.pitmodeOutRangeActive=false;
        settings.pitmodeDisabled=true;
        settings.unlocked=!locked;
        break;
    case OperationMode::locked:
        settings.pitmodeInRangeActive=false;
        settings.pitmodeOutRangeActive=false;
        settings.pitmodeDisabled=false;
        settings.unlocked=false;
        break;
    case OperationMode::unlocked:
        settings.pitmodeInRangeActive=false;
        settings.pitmodeOutRangeActive=false;
        settings.pitmodeDisabled=false;
        settings.unlocked=true;
        break;
    default:
        AP::logger().Write_MessageF("%s HW: %s",TAG,"Operation mode not supported");
        debug("%s HW: %s",TAG,"Operation mode not supported");
        return;
    };
    smartaudioFrame_t request;
    uint8_t frameSize=smartaudioFrameSetOperationMode(&request,&settings);
    backend->send_request(request,frameSize);
}


/**
     * Sets the frecuency to transmit in the vtx.
     * When isPitModeFreq active the frec will be set to be used when in pitmode (in range)
     */
void AP_SmartAudio::set_frequency(uint16_t frecuency,bool isPitModeFreq)
{
    printf("%80s::set_frequency(%d,%d)\t",TAG,frecuency,isPitModeFreq);
    smartaudioFrame_t request;
    uint8_t frameSize=smartaudioFrameSetFrequency(&request,frecuency,isPitModeFreq);
    backend->send_request(request,frameSize);
}

/**
    * Set the power to the vtx device returning the dbm.
    *
    *
    *
    */
void AP_SmartAudio::set_power_dbm(uint8_t power)
{
    printf("%80s::set_power_dbm(%d)\t",TAG,power);
    uint16_t powerLevel=0x00;
    if (backend->version()!=0 && backend->version()!=1 && backend->version()!=2) {
        return;
    }
    smartaudioFrame_t request;
    uint8_t frameSize=0;
    // version 1
    if (backend->version()==0) {
        if (power==14) {
            powerLevel=7;
        }
        if (power==23) {
            powerLevel=16;
        }
        if (power==27) {
            powerLevel=25;
        }
        if (power==29) {
            powerLevel=40;
        }
        frameSize=smartaudioFrameSetPower(&request,powerLevel);
    }
    // hardcoded protocol version 2
    if (backend->version()==1) {
        if (power==14) {
            powerLevel=0;
        }
        if (power==23) {
            powerLevel=1;
        }
        if (power==27) {
            powerLevel=2;
        }
        if (power==29) {
            powerLevel=3;
        }
        frameSize=smartaudioFrameSetPower(&request,powerLevel);
    }
    // hardcoded protocol version 2.1
    if (backend->version()==2) {
        if (power==14) {
            powerLevel=14;
        }
        if (power==23) {
            powerLevel=23;
        }
        if (power==27) {
            powerLevel=27;
        }
        if (power==29) {
            powerLevel=29;
        }
        frameSize=smartaudioFrameSetPower(&request,powerLevel|=128);
    }

    //poweLevel activating the MSB to represent the command

    backend->send_request(request,frameSize);

}


//Takes the backend reading data and set it into the frontend state datas structure
void AP_SmartAudio::update_readings()
{
    vtx_state.current_channel=backend->channel();
    vtx_state.current_band=backend->band();
    vtx_state.current_frequency=backend->frecuency();
    vtx_state.current_power_db=backend->power();
    vtx_state.current_power_mw=backend->power();
    vtx_state.is_in_range_pit_mode=backend->is_in_range_pit_mode();
    vtx_state.is_out_range_pit_mode=backend->is_out_range_pit_mode();
    vtx_state.is_locked=!backend->is_unlocked();
    vtx_state.is_pit_mode_disabled=backend->is_in_pit_mode_disabled();
}

// Main query method in the frontend to get the readings from the vtx, prior to get it, at least one get_settings must be called and well responsed
AP_SmartAudio::SmartAudioState AP_SmartAudio::get_readings()
{
    SmartAudioState currentState;
    if (backend->has_data()) {
        update_readings();
        memcpy(&vtx_state,&currentState,sizeof(vtx_state));
    }
    return currentState;

}