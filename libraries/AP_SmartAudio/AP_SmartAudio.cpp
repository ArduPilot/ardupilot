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

    // @Param: ENABLED
    // @DisplayName: Enable SmartAudio protocol on VTX
    // @Description: Enable SmartAudio protocol on VTX
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLED",     1, AP_SmartAudio::configured_smartaudio_params, enabled, 0,AP_PARAM_FLAG_ENABLE),

    // @Param: VERSION
    // @DisplayName: SmartAudio version
    // @Description: SmartAudio version
    // @Values: 0: Version 1,1:Version 2, 3: Version 2.1
    // @User: Advanced
    AP_GROUPINFO("VERSION",     2, AP_SmartAudio::configured_smartaudio_params, protocol_version, 0),

    // @Param: DEFAULTS
    // @DisplayName: VTX will be configured with VTX params defined by default.
    // @Description: VTX will be configured with VTX params defined by default.
    // @Values: 0: Disabled,1: Enabled
    // @User: Advanced
    AP_GROUPINFO("DEFAULTS",     3, AP_SmartAudio::configured_smartaudio_params, setup_defaults, 0),

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

    if(smartaudio_params.enabled.get()==0){
        debug("SmartAudio protocol it's not active");
        return false;
    }

    // init uart
    _port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_SmartAudio, 0);
    if(_port!=nullptr){

          if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_SmartAudio::loop, void),
                                          "SmartAudio",
                                          1024, AP_HAL::Scheduler::PRIORITY_IO, 60)) {
            return false;
            }

        // setup port options
        _port->set_stop_bits(AP_SERIALMANAGER_SMARTAUDIO_STOP_BITS);
        _port->set_flow_control(AP_SERIALMANAGER_SMARTAUDIO_FLOW_CONTROL);
        _port->set_options(AP_SERIALMANAGER_SMARTAUDIO_OPTIONS);

        return true;
    }
    return false;
}

void AP_SmartAudio::loop(){

    // initialise uart (this must be called from within tick b/c the UART begin must be called from the same thread as it is used from)
    _port->begin(AP_SERIALMANAGER_SMARTAUDIO_BAUD, AP_SERIALMANAGER_SMARTAUDIO_BUFSIZE_RX, AP_SERIALMANAGER_SMARTAUDIO_BUFSIZE_TX);

    // initialize AP_VideoTx settings
        request_settings();
        if(!smartaudio_params.setup_defaults){
            get_readings(AP_VideoTX::get_singleton());
        }else{
            // setup values defined in vtx params
            set_frequency(AP_VideoTX::get_frequency_mhz(AP_VideoTX::get_singleton()->get_band()
            ,AP_VideoTX::get_singleton()->get_channel()),AP_VideoTX::get_singleton()->get_options()==0);
        }

     while (true) {
        // now time to control loop switching
        uint32_t _now=AP_HAL::millis();
        // when one packet is processed in the iteration will be true.
        bool packet_processed=false;
        monkey_testing();
        // setup a queue polling delay of 20 ms.
        hal.scheduler->delay(20);
        // command to process
        packet _current_command;
        // printf(" time-passed-since-last-request:%d %20s %d ",now-last_request_sended_at,"packet-processed:",packet_processed);

        // Proccess response in the next 200 milis from the request are sent.
        if(_now-last_request_sended_at<=200 && is_waiting_response){
            printf("%20s %d and %d ms more","I'M WAITING RESPONSE SINCE",last_request_sended_at,200-(_now-last_request_sended_at));
            // allocate response buffer
            uint8_t _response_buffer[AP_SERIALMANAGER_SMARTAUDIO_BUFSIZE_RX];
            // setup to zero because the
            uint8_t _inline_buffer_length=0;
            // setup sheduler delay to 20 ms again after response processes
            read_response(_response_buffer,_inline_buffer_length);
            hal.scheduler->delay(20);
            // prevent to proccess any queued request from the ring buffer
            packet_processed=true;
        }

        // when pending request and last request sended is timeout, take another packet to send
        if(!packet_processed && requests_queue.pop(_current_command)){
            send_request(_current_command.frame,_current_command.frame_size);
            _current_command.sended_at=AP_HAL::millis();
            last_request_sended_at=AP_HAL::millis();
            is_waiting_response=true;
            // spec says: The Unify Pro response frame is usually send <100ms after a frame is successfully received from the host MCU
            printf(" delay: %d",100);
            hal.scheduler->delay(100);
        }
     }
}

/**
 * Sends an SmartAudio Command to the vtx, waits response on the update event
 * @param frameBuffer frameBuffer to send over the wire
 * @param size  size of the framebuffer wich needs to be sended
 */
void AP_SmartAudio::send_request(smartaudioFrame_t requestFrame,uint8_t size)
{

    if (size<=0) {
        debug("%s HW: %s",TAG,"CANNOT SEND REQUEST, REQUEST IS EMPTY");
        printf("ERROR - %s HW: %s",TAG,"CANNOT SEND REQUEST, REQUEST IS EMPTY");
    }
    if (_port==nullptr) {
        debug("%s HW: %s",TAG,"CANNOT SEND REQUEST, UART NOT CONNECTED");
        printf("ERROR - %s HW: %s",TAG,"CANNOT SEND REQUEST, UART NOT CONNECTED");
        return;
    }

    uint8_t *request=reinterpret_cast<uint8_t*>(&requestFrame.u8RequestFrame);
    // pull line low
    _port->write((uint8_t)0);
    // write request
    for (int i = 0; i < size; ++i) {
        _port->write(request[i]);
    }
    print_bytes_to_hex_string(request,size);
}

/**
 * Reads the response from vtx in the wire
 * - response_buffer, response buffer to fill in
 * - inline_buffer_length , used to passthrought the response lenght in case the response where splitted
 **/
void AP_SmartAudio::read_response(uint8_t *response_buffer,uint8_t inline_buffer_length)
{
    int16_t _incoming_bytes_count = _port->available();
    uint8_t _response_header_size= sizeof(smartaudioFrameHeader_t);

    // check if it is a response in the wire
    if (_incoming_bytes_count < 1) {
        debug("%s HW: %s",TAG,"EMPTY WIRE");
        printf(" WARNING - %s HW: %s",TAG,"EMPTY WIRE");
        return;
    }


    for (int i = 0; i < _incoming_bytes_count; ++i) {
        uint8_t _response_in_bytes = _port->read();

        if ((inline_buffer_length == 0 && _response_in_bytes != SMARTAUDIO_SYNC_BYTE)
            || (inline_buffer_length == 1 && _response_in_bytes != SMARTAUDIO_HEADER_BYTE)) {
            inline_buffer_length = 0;
        } else if (inline_buffer_length < AP_SERIALMANAGER_SMARTAUDIO_BUFSIZE_RX) {
            response_buffer[inline_buffer_length++] = _response_in_bytes;
        }
    }

    //last_io_time = AP_HAL::millis();

    if (inline_buffer_length < _response_header_size) {
        is_waiting_response=false;
        return;
    }

    uint8_t payload_len = response_buffer[3];

    if (inline_buffer_length < payload_len + _response_header_size) {
        // Not all response bytes received yet, splitted response is incoming
        return;
    } else {
        inline_buffer_length = payload_len + _response_header_size;
    }
    is_waiting_response=false;

    uint8_t crc = response_buffer[inline_buffer_length-1];
    if (crc != crc8_dvb_s2(*response_buffer, inline_buffer_length-1)) {
        // crc missmatch
        debug("%s HW: %s",TAG,"BAD CRC CHECK IN RESPONSE");
        printf(" ERROR - %s HW: %s",TAG,"BAD CRC CHECK IN RESPONSE");

    } else {
        parse_frame_response(response_buffer);
    }
    response_buffer=nullptr;
}

/**
 * This method parse the frame response and match the current device configuration  *
 * */
bool AP_SmartAudio::parse_frame_response(const uint8_t *buffer)
{
    if (buffer!=nullptr) {
        smartaudioSettings_t _vtx_settings;
        if(!smartaudioParseResponseBuffer(&_vtx_settings,buffer)){
            return false;
        }

        // update bands and channels accordily when frequency changes
        if(_vtx_settings.frequency_updated && _vtx_settings.userFrequencyMode){
            AP_VideoTX::VideoBand video_band;
            AP_VideoTX::get_band_and_channel(_vtx_settings.frequency,video_band,_vtx_settings.channel);
            _vtx_settings.band=(int)video_band;
            // update channel index to start in 1 not at 0
            _vtx_settings.channel++;
        }
        // update band and frecuency accorly when channel updates
        if(_vtx_settings.channel_updated && !_vtx_settings.userFrequencyMode){
            _vtx_settings.frequency=AP_VideoTX::get_frequency_mhz(_vtx_settings.band,_vtx_settings.channel);
        }


        if(vtx_states_queue.is_empty()){
            vtx_states_queue.push_force(_vtx_settings);
        }else{
            vtx_states_queue.push_force(_vtx_settings);
            // advance to last element pushed
            vtx_states_queue.advance(1);
        }


        return true;
    }

    debug("%s HW: %s",TAG,"EMPTY RESPONSE");
    printf("%s HW: %s",TAG,"EMPTY RESPONSE");

    return false;
}

// Main query method in the frontend to get the readings from the vtx.
bool AP_SmartAudio::get_readings(AP_VideoTX *vtx_dest)
{
    // take the first register from the output buffer
   smartaudioSettings_t _current_state;

    // peek from buffer
   vtx_states_queue.peek(&_current_state,1);

   // setting frecuency
    vtx_dest->set_frequency_mhz(_current_state.frequency);
    // set channel
    vtx_dest->set_band((AP_VideoTX::VideoBand)((int)_current_state.band));
    // setting channel 0 -> 40
    vtx_dest->set_channel(_current_state.channel);

   // define vtx options TODO: Review this after define policies

   //  pitmode disabled
   if(_current_state.pitmodeDisabled){
       vtx_dest->set_options(0);
   }
   // pitmode types
   if(_current_state.pitmodeInRangeActive){
       vtx_dest->set_options(1);
   }else{
         if(_current_state.pitmodeOutRangeActive){
             vtx_dest->set_options(2);
         }
   }
   // pitmode is enabled after boot if using this way of disabling it
   if(_current_state.pitmodeDisabled && (_current_state.pitmodeInRangeActive || _current_state.pitmodeOutRangeActive)){
       vtx_dest->set_options(3);
   }

   // locking status
   vtx_dest->set_locking(!_current_state.unlocked);


   // transform power levels in dbm
   uint8_t _power_in_dbm=0;

   // spec 2.1 power-levels in dbm
   vtx_dest->set_power_dbm(_current_state.power);

   // specs 1 and 2 power-levels need transformation to dbm power
   if(_current_state.version!=SMARTAUDIO_SPEC_PROTOCOL_v21){
       // search in power tables
        if(_get_power_in_dbm_from_vtx_power_level(_current_state.power,_current_state.version,_power_in_dbm)){
            vtx_dest->set_power_dbm(_power_in_dbm);
        }else{
            _get_power_in_dbm_from_vtx_power_level(POWER_LEVELS[_current_state.version][0],_current_state.version,_power_in_dbm);
            vtx_dest->set_power_dbm(_power_in_dbm);
        }
   }

   return !vtx_states_queue.is_empty();
}

// returns the dbs associated by power-level input
bool AP_SmartAudio::_get_power_in_dbm_from_vtx_power_level(uint8_t power_level, uint8_t& protocol_version, uint8_t& power_in_dbm)
{

    for (uint8_t j = 0; j < 4; j++) {
        if (POWER_LEVELS[protocol_version][j] == power_level) {
            switch (j)
            {
            case 0:
                power_in_dbm=14;
                break;
            case 1:
                power_in_dbm=23;
                break;
            case 2:
                power_in_dbm=27;
                break;
            case 3:
                power_in_dbm=29;
                break;
            default:
                return false;
            }

            return true;
        }
    }
    return false;
}


/**
 * Sends get settings command.
 * */
void AP_SmartAudio::request_settings()
{
    printf("%80s::request_settings()\t",TAG);
    smartaudioFrame_t request;
    uint8_t frame_size=smartaudioFrameGetSettings(&request);
    packet command;
    command.frame=request;
    command.frame_size=frame_size;
    requests_queue.push_force(command);
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
     // take the first register from the output buffer
   smartaudioSettings_t _current_state;

    // peek from buffer
   vtx_states_queue.peek(&_current_state,1);

    printf("%80s::set_operation_mode(%d,%d)\t",TAG,mode,locked);
    if (_current_state.version<2) {
        debug("%s HW: %s",TAG,"Device can't change operation mode. Spec protocol not supported");
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
        debug("%s HW: %s",TAG,"Operation mode not supported");
        return;
    };
    smartaudioFrame_t request;
    uint8_t frame_size=smartaudioFrameSetOperationMode(&request,&settings);
    packet command;
    command.frame=request;
    command.frame_size=frame_size;
    requests_queue.push_force(command);
}


/**
     * Sets the frecuency to transmit in the vtx.
     * When isPitModeFreq active the frec will be set to be used when in pitmode (in range)
     */
void AP_SmartAudio::set_frequency(uint16_t frecuency,bool isPitModeFreq)
{
    printf("%80s::set_frequency(%d,%d)\t",TAG,frecuency,isPitModeFreq);
    smartaudioFrame_t request;
    uint8_t frame_size=smartaudioFrameSetFrequency(&request,frecuency,isPitModeFreq);
    packet command;
    command.frame=request;
    command.frame_size=frame_size;
    requests_queue.push_force(command);
}

/**
    * Set the power to the vtx device returning the dbm.
    *
    *
    *
    */
void AP_SmartAudio::set_power_dbm(uint8_t power)
{
      // take the first register from the output buffer
   smartaudioSettings_t _current_state;

    // peek from buffer
   vtx_states_queue.peek(&_current_state,1);
    smartaudioFrame_t request;
    uint8_t frame_size=0;

    printf("%80s::set_power_dbm(%d)\t",TAG,power);
    uint16_t powerLevel=0x00;
    if (_current_state.version!=SMARTAUDIO_SPEC_PROTOCOL_v1 && _current_state.version!=SMARTAUDIO_SPEC_PROTOCOL_v2 && _current_state.version!=SMARTAUDIO_SPEC_PROTOCOL_v21) {
        return;
    }

    // version 1
    if (_current_state.version==SMARTAUDIO_SPEC_PROTOCOL_v1) {
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
        frame_size=smartaudioFrameSetPower(&request,powerLevel);
    }
    // hardcoded protocol version 2
    if (_current_state.version==SMARTAUDIO_SPEC_PROTOCOL_v2) {
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
        frame_size=smartaudioFrameSetPower(&request,powerLevel);
    }
    // hardcoded protocol version 2.1
    if (_current_state.version==SMARTAUDIO_SPEC_PROTOCOL_v21) {
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
        frame_size=smartaudioFrameSetPower(&request,powerLevel|=128);
    }

   packet command;
   command.frame=request;
   command.frame_size=frame_size;
   requests_queue.push_force(command);

}

