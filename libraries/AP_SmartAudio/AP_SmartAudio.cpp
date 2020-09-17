#pragma once
/**
 *
    _____            .___   __________.__.__          __
  /  _  \_______  __| _/_ _\______   \__|  |   _____/  |_
 /  /_\  \_  __ \/ __ |  |  \     ___/  |  |  /  _ \   __\
/    |    \  | \/ /_/ |  |  /    |   |  |  |_(  <_> )  |
\____|__  /__|  \____ |____/|____|   |__|____/\____/|__|
        \/           \/
*
*
* */


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
AP_VideoTX *ap_video_tx=AP_VideoTX::get_singleton();

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

     while (true) {

        // now time to control loop switching
        uint32_t _now=AP_HAL::millis();

        // when one packet is processed in the iteration will be true.
        bool packet_processed=false;

        // setup a queue polling delay of 20 ms.
        hal.scheduler->delay(20);

        // command to process
        packet _current_command;

        // Proccess response in the next 200 milis from the request are sent.
        if(_now-last_request_sended_at>100 && _now-last_request_sended_at<=1200 && is_waiting_response){
            printf("\n%20s %d and %d ms more","I'M WAITING RESPONSE SINCE",last_request_sended_at,1200-(_now-last_request_sended_at));
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

        update(false);
     }
}

// updates the smartaudio state in sync whith AP_VideoTX
bool AP_SmartAudio::update(bool force)
{
    if(vtx_states_queue.available()==0){
        if(!hal.util->get_soft_armed()) {
            request_settings();
        }
        return false;
    }

    if(_get_current_state()->userFrequencyMode && _get_current_state()->frequency!=ap_video_tx->get_frequency_mhz()){
        set_frequency(ap_video_tx->get_frequency_mhz(),!_get_current_state()->pitmodeDisabled);
    }

    if(!_get_current_state()->userFrequencyMode && _get_current_state()->channel!=ap_video_tx->get_channel() && _get_current_state()->band!=ap_video_tx->get_band()){
        set_channel((ap_video_tx->get_band()*8)+ap_video_tx->get_channel());
    }

    if(_get_current_state()->version==SMARTAUDIO_SPEC_PROTOCOL_v21 && _get_power_in_mw_from_dbm(_get_current_state()->power_in_dbm)!= ap_video_tx->get_power_mw()){
        set_power_mw(ap_video_tx->get_power_mw());
    }

    if(_get_current_state()->version!=SMARTAUDIO_SPEC_PROTOCOL_v21
    && ap_video_tx->get_power_mw()!=_get_power_in_mw_from_dbm(_get_power_in_dbm_from_vtx_power_level(_get_current_state()->power,_get_current_state()->version))){
        set_power_mw(ap_video_tx->get_power_mw());
    }

    if( (ap_video_tx->get_options()==0 && !_get_current_state()->pitmodeDisabled) || (ap_video_tx->get_options()==1 && _get_current_state()->pitmodeDisabled) ){
        uint8_t operation_mode= 0x00;
        operation_mode = _get_current_state()->pitmodeInRangeActive && 1<<0;
        operation_mode = _get_current_state()->pitmodeInRangeActive && 1<<1;
        operation_mode = ap_video_tx->get_options()==0 && 1<<2;
        operation_mode = _get_current_state()->unlocked==0 && 1<<3;
        set_operation_mode(operation_mode);
    }

    return true;
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
         printf("\n REQ-SEND bytes:%02X",request[i]);
    }
    printf("-------------->");print_bytes_to_hex_string(request,size);
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

    printf("\n READ RESPONSE incoming_bytes_count:%d",_incoming_bytes_count);
    for (int i = 0; i < _incoming_bytes_count; ++i) {
        uint8_t _response_in_bytes = _port->read();
        printf("\n READ RESPONSE _response_in_bytes:%02X",_response_in_bytes);
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
 *
 * This method only changes the internal state vtx buffer. Using response parser and then updating the internal buffer.
 * */
bool AP_SmartAudio::parse_frame_response(const uint8_t *buffer)
{
    if (buffer!=nullptr) {

        smartaudioSettings_t _vtx_settings;
        // process response buffer
        if(!smartaudioParseResponseBuffer(&_vtx_settings,buffer)){
            printf("%20s","Unparseable buffer response");
             print_bytes_to_hex_string(const_cast<uint8_t*>(buffer),sizeof(buffer));
            return false;
        }

        // if partial updates because setters method to vtx.
        if(_vtx_settings.update_flags!=0x0F){
            smartaudioSettings_t _current_vtx_settings;

            // take the current vtx info
            vtx_states_queue.peek(&_current_vtx_settings,1);

            // get update flags from the parsed response_buffer
            _current_vtx_settings.update_flags=_vtx_settings.update_flags;


            if( _vtx_settings.update_flags & (1 << 0)){ // freq has changed
                _current_vtx_settings.frequency=_vtx_settings.frequency;
                _current_vtx_settings.userFrequencyMode=_vtx_settings.userFrequencyMode;
            }

            if( _vtx_settings.update_flags & (1 << 1)){ // channel has changed
                _current_vtx_settings.band=_vtx_settings.channel/8;
                _current_vtx_settings.channel=_vtx_settings.channel%8;
            }


            if( _vtx_settings.update_flags & (1 << 2)){ // power has changed
                _current_vtx_settings.power=_vtx_settings.power;

                if(_current_vtx_settings.version==SMARTAUDIO_SPEC_PROTOCOL_v21){
                    _current_vtx_settings.power_in_dbm=_vtx_settings.power;
                }

            }


            if( _vtx_settings.update_flags & (1 << 3)){ // mode has changed
                _current_vtx_settings.pitmodeDisabled=_vtx_settings.pitmodeDisabled;
                _current_vtx_settings.pitmodeInRangeActive=_vtx_settings.pitmodeInRangeActive;
                _current_vtx_settings.pitmodeOutRangeActive=_vtx_settings.pitmodeOutRangeActive;
                _current_vtx_settings.unlocked=_vtx_settings.unlocked;
            }

            _vtx_settings=_current_vtx_settings;

        }

        // update bands and channels accordily when frequency changes
        if((_vtx_settings.update_flags & 1<<0) && _vtx_settings.userFrequencyMode){

            AP_VideoTX::VideoBand video_band;

            AP_VideoTX::get_band_and_channel(_vtx_settings.frequency,video_band,_vtx_settings.channel);

            _vtx_settings.band=(int)video_band;
        }
        // update band and frecuency accorly when channel updates
        if( (_vtx_settings.update_flags & 1<<1) && !_vtx_settings.userFrequencyMode){

            _vtx_settings.frequency=AP_VideoTX::get_frequency_mhz(_vtx_settings.band,_vtx_settings.channel);

        }


        // reset vtx_settings_change_control variables
        _vtx_settings.update_flags=0x00;

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

/**
 *  This method parses the internal vtx current state stored in internal buffer, transforming the state values into the AP_VideoTx passed as argument.
 *   @DependsOn: AP_VideoTX, AP_VideoTX::VideoBand
 * */
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
   // setup default value for options
    vtx_dest->set_options(0);

      // pitmode enabled
   if(_current_state.pitmodeOutRangeActive ||  _current_state.pitmodeInRangeActive || !_current_state.pitmodeDisabled){
       vtx_dest->set_options(1);
   }
    // pitmode disabled only by this option
   if(_current_state.pitmodeDisabled){
       vtx_dest->set_options(0);
   }

   // locking status
   vtx_dest->set_locking(_current_state.unlocked==0?1:0);

   // spec 2.1 power-levels in dbm
   vtx_dest->set_power_dbm(_current_state.power_in_dbm);

   // specs 1 and 2 power-levels need transformation to dbm power
   if(_current_state.version!=SMARTAUDIO_SPEC_PROTOCOL_v21){
       // search in power tables
        if(_get_power_in_dbm_from_vtx_power_level(_current_state.power,_current_state.version,_current_state.power_in_dbm)){
            vtx_dest->set_power_dbm(_current_state.power_in_dbm);
        }else{
            _get_power_in_dbm_from_vtx_power_level(POWER_LEVELS[_current_state.version][0],_current_state.version,_current_state.power_in_dbm);
            vtx_dest->set_power_dbm(_current_state.power_in_dbm);
        }
   }

   return !vtx_states_queue.is_empty();
}

// utility method to get power in dbm mapping to power levels
uint8_t AP_SmartAudio::_get_power_in_dbm_from_vtx_power_level(uint8_t power_level, uint8_t& protocol_version)
{
    uint8_t power_in_dbm=0;
    return _get_power_in_dbm_from_vtx_power_level(power_level,protocol_version,power_in_dbm);
}
// returns the dbs associated by power-level input
uint8_t AP_SmartAudio::_get_power_in_dbm_from_vtx_power_level(uint8_t power_level, uint8_t& protocol_version, uint8_t& power_in_dbm)
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
            }
        }
    }
 return power_in_dbm;
}


/**
 * Sends get settings command.
 * */
void AP_SmartAudio::request_settings()
{
    printf("\n%20s::request_settings()\t",TAG);
    smartaudioFrame_t request;
    uint8_t frame_size=smartaudioFrameGetSettings(&request);
    packet command;
    command.frame=request;
    command.frame_size=frame_size;
    requests_queue.push_force(command);
}


void AP_SmartAudio::set_operation_mode(uint8_t mode){
     // take the first register from the output buffer
   smartaudioSettings_t _current_state;

    // peek from buffer
   vtx_states_queue.peek(&_current_state,1);

    printf("\n%80s::set_operation_mode(%02X)\t",TAG,mode);
    if (_current_state.version<2) {
        debug("%s HW: %s",TAG,"Device can't change operation mode. Spec protocol not supported");
        return;
    }
    smartaudioSettings_t settings;
    settings.pitmodeInRangeActive=mode & 1<<0;
    settings.pitmodeOutRangeActive=mode & 1<<1;
    settings.pitmodeDisabled=mode & 1<<2;
    settings.unlocked=(mode & 1<<3)!=0?1:0;

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

// enqueue a set channel request
void AP_SmartAudio::set_channel(uint8_t chan_idx){
        printf("\n%80s::set_channel(%d)\t",TAG,chan_idx);
        smartaudioFrame_t request;
        uint8_t frame_size=smartaudioFrameSetChannel(&request,chan_idx);
         packet command;
        command.frame=request;
        command.frame_size=frame_size;
        requests_queue.push_force(command);
}

/**
 * Request pitMode Frequency setted into the vtx hardware
 * */
void AP_SmartAudio::get_pit_mode_frequency(){
    printf("%80s::","get_pit_mode_frequency()\t");
    smartaudioFrame_t request;
    uint8_t frame_size=smartaudioFrameGetPitmodeFrequency(&request);
    packet command;
    command.frame=request;
    command.frame_size=frame_size;
    requests_queue.push_force(command);
}


// send vtx request to set power defined in dbm
void AP_SmartAudio::set_power_dbm(uint8_t power)
{
      // take the first register from the output buffer
   smartaudioSettings_t _current_state;

    // peek from buffer
   vtx_states_queue.peek(&_current_state,1);
    smartaudioFrame_t request;
    uint8_t frame_size=0;

    printf("\n%80s::set_power_dbm(%d)\t",TAG,power);
    frame_size=smartaudioFrameSetPower(&request,_get_power_level_from_dbm(_current_state.version,power));
     packet command;
     command.frame=request;
     command.frame_size=frame_size;
     requests_queue.push_force(command);
}

// send vtx request to set power defined in mw
void AP_SmartAudio::set_power_mw(uint16_t power_mw){
    set_power_dbm(_get_power_in_dbm_from_mw(power_mw));
    return;
}

// returns the power_level applicable when request set power, version 2.1 return the MSB power bit masked at 1
uint8_t AP_SmartAudio::_get_power_level_from_dbm(uint8_t sma_version,uint8_t power){
    uint16_t powerLevel=0x00;
    if (sma_version!=SMARTAUDIO_SPEC_PROTOCOL_v1 && sma_version!=SMARTAUDIO_SPEC_PROTOCOL_v2 && sma_version!=SMARTAUDIO_SPEC_PROTOCOL_v21) {
        return 0;
    }

    // version 1
    if (sma_version==SMARTAUDIO_SPEC_PROTOCOL_v1) {
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
       return powerLevel;
    }
    // hardcoded protocol version 2
    if (sma_version==SMARTAUDIO_SPEC_PROTOCOL_v2) {
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
       return powerLevel;
    }
    // hardcoded protocol version 2.1
    if (sma_version==SMARTAUDIO_SPEC_PROTOCOL_v21) {
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
        return powerLevel|=128;
    }
     return 0;

}




//          __________        __         ___________.__  .__       .__     __
//          \______   \ _____/  |______  \_   _____/|  | |__| ____ |  |___/  |_
//          |    |  _// __ \   __\__  \  |    __)  |  | |  |/ ___\|  |  \   __\
//          |    |   \  ___/|  |  / __ \_|     \   |  |_|  / /_/  >   Y  \  |
//          |______  /\___  >__| (____  /\___  /   |____/__\___  /|___|  /__|
//          \/     \/          \/     \/           /_____/      \/


/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

static void smartaudioFrameInit(const uint8_t command, smartaudioFrameHeader_t *header, const uint8_t payloadLength)
{
    header->syncByte = SMARTAUDIO_SYNC_BYTE;
    header->headerByte= SMARTAUDIO_HEADER_BYTE;
    header->length = payloadLength;
    header->command = command;

}

static void smartaudioUnpackOperationMode(smartaudioSettings_t *settings, const uint8_t operationMode, const bool settingsResponse)
{
    if (settingsResponse) {
        // operation mode bit order is different between 'Get Settings' and 'Set Mode' responses.
        settings->userFrequencyMode = operationMode & 0x01;
        settings->pitmodeDisabled = operationMode & 0x02;
        settings->pitmodeInRangeActive = operationMode & 0x04;
        settings->pitmodeOutRangeActive = operationMode & 0x08;
        settings->unlocked = operationMode & 0x10;
    } else {
        settings->pitmodeInRangeActive = operationMode & 0x01;
        settings->pitmodeOutRangeActive = operationMode & 0x02;
        settings->pitmodeDisabled = operationMode & 0x04;
        settings->unlocked = operationMode & 0x08;
    }
}

static void smartaudioUnpackFrequency(smartaudioSettings_t *settings, const uint16_t frequency)
{
    if (applyBigEndian16(frequency) & SMARTAUDIO_GET_PITMODE_FREQ) {
        settings->pitmodeFrequency = applyBigEndian16(frequency);
    } else {
        settings->frequency = applyBigEndian16(frequency);
    }
}

static void smartaudioUnpackSettings(smartaudioSettings_t *settings, const smartaudioSettingsResponseFrame_t *frame)
{
    settings->channel = frame->channel;
    settings->power = frame->power;
    smartaudioUnpackFrequency(settings, frame->frequency);
    smartaudioUnpackOperationMode(settings, frame->operationMode, true);
}

static void smartaudioUnpackSettings(smartaudioSettings_t *settings, const smartaudioSettingsExtendedResponseFrame_t *frame)
{
    settings->channel = frame->channel;
    settings->power = frame->power;
    settings->power_in_dbm=frame->power_dbm;
    smartaudioUnpackFrequency(settings, frame->frequency);
    smartaudioUnpackOperationMode(settings, frame->operationMode, true);
}

static uint8_t smartaudioPackOperationMode(const smartaudioSettings_t *settings)
{
    uint8_t operationMode = 0;
    operationMode |= settings->pitmodeInRangeActive << 0;
    operationMode |= settings->pitmodeOutRangeActive << 1;
    operationMode |= settings->pitmodeDisabled << 2;
    operationMode |= settings->unlocked << 3;
    return operationMode;
}



size_t smartaudioFrameGetSettings(smartaudioFrame_t *smartaudioFrame)
{
    smartaudioCommandOnlyFrame_t *frame = (smartaudioCommandOnlyFrame_t *)smartaudioFrame;
    smartaudioFrameInit(SMARTAUDIO_CMD_GET_SETTINGS, &frame->header, 0);
    frame->crc = crc8_dvb_s2_update(0, frame, sizeof(smartaudioCommandOnlyFrame_t) - sizeof(frame->crc));
    //frame->crc=crc8_dvb_s2(*(const uint8_t *)frame, sizeof(smartaudioCommandOnlyFrame_t) - sizeof(frame->crc));
    return sizeof(smartaudioCommandOnlyFrame_t);
}

size_t smartaudioFrameGetPitmodeFrequency(smartaudioFrame_t *smartaudioFrame)
{
    smartaudioU16Frame_t *frame = (smartaudioU16Frame_t *)smartaudioFrame;
    smartaudioFrameInit(SMARTAUDIO_CMD_SET_FREQUENCY, &frame->header, sizeof(frame->payload));
    frame->payload = SMARTAUDIO_GET_PITMODE_FREQ;
    frame->crc = crc8_dvb_s2_update(0, frame, sizeof(smartaudioU16Frame_t) - sizeof(frame->crc));
    //frame->crc = crc8_dvb_s2(*(const uint8_t *)frame, sizeof(smartaudioU16Frame_t) - sizeof(frame->crc));
    return sizeof(smartaudioU16Frame_t);
}

size_t smartaudioFrameSetPower(smartaudioFrame_t *smartaudioFrame, const uint8_t power)
{
    smartaudioU8Frame_t *frame = (smartaudioU8Frame_t *)smartaudioFrame;
    smartaudioFrameInit(SMARTAUDIO_CMD_SET_POWER, &frame->header, sizeof(frame->payload));
    frame->payload = power;
    frame->crc = crc8_dvb_s2_update(0, frame, sizeof(smartaudioU8Frame_t) - sizeof(frame->crc));
    //frame->crc = crc8_dvb_s2(*(const uint8_t *)frame, sizeof(smartaudioU8Frame_t) - sizeof(frame->crc));
    return sizeof(smartaudioU8Frame_t);
}

size_t smartaudioFrameSetBandChannel(smartaudioFrame_t *smartaudioFrame, const uint8_t band, const uint8_t channel)
{
    smartaudioU8Frame_t *frame = (smartaudioU8Frame_t *)smartaudioFrame;
    smartaudioFrameInit(SMARTAUDIO_CMD_SET_CHANNEL, &frame->header, sizeof(frame->payload));
    frame->payload = SMARTAUDIO_BANDCHAN_TO_INDEX(band, channel);
    frame->crc = crc8_dvb_s2_update(0, frame, sizeof(smartaudioU8Frame_t) - sizeof(frame->crc));
    //frame->crc = crc8_dvb_s2( *(const uint8_t *)frame, sizeof(smartaudioU8Frame_t) - sizeof(frame->crc));
    return sizeof(smartaudioU8Frame_t);
}

size_t smartaudioFrameSetChannel(smartaudioFrame_t *smartaudioFrame, const uint8_t channel)
{
    smartaudioU8Frame_t *frame = (smartaudioU8Frame_t *)smartaudioFrame;
    smartaudioFrameInit(SMARTAUDIO_CMD_SET_CHANNEL, &frame->header, sizeof(frame->payload));
    frame->payload = channel;
    frame->crc = crc8_dvb_s2_update(0, frame, sizeof(smartaudioU8Frame_t) - sizeof(frame->crc));
    //frame->crc = crc8_dvb_s2( *(const uint8_t *)frame, sizeof(smartaudioU8Frame_t) - sizeof(frame->crc));
    return sizeof(smartaudioU8Frame_t);
}

size_t smartaudioFrameSetFrequency(smartaudioFrame_t *smartaudioFrame, const uint16_t frequency, const bool pitmodeFrequency)
{
    smartaudioU16Frame_t *frame = (smartaudioU16Frame_t *)smartaudioFrame;
    smartaudioFrameInit(SMARTAUDIO_CMD_SET_FREQUENCY, &frame->header, sizeof(frame->payload));
    frame->payload = applyBigEndian16(frequency | (pitmodeFrequency ? SMARTAUDIO_SET_PITMODE_FREQ : 0x00));
    frame->crc = crc8_dvb_s2_update(0, frame, sizeof(smartaudioU16Frame_t) - sizeof(frame->crc));
    return sizeof(smartaudioU16Frame_t);
}

/** Addition because the define macro seems that's not working. TODO: Refactor this using AP routines.*/
uint16_t applyBigEndian16(uint16_t bytes)
{
    return (bytes << 8) | ((bytes >> 8) & 0xFF);
}

size_t smartaudioFrameSetOperationMode(smartaudioFrame_t *smartaudioFrame, const smartaudioSettings_t *settings)
{
    smartaudioU8Frame_t *frame = (smartaudioU8Frame_t *)smartaudioFrame;
    smartaudioFrameInit(SMARTAUDIO_CMD_SET_MODE, &frame->header, sizeof(frame->payload));
    frame->payload = smartaudioPackOperationMode(settings);
    frame->crc = crc8_dvb_s2_update(0, frame, sizeof(smartaudioU8Frame_t) - sizeof(frame->crc));
    return sizeof(smartaudioU8Frame_t);
}

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

bool smartaudioParseResponseBuffer(smartaudioSettings_t *settings, const uint8_t *buffer)
{
    print_bytes_to_hex_string(const_cast <uint8_t*>(buffer),16);

    const smartaudioFrameHeader_t *header = (const smartaudioFrameHeader_t *)buffer;
    const uint8_t fullFrameLength = sizeof(smartaudioFrameHeader_t) + header->length;
    const uint8_t headerPayloadLength = fullFrameLength - 1; // subtract crc byte from length
    const uint8_t *startPtr = buffer + 2;
    const uint8_t *endPtr = buffer + headerPayloadLength;
    printf(" CRC BYTE:%02X",*endPtr);
    printf(" CRC CHECK:%02X\n",crc8_dvb_s2_update(0x00,startPtr, headerPayloadLength-2));
    if (crc8_dvb_s2_update(0x00, startPtr, headerPayloadLength-2)!=*(endPtr) || header->headerByte != SMARTAUDIO_HEADER_BYTE || header->syncByte!=SMARTAUDIO_SYNC_BYTE) {
        return false;
    }

    switch (header->command) {
    case SMARTAUDIO_RSP_GET_SETTINGS_V1: {
        const smartaudioSettingsResponseFrame_t *resp = (const smartaudioSettingsResponseFrame_t *)buffer;
        settings->version = SMARTAUDIO_SPEC_PROTOCOL_v1;
        smartaudioUnpackSettings(settings, resp);
        settings->update_flags=0x0F;
    }
    break;
    case SMARTAUDIO_RSP_GET_SETTINGS_V2: {
        const smartaudioSettingsResponseFrame_t *resp = (const smartaudioSettingsResponseFrame_t *)buffer;
        settings->version = SMARTAUDIO_SPEC_PROTOCOL_v2;
        smartaudioUnpackSettings(settings, resp);
        settings->update_flags=0x0F;
    }
    break;
    case SMARTAUDIO_RSP_GET_SETTINGS_V21: {
        const smartaudioSettingsExtendedResponseFrame_t *resp = (const smartaudioSettingsExtendedResponseFrame_t *)buffer;
        settings->version = SMARTAUDIO_SPEC_PROTOCOL_v21;
        smartaudioUnpackSettings(settings, resp);
        settings->update_flags=0x0F;
    }
    break;
    case SMARTAUDIO_RSP_SET_FREQUENCY: {
        const smartaudioU16ResponseFrame_t *resp = (const smartaudioU16ResponseFrame_t *)buffer;
        smartaudioUnpackFrequency(settings, resp->payload);
       settings->update_flags=0x01;
    }
    break;
    case SMARTAUDIO_RSP_SET_CHANNEL: {
        const smartaudioU8ResponseFrame_t *resp = (const smartaudioU8ResponseFrame_t *)buffer;
        settings->channel = resp->payload;
        settings->update_flags=0x02;
    }
    break;
    case SMARTAUDIO_RSP_SET_POWER: {
        const smartaudioU16ResponseFrame_t *resp = (const smartaudioU16ResponseFrame_t *)buffer;
        settings->channel = (resp->payload >> 8) & 0xFF;
        settings->power = resp->payload & 0xFF;
        settings->update_flags=0x04;
    }
    break;
    case SMARTAUDIO_RSP_SET_MODE: {
        const smartaudioU8ResponseFrame_t *resp = (const smartaudioU8ResponseFrame_t*)buffer;
        smartaudioUnpackOperationMode(settings, resp->payload, false);
        settings->update_flags=0x08;
    }
    break;
    default:
        return false;
    }
    return true;
}

