#include "AP_SmartAudio.h"
#include <AP_Math/crc.h>
#include <GCS_MAVLink/GCS.h>

#ifdef SA_DEBUG
# define debug(fmt, args...)	hal.console->printf("SA: " fmt "\n", ##args)
//#define debug(fmt, args...) gcs().send_text(MAV_SEVERITY_DEBUG , fmt, ##args);
#else
//# define debug(fmt, args...)	do {} while(0)
#endif

extern const AP_HAL::HAL &hal;
const char *TAG="VTX-SMARTAUDIO ";


// table of user settable parameters
const AP_Param::GroupInfo AP_SmartAudio::var_info[] = {

    // @Param: DEFAULTS
    // @DisplayName: VTX will be configured with VTX retrieved from hardware vtx.
    // @Description: VTX will be configured with VTX retrieved from hardware vtx.
    // @Values: 0: Disabled, 1: Enabled
    // @User: Advanced
    AP_GROUPINFO("DEFAULTS", 1, AP_SmartAudio, _smart_audio_param_setup_defaults, 0),

    // @Param: HOTDEPLOY
    // @DisplayName: VTX will be configured with VTX configured params on the fly.
    // @Description: VTX will be configured with VTX retrieved params, updaten hw-vtx config without needing of reboot.
    // @Values: 0: Disabled, 1: Enabled
    // @User: Advanced
    AP_GROUPINFO("HOTDEPLOY", 2, AP_SmartAudio, _smart_audio_param_hot_deploy, 0),

    AP_GROUPEND

};

AP_SmartAudio::AP_SmartAudio()
{
    AP_Param::setup_object_defaults(this, var_info);
    _singleton = this;

}

AP_SmartAudio *AP_SmartAudio::_singleton;

// initialization start making a request settings to the vtx
bool AP_SmartAudio::init()
{

    debug("SmartAudio init");

    if (AP::vtx().get_enabled()==0){
        debug("SmartAudio protocol it's not active");
        return false;
    }

    // init uart
    _port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_SmartAudio, 0);
    if (_port!=nullptr){

          if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_SmartAudio::loop, void),
                                          "SmartAudio",
                                          512, AP_HAL::Scheduler::PRIORITY_IO, -1)) {
            return false;
            }

        // setup port options
        _port->set_stop_bits(2);
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);

        // initialize AP_VideoTx settings
        request_settings();

        return true;
    }
    return false;
}

void AP_SmartAudio::loop(){
    uint8_t res_retries=0;


    // initialise uart (this must be called from within tick b/c the UART begin must be called from the same thread as it is used from)
    _port->begin(AP_SMARTAUDIO_UART_BAUD , AP_SMARTAUDIO_UART_BUFSIZE_RX, AP_SMARTAUDIO_UART_BUFSIZE_TX);


     while (true) {

        // now time to control loop switching
        uint32_t now=AP_HAL::millis();

         // print debug info
        _print_debug_info();

        // when one packet is processed in the iteration will be true.
        bool packet_processed=false;

        // command to process
        Packet current_command;

        // setup a queue polling delay of 20 ms.
     //   hal.scheduler->delay(100);

        // Proccess response in the next 200 milis from the request are sent.
        if(now-_last_request_sended_at_ms>100 && now-_last_request_sended_at_ms<=1000 && _is_waiting_response){
            // allocate response buffer
            uint8_t _response_buffer[AP_SMARTAUDIO_UART_BUFSIZE_RX];

            // setup sheduler delay to 50 ms again after response processes
            if(!read_response(_response_buffer)){
                res_retries++;
                hal.scheduler->delay(100+res_retries);
            }else{
                gcs().send_text(MAV_SEVERITY_NOTICE,"RESPONSE RETRY:%u",res_retries);
            }

            //hal.scheduler->delay(100);
            // prevent to proccess any queued request from the ring buffer
            packet_processed=true;

        }else{
            _is_waiting_response=false;
        }

        // when pending request and last request sended is timeout, take another packet to send
        if (!packet_processed && requests_queue.pop(current_command)){
            // send the popped command from bugger
            send_request(current_command.frame, current_command.frame_size);

            //update send packet time
            current_command.sended_at_ms=AP_HAL::millis();

            //update last send
            _last_request_sended_at_ms=AP_HAL::millis();

            // next loop we expect the response
            _is_waiting_response=true;

            // spec says: The Unify Pro response frame is usually send <100ms after a frame is successfully received from the host MCU
            hal.scheduler->delay(100+res_retries);
            res_retries=0;

        }

        // updates AP_VideoTx values from AP_VideoTx configured params if hot_deploy is enabled
        hot_deploy();

        // enqueu new packets in the buffer to sincro the hw vtx config
        update(false);


     }
}

void AP_SmartAudio::_print_debug_info(){
    uint32_t now_ms=AP_HAL::millis();
    if(now_ms-_debug_last_printing_time_ms>3000){
        debug("\nAP_SmartAudio: Printing Status debug info [%lu,%lu]",now_ms,(now_ms-_debug_last_printing_time_ms));
        _debug_last_printing_time_ms=AP_HAL::millis();
        smartaudioSettings_t current_vtx_status={};

        _peek_vtx_state(current_vtx_status);
        _print_state(current_vtx_status,false,true,false);
        _print_stats(true,true,true);
    }
}

void AP_SmartAudio::_print_state(smartaudioSettings_t& state,bool details,bool updates,bool ap_video_tx_details){

    if(updates){
        if(requests_queue.is_empty()){
            debug("AP_SmartAudio: Hardware vtx is %s, has %s and %s with hot deploy %s"
            ,(state.initialized?"initialized":"unknown")
            ,"no pending updates"
            ,(_is_waiting_response?"is waiting response":"is idle")
            ,(_smart_audio_param_hot_deploy==0?"OFF":"ON")
            );
        }else{
            debug("AP_SmartAudio: Hardware vtx is %s, has %u pending updates and %s with hot deploy %s"
            ,(state.initialized?"initialized":"unknown")
            ,uint8_t(requests_queue.available())
            ,(_is_waiting_response?"is waiting response":"is idle")
            ,(_smart_audio_param_hot_deploy==0?"OFF":"ON")
            );
        }
    }

    if(details){
        debug("AP_SmartAudio: Vtx detailed status is \n ["
        "  version:%u"
        ", channel:%u"
        ", power:%u"
        ", freq:%d"
        ", band:%u"
        ", options:{"
        ", userFrequencyMode:%u"
        ", pitmodeRunning:%u"
        ", pitmodeInRangeActive:%u"
        ", pitmodeOutRangeActive:%u"
        ", unlocked:%u}"
        ", ]"
        , state.version
        , state.channel
        , state.power
        , state.frequency
        , state.band
        , state.userFrequencyMode
        , state.pitModeRunning
        , state.pitmodeInRangeActive
        , state.pitmodeOutRangeActive
        , state.unlocked
        );
    }

        if(ap_video_tx_details){
        debug("AP_VideoTx: Vtx detailed status is \n ["
        ", channel:%u - configured_channel:%u"
        ", power:%u - configured_power:%u"
        ", freq:%d "
        ", band:%u - configured_band:%u"
        ", options:%u - configured_options:%u"
        ", ]"
        , AP::vtx().get_channel(), AP::vtx().get_configured_channel()
        , AP::vtx().get_power_mw(), AP::vtx().get_configured_power_mw()
        , AP::vtx().get_frequency_mhz()
        , AP::vtx().get_band(), AP::vtx().get_configured_band()
        , AP::vtx().get_options(), AP::vtx().get_configured_options()
        );
    }


}

void AP_SmartAudio::_print_stats(bool errors,bool autobauding, bool commands_counters){

    if(errors){
         debug("AP_SmartAudio STATS\t"
        "ERRORS [Unknow Responses Types count: %u, Bad lenght Responses count: %u,Bad Headers Responses count: %u, Bad Crc response count: %u, Unparseable Responses count: %u ]",
        _saStat.badcode,
        _saStat.badlen,
        _saStat.badpre,
        _saStat.crc,
        _saStat.ooopresp);
    }
    if(autobauding){
        debug("AP_SmartAudio STATS\t"
        "AUTO ADJUST BAUD STATS [Responses count: %u, Request counter: %u, Baud Rate: %u, Baud Rate direcction:%s ]",
        _saStat.pktrcvd,
        _saStat.pktsent,
        _sa_smartbaud,
        (_sa_smartbaud_adjdir>0?"Up":"Down"));
    }

    if(commands_counters){
        debug("AP_SmartAudio STATS\t"
        "COMMANDS  [ FAILED READS: %u, REQ_SETTINGS: %u, OTHER REQUESTS:%u ]",
        _saStat.ooopresp,
        _saStat.rqsettings,
        _saStat.other_commands
        );

    }

}

// updates the smartaudio state in sync whith AP_VideoTX
bool AP_SmartAudio::update(bool force)
{
    if (_vtx_current_state==nullptr && requests_queue.is_empty() && !_is_waiting_response){
        if (!hal.util->get_soft_armed()) {
            request_settings();
        }
        return false;
    }
    smartaudioSettings_t current_state;
    //_get_current_state(&current_state);
    _peek_vtx_state(current_state);

    // prevent sync while waiting news from hw vtx or not available current_state but can be forced
    if ((_is_waiting_response || _vtx_current_state==nullptr) && !force){
        return false;
    }

    // send request update for freq with ap_vtx values
    if (current_state.userFrequencyMode && current_state.frequency!=AP::vtx().get_frequency_mhz()){
        debug("UPDATE AP_VTX->HW_VTX: FREQ");
        set_frequency(AP::vtx().get_frequency_mhz(), current_state.pitModeRunning);
    }

    // send request update channel  with ap_vtx values
    if (!current_state.userFrequencyMode && (current_state.channel!=AP::vtx().get_channel() || current_state.band!=AP::vtx().get_band())){
        debug("UPDATE AP_VTX->HW_VTX: CHAN");
        set_channel((AP::vtx().get_band()*8)+AP::vtx().get_channel());
    }

    // sync power with AP_VideoTx backend
    if(sync_power()){
        set_power_mw(AP::vtx().get_power_mw(),_vtx_current_state->version);
    }

    uint8_t curr_operation_mode=(current_state.pitmodeInRangeActive<<0)
    | (current_state.pitmodeOutRangeActive<<1)
    | (current_state.pitModeRunning<<2)
    | ((current_state.unlocked>0)<<3);

    uint8_t new_operation_mode=curr_operation_mode;

    new_operation_mode ^= (-(AP::vtx().get_options() & uint8_t(AP_VideoTX::VideoOptions::VTX_PITMODE)) ^ new_operation_mode) & (1UL << 2);

    new_operation_mode ^= (-(AP::vtx().get_options() & uint8_t(AP_VideoTX::VideoOptions::VTX_UNLOCKED)) ^ new_operation_mode) & (1UL << 3);

    if(curr_operation_mode!=new_operation_mode){
        debug("UPDATE AP_VTX->HW_VTX: OPTIONS");
        set_operation_mode(new_operation_mode);
    }

    return true;
}

/**
 * Sends an SmartAudio Command to the vtx, waits response on the update event
 * @param frameBuffer frameBuffer to send over the wire
 * @param size  size of the framebuffer wich needs to be sended
 */
void AP_SmartAudio::send_request(smartaudioFrame_t requestFrame, uint8_t size)
{

    if (size<=0) {
        debug("ERROR - %s HW: %s", TAG, "CANNOT SEND REQUEST, REQUEST IS EMPTY");
        return;
    }
    if (_port==nullptr) {
        debug("ERROR - %s HW: %s", TAG, "CANNOT SEND REQUEST, UART NOT CONNECTED");
        return;
    }

    uint8_t *request=reinterpret_cast<uint8_t*>(&requestFrame.u8RequestFrame);
    // pull line low
    _port->write((uint8_t)0);

    // write request
    for (uint8_t i= 0; i < size; ++i) {
        _port->write(request[i]);
    }

    _saStat.pktsent++;
#ifdef SA_DEBUG
    _print_bytes_to_hex_string("send_request():", request, size);
#endif
}

/**
 * Reads the response from vtx in the wire
 * - response_buffer, response buffer to fill in
 * - inline_buffer_length , used to passthrought the response lenght in case the response where splitted
 **/
bool AP_SmartAudio::read_response(uint8_t *response_buffer)
{
    int16_t incoming_bytes_count = _port->available();
    uint8_t response_header_size= sizeof(smartaudioFrameHeader_t);

    // check if it is a response in the wire
    if (incoming_bytes_count <= 0) {
       _saStat.ooopresp++;
        // process autobaud routine
        saAutobaud();
        return false;
    }

    for (uint8_t i= 0; i < incoming_bytes_count; ++i) {
        uint8_t response_in_bytes = _port->read();
        debug("Ap_SmartAudio: Received byte %02X",response_in_bytes);
        if ((_inline_buffer_length == 0 && response_in_bytes != SMARTAUDIO_SYNC_BYTE)
            || (_inline_buffer_length == 1 && response_in_bytes != SMARTAUDIO_HEADER_BYTE)) {
            debug("Ap_SmartAudio: byte discard:%02X", response_in_bytes);
            _inline_buffer_length = 0;
        } else if (_inline_buffer_length < AP_SMARTAUDIO_UART_BUFSIZE_RX) {
            response_buffer[_inline_buffer_length++] = response_in_bytes;
        }
    }
#ifdef SA_DEBUG
    _print_bytes_to_hex_string("read_response():", response_buffer, incoming_bytes_count);
#endif
    //last_io_time = AP_HAL::millis();

    if (_inline_buffer_length < response_header_size) {
        debug("INPUT FILTERED FROM UART IS SHORTER THAN EXPLICIT RESPONSE LENGTH");
        _is_waiting_response=false;
        return false;
    }

    uint8_t payload_len = response_buffer[3];

    if (_inline_buffer_length < payload_len + response_header_size) {
        // Not all response bytes received yet, splitted response is incoming
        debug("RESPONSE SEEMS TO BE SPLITTED WAITING IN OTHER LOOP CYCLE ?");
        return true;
    } else {
        _inline_buffer_length = payload_len + response_header_size;
    }
    _is_waiting_response=false;

    bool correct_parse=false;
    correct_parse=parse_frame_response(response_buffer);
    response_buffer=nullptr;
    _inline_buffer_length=0;
    _saStat.pktrcvd++;
    return correct_parse;
}

/**
 * This method parse the frame response and match the current device configuration  *
 *
 * This method only changes the internal state vtx buffer. Using response parser and then updating the internal buffer.
 * */
bool AP_SmartAudio::parse_frame_response(const uint8_t *buffer)
{
    if (buffer!=nullptr) {

        // construct using aggreate initilializer to prevent garbage values into the struct
        smartaudioSettings_t vtx_settings={};

        // process response buffer
        if (!smartaudioParseResponseBuffer(&vtx_settings, buffer)){
            debug("%80s", "Unparseable buffer response");
            _saStat.ooopresp++;
            return false;
        }

        debug("%80s %02X", "selecting update path using:", vtx_settings.update_flags);

        // if partial updates because using setters method to vtx.
        if (vtx_settings.update_flags != uint8_t(AP_SmartAudio::HWVtxUpdates::OVERALL_UPD) ){
            debug("%80s", "parse_frame_response:: overall update flow path");

            smartaudioSettings_t current_vtx_settings;

            // take the current vtx info
            _peek_vtx_state(current_vtx_settings);

            // get update flags from the parsed response_buffer
            current_vtx_settings.update_flags=vtx_settings.update_flags;

            // freq has changed
            if ( vtx_settings.update_flags & uint8_t(AP_SmartAudio::HWVtxUpdates::FREQ_UPD)){
                debug("%80s", "parse_frame_response:: freq update flow path");
                current_vtx_settings.frequency=vtx_settings.frequency;
                current_vtx_settings.userFrequencyMode=vtx_settings.userFrequencyMode;
            }

            // channel has changed
            if ( vtx_settings.update_flags & uint8_t(AP_SmartAudio::HWVtxUpdates::CHAN_UPD)){
                debug("%80s", "parse_frame_response:: band chan update flow path");
                current_vtx_settings.band=vtx_settings.channel/8;
                current_vtx_settings.channel=vtx_settings.channel%8;
            }

            // power has changed
            if ( vtx_settings.update_flags & uint8_t(AP_SmartAudio::HWVtxUpdates::POW_UPD)){
                debug("%80s", "parse_frame_response:: pow update flow path");
                if (current_vtx_settings.version==SMARTAUDIO_SPEC_PROTOCOL_v21){
                    // setted the power in dbm
                    current_vtx_settings.power_in_dbm=vtx_settings.power;
                }else{
                      // setted the power level
                      current_vtx_settings.power=vtx_settings.power;
                }
            }

            // mode has changed
            if ( vtx_settings.update_flags & uint8_t(AP_SmartAudio::HWVtxUpdates::MODE_UPD)){
                debug("%80s", "parse_frame_response:: mode update flow path");
                current_vtx_settings.pitModeRunning=vtx_settings.pitModeRunning;
                current_vtx_settings.pitmodeInRangeActive=vtx_settings.pitmodeInRangeActive;
                current_vtx_settings.pitmodeOutRangeActive=vtx_settings.pitmodeOutRangeActive;
                current_vtx_settings.unlocked=vtx_settings.unlocked;
            }

            vtx_settings=current_vtx_settings;

        }

        // update bands and channels accordily when frequency changes
        if ((vtx_settings.update_flags & uint8_t(AP_SmartAudio::HWVtxUpdates::FREQ_UPD)) && vtx_settings.userFrequencyMode){
            debug("%80s", "parse_frame_response:: band update in user freq mode");
            AP_VideoTX::VideoBand video_band;
            AP_VideoTX::get_band_and_channel(vtx_settings.frequency, video_band, vtx_settings.channel);
            vtx_settings.band=(int)video_band;
        }

        // update band and frequency accorly when channel updates
        if ( (vtx_settings.update_flags & uint8_t(AP_SmartAudio::HWVtxUpdates::CHAN_UPD)) && !vtx_settings.userFrequencyMode){
            debug("%80s", "parse_frame_response:: freq update from band and chan settings");
            vtx_settings.frequency=AP_VideoTX::get_frequency_mhz(vtx_settings.band, vtx_settings.channel);
            AP_VideoTX::VideoBand video_band;
            AP_VideoTX::get_band_and_channel(vtx_settings.frequency, video_band, vtx_settings.channel);
            vtx_settings.band=(int)video_band;
        }

        // reset vtx_settings_change_control variables
        vtx_settings.update_flags=uint8_t(AP_SmartAudio::HWVtxUpdates::NO_UPD);
        vtx_settings.initialized=true;
        _push_vtx_state(vtx_settings);
        return true;
    }

   // debug("%s HW: %s", TAG, "EMPTY RESPONSE");
    debug("%s HW: %s", TAG, "EMPTY RESPONSE");

    return false;
}

/**
 *  This method parses the internal vtx current state stored in internal buffer, transforming the state values into the AP_VideoTx passed as argument.
 *   @DependsOn: AP_VideoTX, AP_VideoTX::VideoBand
 * */
bool AP_SmartAudio::get_readings(AP_VideoTX *vtx_dest)
{
    // take the first register from the output buffer
   smartaudioSettings_t current_state;

    // peek from buffer
   //vtx_states_queue.peek(&current_state, 1);
   _peek_vtx_state(current_state);

   // setting frequency
    vtx_dest->set_frequency_mhz(current_state.frequency);

    // set channel
    vtx_dest->set_band((AP_VideoTX::VideoBand)((int)current_state.band));

    // setting channel 0 -> 40
    vtx_dest->set_channel(current_state.channel);

   // setup default value for options
    vtx_dest->set_options(0);

      // pitmode enabled
   if (current_state.pitmodeOutRangeActive ||  current_state.pitmodeInRangeActive || current_state.pitModeRunning){
       vtx_dest->set_options(vtx_dest->get_options() | uint8_t(AP_VideoTX::VideoOptions::VTX_PITMODE));
   }
    // pitmode disabled only by this option
   if (!current_state.pitModeRunning){
       vtx_dest->set_options(vtx_dest->get_options() & ~uint8_t(AP_VideoTX::VideoOptions::VTX_PITMODE));
   }

   // locking status

   vtx_dest->set_options(current_state.unlocked==0?
   (vtx_dest->get_options() | uint8_t(AP_VideoTX::VideoOptions::VTX_UNLOCKED))
   :
   (vtx_dest->get_options() | ~uint8_t(AP_VideoTX::VideoOptions::VTX_UNLOCKED))
   );

   // spec 2.1 power-levels in dbm
   vtx_dest->set_power_dbm(current_state.power_in_dbm);

   // specs 1 and 2 power-levels need transformation to dbm power
   if (current_state.version!=SMARTAUDIO_SPEC_PROTOCOL_v21){
       // search in power tables
        if (_get_power_in_dbm_from_vtx_power_level(current_state.power, current_state.version, current_state.power_in_dbm)){
            vtx_dest->set_power_dbm(current_state.power_in_dbm);
        }else{
            _get_power_in_dbm_from_vtx_power_level(POWER_LEVELS[current_state.version][0], current_state.version, current_state.power_in_dbm);
            vtx_dest->set_power_dbm(current_state.power_in_dbm);
        }
   }

   //return !vtx_states_queue.is_empty();
   return _vtx_current_state!=nullptr;
}



/**
 * Sends get settings command.
 * */
void AP_SmartAudio::request_settings()
{
    //debug("%80s::request_settings()\t", TAG);
    _saStat.rqsettings++;
    debug("request_settings()");
    smartaudioFrame_t request;
    uint8_t frame_size=smartaudioFrameGetSettings(&request);
    Packet command;
    command.frame=request;
    command.frame_size=frame_size;
    requests_queue.push_force(command);
}


void AP_SmartAudio::set_operation_mode(uint8_t mode){
    _saStat.other_commands++;
     // take the first register from the output buffer
   smartaudioSettings_t current_state;

    // peek from buffer
   //vtx_states_queue.peek(&current_state, 1);
   _peek_vtx_state(current_state);

    debug("AP_SmartAudio: Changing operation mode to %02X",  mode);
    // SPEC SAYS ONLY V2 SUPPORT SET MODE BUT THIS INCLUDES 2.1 ?
    if (current_state.version<SMARTAUDIO_SPEC_PROTOCOL_v2) {
        debug("%s HW: %s", TAG, "Device can't change operation mode. Spec protocol not supported");
        return;
    }
    smartaudioSettings_t settings;
    settings.pitmodeInRangeActive=mode & (1<<0);
    settings.pitmodeOutRangeActive=mode & (1<<1);
    settings.pitModeRunning=mode & (1<<2);
    settings.unlocked=(mode & 1<<3)!=0?1:0;

    smartaudioFrame_t request;
    uint8_t frame_size=smartaudioFrameSetOperationMode(&request, &settings);
    Packet command;
    command.frame=request;
    command.frame_size=frame_size;
    requests_queue.push_force(command);

}


/**
     * Sets the frequency to transmit in the vtx.
     * When isPitModeFreq active the freq will be set to be used when in pitmode (in range)
     */
void AP_SmartAudio::set_frequency(uint16_t frequency, bool isPitModeFreq)
{
    _saStat.other_commands++;

    //debug("%80s::set_frequency(%d, %d)\t", TAG, frequency, isPitModeFreq);
    debug("%s: Setting frequency to %d with pitmodeFreq(%d)", "AP_SmartAudio", frequency, isPitModeFreq);
    smartaudioFrame_t request;
    uint8_t frame_size=smartaudioFrameSetFrequency(&request, frequency, isPitModeFreq);
    Packet command;
    command.frame=request;
    command.frame_size=frame_size;
    requests_queue.push_force(command);
}

// enqueue a set channel request
void AP_SmartAudio::set_channel(uint8_t chan_idx){

        _saStat.other_commands++;

        debug("%s: Setting channel to %d ", "AP_SmartAudio", chan_idx);

        //debug("%80s::set_channel(%d)\t", TAG, chan_idx);
        smartaudioFrame_t request;
        uint8_t frame_size=smartaudioFrameSetChannel(&request, chan_idx);
         Packet command;
        command.frame=request;
        command.frame_size=frame_size;
        requests_queue.push_force(command);
}

/**
 * Request pitMode Frequency setted into the vtx hardware
 * */
void AP_SmartAudio::request_pit_mode_frequency(){
    _saStat.other_commands++;

    debug("%s: Requesting pit mode frequency ", "AP_SmartAudio");

    //debug("%80s::", "request_pit_mode_frequency()\t");
    smartaudioFrame_t request;
    uint8_t frame_size=smartaudioFrameGetPitmodeFrequency(&request);
    Packet command;
    command.frame=request;
    command.frame_size=frame_size;
    requests_queue.push_force(command);
}

// send vtx request to set power
 void AP_SmartAudio::set_power(uint8_t power_level){
    _saStat.other_commands++;

    smartaudioFrame_t request;
    uint8_t frame_size=0;

     frame_size=smartaudioFrameSetPower(&request, power_level);
     Packet command;
     command.frame=request;
     command.frame_size=frame_size;
     requests_queue.push_force(command);
 }

// send vtx request to set power defined in mw
void AP_SmartAudio::set_power_mw(uint16_t power_mw,uint8_t spec_version){

    // Spec set power directly in dbm corresponding to the data table responsed by settings
    if(spec_version!=SMARTAUDIO_SPEC_PROTOCOL_v21){
        set_power(_get_power_level_from_dbm(spec_version,_get_power_in_dbm_from_mw(power_mw)));
    }

    // Spec set power using power levels transformation tables
    if(spec_version==SMARTAUDIO_SPEC_PROTOCOL_v21){
        set_power(_get_power_in_dbm_from_mw(power_mw));
    }
    return;
}

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

void AP_SmartAudio::smartaudioFrameInit(const uint8_t command, smartaudioFrameHeader_t *header, const uint8_t payloadLength)
{
    header->syncByte = SMARTAUDIO_SYNC_BYTE;
    header->headerByte= SMARTAUDIO_HEADER_BYTE;
    header->length = payloadLength;
    header->command = command;

}

void AP_SmartAudio::smartaudioUnpackOperationMode(smartaudioSettings_t *settings, const uint8_t operationMode, const bool settingsResponse)
{
    if (settingsResponse) {
        // operation mode bit order is different between 'Get Settings' and 'Set Mode' responses.
        settings->userFrequencyMode = operationMode & 0x01;
        settings->pitModeRunning = operationMode & 0x02;
        settings->pitmodeInRangeActive = operationMode & 0x04;
        settings->pitmodeOutRangeActive = operationMode & 0x08;
        settings->unlocked = operationMode & 0x10;
    } else {
        settings->pitmodeInRangeActive = operationMode & 0x01;
        settings->pitmodeOutRangeActive = operationMode & 0x02;
        settings->pitModeRunning = operationMode & 0x04;
        settings->unlocked = operationMode & 0x08;
    }
}

void AP_SmartAudio::smartaudioUnpackFrequency(AP_SmartAudio::smartaudioSettings_t *settings, const uint16_t frequency)
{

    if (applyBigEndian16(frequency) & SMARTAUDIO_GET_PITMODE_FREQ) {
        settings->pitmodeFrequency = applyBigEndian16(frequency);
    } else {
        settings->frequency = applyBigEndian16(frequency);
    }
}

void AP_SmartAudio::smartaudioUnpackSettings(smartaudioSettings_t *settings, const smartaudioSettingsResponseFrame_t *frame)
{
    settings->channel = frame->channel;
    settings->power = frame->power;
    smartaudioUnpackFrequency(settings, frame->frequency);
    smartaudioUnpackOperationMode(settings, frame->operationMode, true);
}

void AP_SmartAudio::smartaudioUnpackSettings(smartaudioSettings_t *settings, const smartaudioSettingsExtendedResponseFrame_t *frame)
{
    settings->channel = frame->channel;
    settings->power = frame->power;
    settings->power_in_dbm=frame->power_dbm;
    smartaudioUnpackFrequency(settings, frame->frequency);
    smartaudioUnpackOperationMode(settings, frame->operationMode, true);
}

uint8_t AP_SmartAudio::smartaudioPackOperationMode(const smartaudioSettings_t *settings)
{
    uint8_t operationMode = 0;
    operationMode |= (settings->pitmodeInRangeActive << 0);
    operationMode |= (settings->pitmodeOutRangeActive << 1);
    operationMode |= (settings->pitModeRunning << 2);
    operationMode |= (settings->unlocked << 3);
    return operationMode;
}



size_t AP_SmartAudio::smartaudioFrameGetSettings(AP_SmartAudio::smartaudioFrame_t *smartaudioFrame)
{
    AP_SmartAudio::smartaudioCommandOnlyFrame_t *frame = (AP_SmartAudio::smartaudioCommandOnlyFrame_t *)smartaudioFrame;
    smartaudioFrameInit(SMARTAUDIO_CMD_GET_SETTINGS, &frame->header, 0);
    frame->crc = crc8_dvb_s2_update(0, frame, sizeof(AP_SmartAudio::smartaudioCommandOnlyFrame_t) - sizeof(frame->crc));
    //frame->crc=crc8_dvb_s2(*(const uint8_t *)frame, sizeof(smartaudioCommandOnlyFrame_t) - sizeof(frame->crc));
    return sizeof(AP_SmartAudio::smartaudioCommandOnlyFrame_t);
}

size_t AP_SmartAudio::smartaudioFrameGetPitmodeFrequency(AP_SmartAudio::smartaudioFrame_t *smartaudioFrame)
{
    AP_SmartAudio::smartaudioU16Frame_t *frame = (smartaudioU16Frame_t *)smartaudioFrame;
    smartaudioFrameInit(SMARTAUDIO_CMD_SET_FREQUENCY, &frame->header, sizeof(frame->payload));
    frame->payload = SMARTAUDIO_GET_PITMODE_FREQ;
    frame->crc = crc8_dvb_s2_update(0, frame, sizeof(smartaudioU16Frame_t) - sizeof(frame->crc));
    //frame->crc = crc8_dvb_s2(*(const uint8_t *)frame, sizeof(smartaudioU16Frame_t) - sizeof(frame->crc));
    return sizeof(smartaudioU16Frame_t);
}

size_t AP_SmartAudio::smartaudioFrameSetPower(smartaudioFrame_t *smartaudioFrame, const uint8_t power)
{
    smartaudioU8Frame_t *frame = (smartaudioU8Frame_t *)smartaudioFrame;
    smartaudioFrameInit(SMARTAUDIO_CMD_SET_POWER, &frame->header, sizeof(frame->payload));
    frame->payload = power;
    frame->crc = crc8_dvb_s2_update(0, frame, sizeof(smartaudioU8Frame_t) - sizeof(frame->crc));
    //frame->crc = crc8_dvb_s2(*(const uint8_t *)frame, sizeof(smartaudioU8Frame_t) - sizeof(frame->crc));
    return sizeof(smartaudioU8Frame_t);
}

size_t AP_SmartAudio::smartaudioFrameSetBandChannel(smartaudioFrame_t *smartaudioFrame, const uint8_t band, const uint8_t channel)
{
    smartaudioU8Frame_t *frame = (smartaudioU8Frame_t *)smartaudioFrame;
    smartaudioFrameInit(SMARTAUDIO_CMD_SET_CHANNEL, &frame->header, sizeof(frame->payload));
    frame->payload = SMARTAUDIO_BANDCHAN_TO_INDEX(band, channel);
    frame->crc = crc8_dvb_s2_update(0, frame, sizeof(smartaudioU8Frame_t) - sizeof(frame->crc));
    //frame->crc = crc8_dvb_s2( *(const uint8_t *)frame, sizeof(smartaudioU8Frame_t) - sizeof(frame->crc));
    return sizeof(smartaudioU8Frame_t);
}

size_t AP_SmartAudio::smartaudioFrameSetChannel(smartaudioFrame_t *smartaudioFrame, const uint8_t channel)
{
    smartaudioU8Frame_t *frame = (smartaudioU8Frame_t *)smartaudioFrame;
    smartaudioFrameInit(SMARTAUDIO_CMD_SET_CHANNEL, &frame->header, sizeof(frame->payload));
    frame->payload = channel;
    frame->crc = crc8_dvb_s2_update(0, frame, sizeof(smartaudioU8Frame_t) - sizeof(frame->crc));
    //frame->crc = crc8_dvb_s2( *(const uint8_t *)frame, sizeof(smartaudioU8Frame_t) - sizeof(frame->crc));
    return sizeof(smartaudioU8Frame_t);
}

size_t AP_SmartAudio::smartaudioFrameSetFrequency(smartaudioFrame_t *smartaudioFrame, const uint16_t frequency, const bool pitmodeFrequency)
{
    smartaudioU16Frame_t *frame = (smartaudioU16Frame_t *)smartaudioFrame;
    smartaudioFrameInit(SMARTAUDIO_CMD_SET_FREQUENCY, &frame->header, sizeof(frame->payload));
    frame->payload = applyBigEndian16(frequency | (pitmodeFrequency ? SMARTAUDIO_SET_PITMODE_FREQ : 0x00));
    frame->crc = crc8_dvb_s2_update(0, frame, sizeof(smartaudioU16Frame_t) - sizeof(frame->crc));
    return sizeof(smartaudioU16Frame_t);
}

/** Addition because the define macro seems that's not working. TODO: Refactor this using AP routines.*/
uint16_t AP_SmartAudio::applyBigEndian16(uint16_t bytes)
{
    return (bytes << 8) | ((bytes >> 8) & 0xFF);
}

size_t AP_SmartAudio::smartaudioFrameSetOperationMode(smartaudioFrame_t *smartaudioFrame, const smartaudioSettings_t *settings)
{
    smartaudioU8Frame_t *frame = (smartaudioU8Frame_t *)smartaudioFrame;
    smartaudioFrameInit(SMARTAUDIO_CMD_SET_MODE, &frame->header, sizeof(frame->payload));
    frame->payload = smartaudioPackOperationMode(settings);
    frame->crc = crc8_dvb_s2_update(0, frame, sizeof(smartaudioU8Frame_t) - sizeof(frame->crc));
    return sizeof(smartaudioU8Frame_t);
}

#ifdef SA_DEBUG
void AP_SmartAudio::_print_bytes_to_hex_string(const char* msg, uint8_t buf[], uint8_t len)
{
    hal.console->printf("SA: %s ", msg);
    for (uint8_t i = 0; i < len; i++) {
        hal.console->printf("0x%02X ", buf[i]);
    }
    hal.console->printf("\n");
}
#endif

bool  AP_SmartAudio::smartaudioParseResponseBuffer(smartaudioSettings_t *settings, const uint8_t *buffer)
{
    const smartaudioFrameHeader_t *header = (const smartaudioFrameHeader_t *)buffer;
    const uint8_t fullFrameLength = sizeof(smartaudioFrameHeader_t) + header->length;
    const uint8_t headerPayloadLength = fullFrameLength - 1; // subtract crc byte from length
    const uint8_t *startPtr = buffer + 2;
    const uint8_t *endPtr = buffer + headerPayloadLength;

    if (crc8_dvb_s2_update(0x00, startPtr, headerPayloadLength-2)!=*(endPtr) || header->headerByte != SMARTAUDIO_HEADER_BYTE || header->syncByte!=SMARTAUDIO_SYNC_BYTE) {
        debug(" PARSE RESPONSE bad frame response crc check %02X", *endPtr);
        _saStat.crc++;
        return false;
    }
    // SEND TO GCS A MESSAGE TO UNDERSTAND WHATS HAPPENING
    advice_gcs(header->command);
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
        _saStat.badcode++;
        return false;
    }
    return true;
}

void AP_SmartAudio::saAutobaud()
{
    if (_saStat.pktsent < 10) {
        // Not enough samples collected
        return;
    }
    // gcs().send_text(MAV_SEVERITY_NOTICE ,"AP_SmartAudio: AUTOBAUD SANITY CHECK %ul.",((_saStat.pktrcvd * 100) / _saStat.pktsent));

    if (((_saStat.pktrcvd * 100) / _saStat.pktsent) >= 70) {
     //    gcs().send_text(MAV_SEVERITY_NOTICE ,"AP_SmartAudio: El control de baudios está sano %ul.",((_saStat.pktrcvd * 100) / _saStat.pktsent) >= 70);
        // This is okay
        _saStat.pktsent = 0; // Should be more moderate?
        _saStat.pktrcvd = 0;

        return;
    }

//    debug("autobaud: adjusting\r\n");

    if ((_sa_smartbaud_adjdir == 1) && (_sa_smartbaud == AP_SMARTAUDIO_SMARTBAUD_MAX)) {
        _sa_smartbaud_adjdir = -1;
       // gcs().send_text(MAV_SEVERITY_NOTICE ,"AP_SmartAudio: El control de baudios está ajustando descendentemente");
//        debug("autobaud: now going down\r\n");
    } else if ((_sa_smartbaud_adjdir == -1 && _sa_smartbaud == AP_SMARTAUDIO_SMARTBAUD_MIN)) {
        _sa_smartbaud_adjdir = 1;
       // gcs().send_text(MAV_SEVERITY_NOTICE ,"AP_SmartAudio: El control de baudios está ajustando ascendetemente");
//        debug("autobaud: now going up\r\n");
    }

    _sa_smartbaud += AP_SMARTAUDIO_SMARTBAUD_STEP * _sa_smartbaud_adjdir;

  //  debug("autobaud: %d\r\n", _sa_smartbaud);

    //smartAudioSerialPort->vTable->serialSetBaudRate(smartAudioSerialPort, sa_smartbaud);
    _port->begin(_sa_smartbaud,0,0);
   // gcs().send_text(MAV_SEVERITY_NOTICE ,"AP_SmartAudio: El control de baudios está ajustando a %d baudios",_sa_smartbaud);

    _saStat.pktsent = 0;
    _saStat.pktrcvd = 0;

}

// updates the AP_VideoTx current values with the configured ones when hot_deploy is active.
bool AP_SmartAudio::hot_deploy(){
    if(_smart_audio_param_hot_deploy==1){
        if(AP::vtx().get_configured_band()!=AP::vtx().get_band()){
           // debug("AP_SmartAudio: UPDATE BAND FROM PARAMS from current %u to configured %u",AP::vtx().get_band(),AP::vtx().get_configured_band());
            AP::vtx().set_band(AP::vtx().get_configured_band());
        }

        if(AP::vtx().get_configured_channel()!=AP::vtx().get_channel()){
           // debug("AP_SmartAudio: UPDATE CHAN FROM PARAMS from current %u to configured %u",AP::vtx().get_channel(),AP::vtx().get_configured_channel());
            AP::vtx().set_channel(AP::vtx().get_configured_channel());
        }

        if(AP::vtx().get_power_mw()!=AP::vtx().get_configured_power_mw()){
           // debug("AP_SmartAudio: UPDATE POW FROM PARAMS from current %u to configured %u",AP::vtx().get_power_mw(),AP::vtx().get_configured_power_mw());
            AP::vtx().set_power_mw(AP::vtx().get_configured_power_mw());
        }

        if(AP::vtx().get_options()!=AP::vtx().get_configured_options()){
           // debug("AP_SmartAudio: UPDATE OPTIONS FROM PARAMS from current %u to configured %u",AP::vtx().get_options(),AP::vtx().get_configured_options());
            AP::vtx().set_options(AP::vtx().get_configured_options());
        }
        return true;
    }
    return false;
}

// return true if power need to bee synced with the AP_VideoTx backend
bool AP_SmartAudio::sync_power(){
    if(_vtx_current_state->version==SMARTAUDIO_SPEC_PROTOCOL_v21){
        return AP::vtx().get_power_mw()!=_get_power_in_mw_from_dbm(_vtx_current_state->power);
    }else{
        return AP::vtx().get_power_mw()!=_get_power_in_mw_from_dbm(_get_power_in_dbm_from_vtx_power_level(_vtx_current_state->power,_vtx_current_state->version));
    }
}

void AP_SmartAudio::advice_gcs(uint8_t command){
        uint32_t now=AP_HAL::millis();
         switch(command){
             case SMARTAUDIO_RSP_GET_SETTINGS_V1:
                gcs().send_text(MAV_SEVERITY_CRITICAL ,"AP_SmartAudio: Operation REQUEST SETTINGS VERSION 1 completed since %lu milliseconds",(now-_last_request_sended_at_ms));

                break;
            case SMARTAUDIO_RSP_GET_SETTINGS_V2:
                gcs().send_text(MAV_SEVERITY_CRITICAL ,"AP_SmartAudio: Operation REQUEST SETTINGS VERSION 2 completed since %lu milliseconds",(now-_last_request_sended_at_ms));

                break;
            case SMARTAUDIO_RSP_GET_SETTINGS_V21:
                gcs().send_text(MAV_SEVERITY_CRITICAL ,"AP_SmartAudio: Operation REQUEST SETTINGS VERSION 2.1 completed since %lu milliseconds",(now-_last_request_sended_at_ms));

                break;
             case SMARTAUDIO_RSP_SET_CHANNEL:
                gcs().send_text(MAV_SEVERITY_CRITICAL ,"AP_SmartAudio: Operation SET CHANNEL completed since %lu milliseconds",(now-_last_request_sended_at_ms));

                break;
            case SMARTAUDIO_RSP_SET_POWER:
                gcs().send_text(MAV_SEVERITY_CRITICAL ,"AP_SmartAudio: Operation SET POWER completed since %lu milliseconds",(now-_last_request_sended_at_ms));

                break;
            case SMARTAUDIO_RSP_SET_MODE:
                gcs().send_text(MAV_SEVERITY_CRITICAL ,"AP_SmartAudio: Operation SET MODE completed since %lu milliseconds",(now-_last_request_sended_at_ms));

                break;
             default:
                gcs().send_text(MAV_SEVERITY_CRITICAL ,"AP_SmartAudio: Operation UNKNOWN RESPONSE OPERATION completed since %lu milliseconds",(now-_last_request_sended_at_ms));

                break;
         }

    }


