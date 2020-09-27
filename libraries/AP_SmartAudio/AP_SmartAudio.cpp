#include "AP_SmartAudio.h"
#include <AP_Math/crc.h>
#include <GCS_MAVLink/GCS.h>

#define SA_DEBUG     // DEV TESTS ACTIVATE
#ifdef SA_DEBUG
# define debug(fmt, args...)	hal.console->printf("\r " fmt "\n", ##args)
//#define debug(fmt, args...) gcs().send_text(MAV_SEVERITY_DEBUG , fmt, ##args);
#else
//# define debug(fmt, args...)	do {} while(0)
#endif

extern const AP_HAL::HAL &hal;
const char *TAG="VTX-SMARTAUDIO ";


// table of user settable parameters
const AP_Param::GroupInfo AP_SmartAudio::var_info[] = {

    // @Param: DEFAULTS
    // @DisplayName: VTX will be configured with VTX params defined by default.
    // @Description: VTX will be configured with VTX params defined by default.
    // @Values: 0: Disabled, 1: Enabled
    // @User: Advanced
    AP_GROUPINFO("DEFAULTS", 3, AP_SmartAudio, _smart_audio_param_setup_defaults, 0),

    AP_GROUPEND

};

AP_SmartAudio::AP_SmartAudio()
{
    AP_Param::setup_object_defaults(this, var_info);
    singleton = this;

}

AP_SmartAudio *AP_SmartAudio::singleton;

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
                                          256, AP_HAL::Scheduler::PRIORITY_IO, -1)) {
            return false;
            }

        // setup port options
        _port->set_stop_bits(2);
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        _port->set_options(AP_HAL::UARTDriver::OPTION_HDPLEX);


        // initialize AP_VideoTx settings
        request_settings();

        return true;
    }
    return false;
}

void AP_SmartAudio::loop(){

    // initialise uart (this must be called from within tick b/c the UART begin must be called from the same thread as it is used from)
    _port->begin(AP_SMARTAUDIO_UART_BAUD , AP_SMARTAUDIO_UART_BUFSIZE_RX, AP_SMARTAUDIO_UART_BUFSIZE_TX);


     while (true) {

        // now time to control loop switching
        uint32_t now=AP_HAL::millis();

        // when one packet is processed in the iteration will be true.
        bool packet_processed=false;

        // command to process
        Packet current_command;

        // setup a queue polling delay of 20 ms.
        hal.scheduler->delay(50);

        // Proccess response in the next 200 milis from the request are sent.
        debug("LOOP: Checking Responses");
        if (now-last_request_sended_at_ms>100 && now-last_request_sended_at_ms<=1000 && is_waiting_response){
            #ifdef BOARD_IS_SITL
            debug("I'M WAITING RESPONSE SINCE %u and %u ms more", last_request_sended_at_ms, 1000-(now-last_request_sended_at_ms));
            #else
            debug("I'M WAITING RESPONSE SINCE %lu and %lu ms more", last_request_sended_at_ms, 1000-(now-last_request_sended_at_ms));
            #endif

            // allocate response buffer
            uint8_t _response_buffer[AP_SMARTAUDIO_UART_BUFSIZE_RX];
            // setup to zero because the
            uint8_t _inline_buffer_length=0;
            // setup sheduler delay to 50 ms again after response processes
            read_response(_response_buffer, _inline_buffer_length);
            debug("LOOP: scheduler delay 1000");
            hal.scheduler->delay(500);
            // prevent to proccess any queued request from the ring buffer
            debug("LOOP: Packet Processed");
            packet_processed=true;
        }else{
            debug("LOOP: Disabling waiting response");
            is_waiting_response=false;
        }

        // when pending request and last request sended is timeout, take another packet to send
        debug("LOOP: Checking Request");
        if (!packet_processed && requests_queue.pop(current_command)){
            #ifdef BOARD_IS_SITL
            debug(" LOOP MAKING REQUEST TO SEND AT %u ms ", AP_HAL::millis());
            #else
            debug(" LOOP MAKING REQUEST TO SEND AT %lu ms ", AP_HAL::millis());
            #endif
            send_request(current_command.frame, current_command.frame_size);
            current_command.sended_at_ms=AP_HAL::millis();
            last_request_sended_at_ms=AP_HAL::millis();
            is_waiting_response=true;
            // spec says: The Unify Pro response frame is usually send <100ms after a frame is successfully received from the host MCU
            debug(" NEXT LOOP DELAYED %d ms", 100);
            hal.scheduler->delay(100);
        }


        debug("LOOP:Updating HW VTX from AP VTX status");
        update(false);
     }
}

void AP_SmartAudio::_print_state(smartaudioSettings_t *state){
    if (state!=nullptr){
    debug("{version:%u\r"
    ", \nchannel:%u\r"
    ", \npower:%u\r"
    ", \nfreq:%d\r"
    ", \nband:%u\r"
    ", \nuserFrequencyMode:%u\r"
    ", \npitmodeRunning:%u\r"
    ", \npitmodeInRangeActive:%u\r"
    ", \npitmodeOutRangeActive:%u\r"
    ", \nunlocked:%u\r"
    ", \r\n}"
    , state->version
    , state->channel
    , state->power
    , state->frequency
    , state->band
    , state->userFrequencyMode
    , state->pitModeRunning
    , state->pitmodeInRangeActive
    , state->pitmodeOutRangeActive
    , state->unlocked
    );
    }else{
    debug ("STATE : EMPTY STATE ?");
    }
}



// updates the smartaudio state in sync whith AP_VideoTX
bool AP_SmartAudio::update(bool force)
{


    if (vtx_states_queue.is_empty() && requests_queue.is_empty() && !is_waiting_response){
        if (!hal.util->get_soft_armed()) {
            request_settings();
        }
        return false;
    }
    smartaudioSettings_t current_state;
    _get_current_state(&current_state);

     debug("is_waiting_response:%u", is_waiting_response);
    if ((is_waiting_response || _get_current_state(&current_state)==nullptr) && !force){
        debug("Breaking the update ...");
        return false;
    }
    _print_state(&current_state);

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

    // send request update for power with ap_vtx values
    if (current_state.version==SMARTAUDIO_SPEC_PROTOCOL_v21 && _get_power_in_mw_from_dbm(current_state.power_in_dbm)!= AP::vtx().get_power_mw()){
        debug("UPDATE AP_VTX->HW_VTX: POW IN MW FROM DBM");
        set_power_mw(AP::vtx().get_power_mw());
    }

    // send request update for power with ap_vtx values
    if (current_state.version!=SMARTAUDIO_SPEC_PROTOCOL_v21
    && AP::vtx().get_power_mw()!=_get_power_in_mw_from_dbm(_get_power_in_dbm_from_vtx_power_level(current_state.power, current_state.version))){
        debug("UPDATE AP_VTX->HW_VTX: POW IN MW FROM PWLEVEL %d -> %d"
        , AP::vtx().get_power_mw()
        , _get_power_in_mw_from_dbm(_get_power_in_dbm_from_vtx_power_level(current_state.power, current_state.version))
        );

        set_power_mw(AP::vtx().get_power_mw());
    }

    // send request update for options with ap_vtx values
    if ( (AP::vtx().get_options()==0 && current_state.pitModeRunning) || (AP::vtx().get_options()==1 && !current_state.pitModeRunning) ){
        debug("UPDATE AP_VTX->HW_VTX: OPTIONS");
        uint8_t operation_mode= 0x00;

        debug("UPDATE AP_VTX->HW_VTX: OPTIONS LOCKING %u %u", AP::vtx().get_locking(), AP::vtx().get_locking()<<3);
        debug("OPERATION MODE TO UPDATE:%u %u %u %u"
        , 0x0F & AP::vtx().get_locking()<<3
        , 0X0F & AP::vtx().get_options()<<2
        , 0X0F & current_state.pitmodeOutRangeActive<<1
        , 0X0F & current_state.pitmodeInRangeActive<<0
        );

        operation_mode |= (current_state.pitmodeInRangeActive<<0);

        operation_mode |= (current_state.pitmodeOutRangeActive<<1);

        operation_mode |= (AP::vtx().get_options()<<2);

        // 1 unlocked 0 locked
        operation_mode |= (AP::vtx().get_locking()<<3);

        set_operation_mode(operation_mode);
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
    debug(" REQ-SEND %d bytes", size);

    if (size<=0) {
        //debug("%s HW: %s", TAG, "CANNOT SEND REQUEST, REQUEST IS EMPTY");
        debug("ERROR - %s HW: %s", TAG, "CANNOT SEND REQUEST, REQUEST IS EMPTY");
        return;
    }
    if (_port==nullptr) {
        //debug("%s HW: %s", TAG, "CANNOT SEND REQUEST, UART NOT CONNECTED");
        debug("ERROR - %s HW: %s", TAG, "CANNOT SEND REQUEST, UART NOT CONNECTED");
        return;
    }

    uint8_t *request=reinterpret_cast<uint8_t*>(&requestFrame.u8RequestFrame);
    // pull line low
    _port->write((uint8_t)0);
    // write request
    for (uint8_t i= 0; i < size; ++i) {
        _port->write(request[i]);
         debug(" REQ-SEND bytes:%02X", request[i]);
    }
    debug("-------------->");
    AP_SmartAudio::_print_bytes_to_hex_string(request, size);
}

/**
 * Reads the response from vtx in the wire
 * - response_buffer, response buffer to fill in
 * - inline_buffer_length , used to passthrought the response lenght in case the response where splitted
 **/
void AP_SmartAudio::read_response(uint8_t *response_buffer, uint8_t inline_buffer_length)
{
    int16_t incoming_bytes_count = _port->available();
    uint8_t response_header_size= sizeof(smartaudioFrameHeader_t);

    // check if it is a response in the wire
    if (incoming_bytes_count < 1) {
        debug(" WARNING - %s HW: %s", TAG, "EMPTY WIRE");
        return;
    }

    debug("%80s %d", "READ RESPONSE incoming_bytes_count:", incoming_bytes_count);
    for (uint8_t i= 0; i < incoming_bytes_count; ++i) {
        uint8_t response_in_bytes = _port->read();
        debug(" READ RESPONSE response_in_bytes:%02X", response_in_bytes);
        if ((inline_buffer_length == 0 && response_in_bytes != SMARTAUDIO_SYNC_BYTE)
            || (inline_buffer_length == 1 && response_in_bytes != SMARTAUDIO_HEADER_BYTE)) {
            debug(" READ RESPONSE byte discard:%02X", response_in_bytes);
            inline_buffer_length = 0;
        } else if (inline_buffer_length < 16) {
            response_buffer[inline_buffer_length++] = response_in_bytes;
            debug(" READ RESPONSE byte -> response_buffer added :%02X", response_buffer[inline_buffer_length-1]);
        }
    }

    //last_io_time = AP_HAL::millis();

    if (inline_buffer_length < response_header_size) {
        debug("INPUT FILTERED FROM UART IS SHORTER THAN EXPLICIT RESPONSE LENGTH");
        is_waiting_response=false;
        return;
    }

    uint8_t payload_len = response_buffer[3];

    if (inline_buffer_length < payload_len + response_header_size) {
        // Not all response bytes received yet, splitted response is incoming
        debug("RESPONSE SEEMS NOT TO BE COMPLETE WAITING IN OTHER LOOP CYCLE ?");
        return;
    } else {
        inline_buffer_length = payload_len + response_header_size;
    }
    is_waiting_response=false;

    debug(" PARSING FRAME RESPONSE");
    parse_frame_response(response_buffer);
    debug(" PARSING FRAME RESPONSE ENDED");

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

        smartaudioSettings_t vtx_settings;
        // process response buffer
        if (!smartaudioParseResponseBuffer(&vtx_settings, buffer)){
            debug("%80s", "Unparseable buffer response");
            return false;
        }

        debug("%80s %02X", "selecting update path using:", vtx_settings.update_flags);
        // if partial updates because setters method to vtx.
        if (vtx_settings.update_flags!=0x0F){
            debug("%80s", "parse_frame_response:: overall update flow path");

            smartaudioSettings_t current_vtx_settings;

            // take the current vtx info
            vtx_states_queue.peek(&current_vtx_settings, 1);

            // get update flags from the parsed response_buffer
            current_vtx_settings.update_flags=vtx_settings.update_flags;

            // freq has changed
            if ( vtx_settings.update_flags & (1 << 0)){
                debug("%80s", "parse_frame_response:: freq update flow path");
                current_vtx_settings.frequency=vtx_settings.frequency;
                current_vtx_settings.userFrequencyMode=vtx_settings.userFrequencyMode;
            }

            // channel has changed
            if ( vtx_settings.update_flags & (1 << 1)){
                debug("%80s", "parse_frame_response:: band chan update flow path");
                current_vtx_settings.band=vtx_settings.channel/8;
                current_vtx_settings.channel=vtx_settings.channel%8;
            }

            // power has changed
            if ( vtx_settings.update_flags & (1 << 2)){
                debug("%80s", "parse_frame_response:: pow update flow path");
                current_vtx_settings.power=vtx_settings.power;

                if (current_vtx_settings.version==SMARTAUDIO_SPEC_PROTOCOL_v21){
                    current_vtx_settings.power_in_dbm=vtx_settings.power;
                }

            }

            // mode has changed
            if ( vtx_settings.update_flags & (1 << 3)){
                debug("%80s", "parse_frame_response:: mode update flow path");
                current_vtx_settings.pitModeRunning=vtx_settings.pitModeRunning;
                current_vtx_settings.pitmodeInRangeActive=vtx_settings.pitmodeInRangeActive;
                current_vtx_settings.pitmodeOutRangeActive=vtx_settings.pitmodeOutRangeActive;
                current_vtx_settings.unlocked=vtx_settings.unlocked;
            }

            vtx_settings=current_vtx_settings;

        }

        // update bands and channels accordily when frequency changes
        if ((vtx_settings.update_flags & 1<<0) && vtx_settings.userFrequencyMode){
            debug("%80s", "parse_frame_response:: band update in user freq mode");
            AP_VideoTX::VideoBand video_band;

            AP_VideoTX::get_band_and_channel(vtx_settings.frequency, video_band, vtx_settings.channel);

            vtx_settings.band=(int)video_band;
        }

        // update band and frequency accorly when channel updates
        if ( (vtx_settings.update_flags & 1<<1) && !vtx_settings.userFrequencyMode){
            debug("%80s", "parse_frame_response:: freq update from band and chan settings");
            vtx_settings.frequency=AP_VideoTX::get_frequency_mhz(vtx_settings.band, vtx_settings.channel);
        }

        // reset vtx_settings_change_control variables
        vtx_settings.update_flags=0x00;

        if (vtx_states_queue.is_empty()){
            debug("%80s", "parse_frame_response:: insert state into ringbuffer");
            vtx_states_queue.push_force(vtx_settings);
        }else{
            debug("%80s", "parse_frame_response:: insert second state into ringbuffer and advance");
            vtx_states_queue.push_force(vtx_settings);

            // advance to last element pushed
            vtx_states_queue.advance(1);
        }

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
   vtx_states_queue.peek(&current_state, 1);

   // setting frequency
    vtx_dest->set_frequency_mhz(current_state.frequency);

    // set channel
    vtx_dest->set_band((AP_VideoTX::VideoBand)((int)current_state.band));

    // setting channel 0 -> 40
    vtx_dest->set_channel(current_state.channel);

   // define vtx options TODO: Review this after define policies
   // setup default value for options
    vtx_dest->set_options(0);

      // pitmode enabled
   if (current_state.pitmodeOutRangeActive ||  current_state.pitmodeInRangeActive || !current_state.pitModeRunning){
       vtx_dest->set_options(1);
   }
    // pitmode disabled only by this option
   if (current_state.pitModeRunning){
       vtx_dest->set_options(0);
   }

   // locking status
   vtx_dest->set_locking(current_state.unlocked==0?1:0);

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

   return !vtx_states_queue.is_empty();
}

// utility method to get power in dbm mapping to power levels
uint8_t AP_SmartAudio::_get_power_in_dbm_from_vtx_power_level(uint8_t power_level, uint8_t& protocol_version)
{
    uint8_t power_in_dbm=0;
    return _get_power_in_dbm_from_vtx_power_level(power_level, protocol_version, power_in_dbm);
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
    debug("%80s::request_settings()\t", TAG);
    smartaudioFrame_t request;
    uint8_t frame_size=smartaudioFrameGetSettings(&request);
    Packet command;
    command.frame=request;
    command.frame_size=frame_size;
    requests_queue.push_force(command);
}


void AP_SmartAudio::set_operation_mode(uint8_t mode){
     // take the first register from the output buffer
   smartaudioSettings_t current_state;

    // peek from buffer
   vtx_states_queue.peek(&current_state, 1);

    debug("%80s::set_operation_mode(%02X)\t", TAG, mode);
    // SPEC SAYS ONLY V2 SUPPORT SET MODE BUT THIS INCLUDES 2.1 ?
    if (current_state.version<SMARTAUDIO_SPEC_PROTOCOL_v2) {
        debug("%s HW: %s", TAG, "Device can't change operation mode. Spec protocol not supported");
        return;
    }
    smartaudioSettings_t settings;
    settings.pitmodeInRangeActive=mode & 1<<0;
    settings.pitmodeOutRangeActive=mode & 1<<1;
    settings.pitModeRunning=mode & 1<<2;
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
    debug("%80s::set_frequency(%d, %d)\t", TAG, frequency, isPitModeFreq);
    smartaudioFrame_t request;
    uint8_t frame_size=smartaudioFrameSetFrequency(&request, frequency, isPitModeFreq);
    Packet command;
    command.frame=request;
    command.frame_size=frame_size;
    requests_queue.push_force(command);
}

// enqueue a set channel request
void AP_SmartAudio::set_channel(uint8_t chan_idx){
        debug("%80s::set_channel(%d)\t", TAG, chan_idx);
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
    debug("%80s::", "request_pit_mode_frequency()\t");
    smartaudioFrame_t request;
    uint8_t frame_size=smartaudioFrameGetPitmodeFrequency(&request);
    Packet command;
    command.frame=request;
    command.frame_size=frame_size;
    requests_queue.push_force(command);
}


// send vtx request to set power defined in dbm
void AP_SmartAudio::set_power_dbm(uint8_t power)
{
      // take the first register from the output buffer
   smartaudioSettings_t current_state;

    // peek from buffer
   vtx_states_queue.peek(&current_state, 1);
    smartaudioFrame_t request;
    uint8_t frame_size=0;

    debug("%80s::set_power_dbm(%d)\t", TAG, power);
    frame_size=smartaudioFrameSetPower(&request, AP_SmartAudio::_get_power_level_from_dbm(current_state.version, power));
     Packet command;
     command.frame=request;
     command.frame_size=frame_size;
     requests_queue.push_force(command);
}

// send vtx request to set power defined in mw
void AP_SmartAudio::set_power_mw(uint16_t power_mw){
    set_power_dbm(_get_power_in_dbm_from_mw(power_mw));
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
    operationMode |= settings->pitmodeInRangeActive << 0;
    operationMode |= settings->pitmodeOutRangeActive << 1;
    operationMode |= settings->pitModeRunning << 2;
    operationMode |= settings->unlocked << 3;
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

void AP_SmartAudio::_print_bytes_to_hex_string(uint8_t buf[], uint8_t x)
{
    int i;
    for (i = 0; i < x; i++) {
        if (i > 0) {
            debug(":");
            break;
        }
        debug("%02X", buf[i]);
    }
    debug("\n");
}

bool  AP_SmartAudio::smartaudioParseResponseBuffer(smartaudioSettings_t *settings, const uint8_t *buffer)
{
    const smartaudioFrameHeader_t *header = (const smartaudioFrameHeader_t *)buffer;
    const uint8_t fullFrameLength = sizeof(smartaudioFrameHeader_t) + header->length;
    const uint8_t headerPayloadLength = fullFrameLength - 1; // subtract crc byte from length
    const uint8_t *startPtr = buffer + 2;
    const uint8_t *endPtr = buffer + headerPayloadLength;

    if (crc8_dvb_s2_update(0x00, startPtr, headerPayloadLength-2)!=*(endPtr) || header->headerByte != SMARTAUDIO_HEADER_BYTE || header->syncByte!=SMARTAUDIO_SYNC_BYTE) {
        debug(" PARSE RESPONSE bad frame response crc check %02X", *endPtr);
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

