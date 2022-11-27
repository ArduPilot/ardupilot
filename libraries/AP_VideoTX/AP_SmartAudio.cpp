/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   SmartAudio protocol parsing and data structures taken from betaflight
 */

#include "AP_SmartAudio.h"
#include <AP_Math/crc.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_SerialManager/AP_SerialManager.h>

#if HAL_SMARTAUDIO_ENABLED

#ifdef SA_DEBUG
# define debug(fmt, args...)	do { hal.console->printf("SA: " fmt "\n", ##args); } while (0)
#else
# define debug(fmt, args...)	do {} while(0)
#endif

extern const AP_HAL::HAL &hal;

AP_SmartAudio::AP_SmartAudio()
{
    _singleton = this;
}

AP_SmartAudio *AP_SmartAudio::_singleton;

// initialization start making a request settings to the vtx
bool AP_SmartAudio::init()
{
    debug("SmartAudio init");

    if (AP::vtx().get_enabled()==0) {
        debug("SmartAudio protocol it's not active");
        return false;
    }

    // init uart
    _port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_SmartAudio, 0);
    if (_port!=nullptr) {
        _port->configure_parity(0);
        _port->set_stop_bits(AP::vtx().has_option(AP_VideoTX::VideoOptions::VTX_SA_ONE_STOP_BIT) ? 1 : 2);
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        _port->set_options((_port->get_options() & ~AP_HAL::UARTDriver::OPTION_RXINV)
            | AP_HAL::UARTDriver::OPTION_HDPLEX | AP_HAL::UARTDriver::OPTION_PULLDOWN_TX | AP_HAL::UARTDriver::OPTION_PULLDOWN_RX);
        if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_SmartAudio::loop, void),
                                          "SmartAudio",
                                          768, AP_HAL::Scheduler::PRIORITY_IO, -1)) {
            return false;
        }

        return true;
    }
    return false;
}

void AP_SmartAudio::loop()
{
    AP_VideoTX &vtx = AP::vtx();

    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay(100);
    }

    // allocate response buffer
    uint8_t _response_buffer[AP_SMARTAUDIO_MAX_PACKET_SIZE];

    // initialise uart (this must be called from within tick b/c the UART begin must be called from the same thread as it is used from)
    _port->begin(_smartbaud, AP_SMARTAUDIO_UART_BUFSIZE_RX, AP_SMARTAUDIO_UART_BUFSIZE_TX);


    while (true) {
        // now time to control loop switching
        uint32_t now = AP_HAL::millis();

        // when pending request and last request sended is timeout, take another packet to send
        if (!_is_waiting_response) {
            // command to process
            Packet current_command;

            // repeatedly initialize UART until we know what the VTX is
            if (!_initialised) {
                // request settings every second
                if (requests_queue.is_empty() && !hal.util->get_soft_armed() && now - _last_request_sent_ms > 1000) {
                    request_settings();
                }
            }

            if (requests_queue.pop(current_command)) {
                // send the popped command from bugger
                send_request(current_command.frame, current_command.frame_size);

                now = AP_HAL::millis();
                // it takes roughly 15ms to send a request, don't turn around and try and read until
                // this time has elapsed
                hal.scheduler->delay(20);

                _last_request_sent_ms = now;

                // next loop we expect a response
                _is_waiting_response = true;
            }
        }

        // nothing going on so give CPU to someone else
        if (!_is_waiting_response || !_initialised) {
            hal.scheduler->delay(100);
        }

        // On my Unify Pro32 the SmartAudio response is sent exactly 100ms after the request
        // and the initial response is 40ms long so we should wait at least 140ms before giving up
        if (now - _last_request_sent_ms < 200 && _is_waiting_response) {

            // setup scheduler delay to 50 ms again after response processes
            if (!read_response(_response_buffer)) {
                hal.scheduler->delay(10);
            } else {
                // successful response, wait another 100ms to give the VTX a chance to recover
                // before sending another command. This is needed on the Atlatl v1.
                hal.scheduler->delay(100);
            }

        } else if (_is_waiting_response) { // timeout
            // process autobaud routine
            update_baud_rate();
            _port->discard_input();
            _inline_buffer_length = 0;
            _is_waiting_response = false;
            debug("response timeout");
        } else if (_initialised) {
            if (AP::vtx().have_params_changed() ||_vtx_power_change_pending
                || _vtx_freq_change_pending || _vtx_options_change_pending) {
                update_vtx_params();
                set_configuration_pending(true);
                vtx.set_configuration_finished(false);
                // we've tried to update something, re-request the settings so that they
                // are reflected correctly
                request_settings();
            } else if (is_configuration_pending()) {
                AP::vtx().announce_vtx_settings();
                set_configuration_pending(false);
                vtx.set_configuration_finished(true);
            }
        }
    }
}

// send requests to the VTX to match the configured VTX parameters
void AP_SmartAudio::update_vtx_params()
{
    AP_VideoTX& vtx = AP::vtx();

    _vtx_freq_change_pending = vtx.update_band() || vtx.update_channel() || vtx.update_frequency() || _vtx_freq_change_pending;
    _vtx_power_change_pending = vtx.update_power() || _vtx_power_change_pending;
    _vtx_options_change_pending = vtx.update_options() || _vtx_options_change_pending;

    if (_vtx_freq_change_pending || _vtx_power_change_pending || _vtx_options_change_pending) {
        // make the desired frequency match the desired band and channel
        if (_vtx_freq_change_pending) {
            if (vtx.update_band() || vtx.update_channel()) {
                vtx.update_configured_frequency();
            } else {
                vtx.update_configured_channel_and_band();
            }
        }

        debug("update_params(): freq %d->%d, chan: %d->%d, band: %d->%d, pwr: %d->%d, opts: %d->%d",
            vtx.get_frequency_mhz(),  vtx.get_configured_frequency_mhz(),
            vtx.get_channel(), vtx.get_configured_channel(),
            vtx.get_band(), vtx.get_configured_band(),
            vtx.get_power_mw(), vtx.get_configured_power_mw(),
            vtx.get_options() & 0xF, vtx.get_configured_options() & 0xF);

        uint8_t opts = vtx.get_configured_options();
        uint8_t pitMode = vtx.get_configured_pitmode();
        uint8_t mode;
        // check if we are turning pitmode on or off, but only on SA 2.0+
        if (pitMode != vtx.get_pitmode() && _protocol_version >= SMARTAUDIO_SPEC_PROTOCOL_v2) {
            if (vtx.get_pitmode()) {
                debug("Turning OFF pitmode");
                // turn it off
                mode = 0x04 | ((opts & uint8_t(AP_VideoTX::VideoOptions::VTX_UNLOCKED)) << 2);
            } else {
                debug("Turning ON pitmode");
                // turn it on (in range pitmode flag)
                mode = 0x01 | ((opts & uint8_t(AP_VideoTX::VideoOptions::VTX_UNLOCKED)) << 2);
            }
        } else {
            mode = ((opts & uint8_t(AP_VideoTX::VideoOptions::VTX_UNLOCKED)) << 2);
            if (pitMode) {
                mode |= 0x01;
            }
        }

        if (pitMode) {// prevent power changes in pitmode as this takes the VTX out of pitmode
            _vtx_power_change_pending = false;
        }

        // prioritize pitmode changes
        if (_vtx_options_change_pending) {
            debug("update mode '%c%c%c%c'", (mode & 0x8) ? 'U' : 'L',
                (mode & 0x4) ? 'N' : ' ', (mode & 0x2) ? 'O' : ' ', (mode & 0x1) ? 'I' : ' ');
            set_operation_mode(mode);
        } else if (_vtx_freq_change_pending) {
            debug("update frequency");
            if (_vtx_use_set_freq) {
                set_frequency(vtx.get_configured_frequency_mhz(), false);
            } else {
                set_channel(vtx.get_configured_band() * VTX_MAX_CHANNELS + vtx.get_configured_channel());
            }
        } else if (_vtx_power_change_pending) {
            debug("update power (ver %u)", _protocol_version);
            switch (_protocol_version) {
            case SMARTAUDIO_SPEC_PROTOCOL_v21:
                set_power(vtx.get_configured_power_dbm() | 0x80);
                break;
            case SMARTAUDIO_SPEC_PROTOCOL_v2:
                set_power(vtx.get_configured_power_level());
                break;
            default:    // v1
                switch(vtx.get_configured_power_level()) {
                    case 1: set_power(16); break; // 200mw
                    case 2: set_power(25); break; // 500mw
                    case 3: set_power(40); break; // 800mw
                    default: set_power(7); break; // 25mw
                }
                break;
            }
        }
    } else {
        vtx.set_configuration_finished(true);
    }
}
/**
 * Sends an SmartAudio Command to the vtx, waits response on the update event
 * @param frameBuffer frameBuffer to send over the wire
 * @param size  size of the framebuffer wich needs to be sended
 */
void AP_SmartAudio::send_request(const Frame& requestFrame, uint8_t size)
{
    AP_VideoTX &vtx = AP::vtx();

    if (size <= 0 || _port == nullptr) {
        return;
    }

    const uint8_t *request = reinterpret_cast<const uint8_t*>(&requestFrame);

    // write request
    if (vtx.has_option(AP_VideoTX::VideoOptions::VTX_PULLDOWN)) {
        _port->write((uint8_t)0x00);
    }
    _port->write(request, size);
    _port->flush();

    _packets_sent++;
#ifdef SA_DEBUG
    print_bytes_to_hex_string("send_request():", request, size);
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

    const uint8_t response_header_size= sizeof(FrameHeader);

    // check if it is a response in the wire
    if (incoming_bytes_count <= 0) {
        return false;
    }

    // wait until we have enough bytes to read a header
    if (incoming_bytes_count < response_header_size && _inline_buffer_length == 0) {
        return false;
    }

    // now have at least the header, read it if necessary
    if (_inline_buffer_length == 0) {
        uint8_t b = _port->read();
        // didn't see a sync byte, discard and go around again
        if (b != SMARTAUDIO_SYNC_BYTE) {
            return false;
        }
        response_buffer[_inline_buffer_length++] = b;

        b = _port->read();
        // didn't see a header byte, discard and reset
        if (b != SMARTAUDIO_HEADER_BYTE) {
            _inline_buffer_length = 0;
            return false;
        }

        response_buffer[_inline_buffer_length++] = b;

        // read the rest of the header
        for (; _inline_buffer_length < response_header_size; _inline_buffer_length++) {
            b = _port->read();
            response_buffer[_inline_buffer_length] = b;
        }

        FrameHeader* header = (FrameHeader*)response_buffer;
        incoming_bytes_count -= response_header_size;

        // implementations that ignore the CRC also appear to not account for it in the frame length
        if (ignore_crc()) {
            header->length++;
        }
        _packet_size = header->length;
    }

    // read the rest of the packet
    for (uint8_t i= 0; i < incoming_bytes_count && _inline_buffer_length < _packet_size + response_header_size; i++) {
        uint8_t response_in_bytes = _port->read();

        // check for overflow
        if (_inline_buffer_length >= AP_SMARTAUDIO_MAX_PACKET_SIZE) {
            _inline_buffer_length = 0;
            _is_waiting_response = false;
            return false;
        }

        response_buffer[_inline_buffer_length++] = response_in_bytes;
    }

    // didn't get the whole packet
    if (_inline_buffer_length < _packet_size + response_header_size) {
        return false;
    }

#ifdef SA_DEBUG
    print_bytes_to_hex_string("read_response():", response_buffer, _inline_buffer_length);
#endif
    _is_waiting_response = false;

    bool correct_parse = parse_response_buffer(response_buffer);
    response_buffer = nullptr;
    _inline_buffer_length=0;
    _packet_size = 0;
    _packets_rcvd++;
    // reset the lost packets to 0
    _packets_sent =_packets_rcvd;
    return correct_parse;
}

// format a simple command and push into the request queue
void AP_SmartAudio::push_command_only_frame(uint8_t cmd_id)
{
    Packet command;
    // according to the spec the length should include the CRC, but no implementation appears to
    // do this
    command.frame.header.init(cmd_id, 0);
    command.frame_size = SMARTAUDIO_COMMAND_FRAME_SIZE;
    command.frame.payload[0] = crc8_dvb_s2_update(0, &command.frame, SMARTAUDIO_COMMAND_FRAME_SIZE - 1);
    requests_queue.push_force(command);
}

// format an 8-bit command and push into the request queue
void AP_SmartAudio::push_uint8_command_frame(uint8_t cmd_id, uint8_t data)
{
    Packet command;
    command.frame.header.init(cmd_id, sizeof(uint8_t));
    command.frame_size = SMARTAUDIO_U8_COMMAND_FRAME_SIZE;

    command.frame.payload[0] = data;
    command.frame.payload[1] = crc8_dvb_s2_update(0, &command.frame, SMARTAUDIO_U8_COMMAND_FRAME_SIZE - 1);
    requests_queue.push_force(command);
}

// format an 16-bit command and push into the request queue
void AP_SmartAudio::push_uint16_command_frame(uint8_t cmd_id, uint16_t data)
{
    Packet command;
    command.frame.header.init(cmd_id, sizeof(uint16_t));
    command.frame_size = SMARTAUDIO_U16_COMMAND_FRAME_SIZE;
    put_be16_ptr(command.frame.payload, data);
    command.frame.payload[2] = crc8_dvb_s2_update(0, &command.frame, SMARTAUDIO_U16_COMMAND_FRAME_SIZE - 1);
    requests_queue.push_force(command);
}

/**
 * Sends get settings command.
 * */
void AP_SmartAudio::request_settings()
{
    debug("request_settings()");
    push_command_only_frame(SMARTAUDIO_CMD_GET_SETTINGS);
}

void AP_SmartAudio::set_operation_mode(uint8_t mode)
{
    debug("Setting mode to 0x%x", mode);
    push_uint8_command_frame(SMARTAUDIO_CMD_SET_MODE, mode);
}

/**
     * Sets the frequency to transmit in the vtx.
     * When isPitModeFreq active the freq will be set to be used when in pitmode (in range)
     */
void AP_SmartAudio::set_frequency(uint16_t frequency, bool isPitModeFreq)
{
    debug("Setting frequency to %d with pitmode == %d", frequency, isPitModeFreq);
    push_uint16_command_frame(SMARTAUDIO_CMD_SET_FREQUENCY,
        frequency | (isPitModeFreq ? SMARTAUDIO_SET_PITMODE_FREQ : 0x00));
}

// enqueue a set channel request
void AP_SmartAudio::set_channel(uint8_t channel)
{
    debug("Setting channel to %d", channel);
    push_uint8_command_frame(SMARTAUDIO_CMD_SET_CHANNEL, channel);
}

/**
 * Request pitMode Frequency setted into the vtx hardware
 * */
void AP_SmartAudio::request_pit_mode_frequency()
{
    debug("Requesting pit mode frequency");
    push_uint16_command_frame(SMARTAUDIO_CMD_SET_FREQUENCY, SMARTAUDIO_GET_PITMODE_FREQ);
}

// send vtx request to set power
void AP_SmartAudio::set_power(uint8_t power_level)
{
    debug("Setting power to %d", power_level);
    push_uint8_command_frame(SMARTAUDIO_CMD_SET_POWER, power_level);
}


void AP_SmartAudio::set_band_channel(const uint8_t band, const uint8_t channel)
{
    debug("Setting band/channel to %d/%d", band, channel);
    push_uint16_command_frame(SMARTAUDIO_CMD_SET_CHANNEL, SMARTAUDIO_BANDCHAN_TO_INDEX(band, channel));
}

void AP_SmartAudio::unpack_frequency(AP_SmartAudio::Settings *settings, const uint16_t frequency)
{
    if (frequency & SMARTAUDIO_GET_PITMODE_FREQ) {
        settings->pitmodeFrequency = frequency;
    } else {
        settings->frequency = frequency;
    }
}

// SmartAudio v1/v2
void AP_SmartAudio::unpack_settings(Settings *settings, const SettingsResponseFrame *frame)
{
    settings->channel = frame->channel % VTX_MAX_CHANNELS;
    settings->band = frame->channel / VTX_MAX_CHANNELS;
    settings->power = frame->power;
    settings->mode = frame->operationMode;
    settings->num_power_levels = 0;
    unpack_frequency(settings, be16toh(frame->frequency));
}

// SmartAudio v2.1
void AP_SmartAudio::unpack_settings(Settings *settings, const SettingsExtendedResponseFrame *frame)
{
    unpack_settings(settings, &frame->settings);
    settings->power_in_dbm = frame->power_dbm;
    settings->num_power_levels = frame->num_power_levels + 1;
    memcpy(settings->power_levels, frame->power_levels, frame->num_power_levels + 1);
}

#ifdef SA_DEBUG
void AP_SmartAudio::print_bytes_to_hex_string(const char* msg, const uint8_t buf[], uint8_t len)
{
    hal.console->printf("SA: %s ", msg);
    for (uint8_t i = 0; i < len; i++) {
        hal.console->printf("0x%02X ", buf[i]);
    }
    hal.console->printf("\n");
}
#endif

void AP_SmartAudio::print_settings(const Settings* settings)
{
    debug("SETTINGS: VER: %u, MD: '%c%c%c%c%c', CH: %u, PWR: %u, DBM: %u FREQ: %u, BND: %u",
            settings->version,
            (settings->mode & 0x10) ? 'U' : 'L',// (L)ocked or (U)nlocked
            (settings->mode & 0x8) ? 'O' : ' ', // (O)ut-range pitmode
            (settings->mode & 0x4) ? 'I' : ' ', // (I)n-range pitmode
            (settings->mode & 0x2) ? 'P' : ' ', // (P)itmode running
            (settings->mode & 0x1) ? 'F' : 'C', // Set (F)requency or (C)hannel
            settings->channel, settings->power, settings->power_in_dbm, settings->frequency, settings->band);
}

void AP_SmartAudio::update_vtx_settings(const Settings& settings)
{
    AP_VideoTX& vtx = AP::vtx();

    vtx.set_enabled(true);
    vtx.set_frequency_mhz(settings.frequency);
    vtx.set_band(settings.band);
    vtx.set_channel(settings.channel);
    // SA21 sends us a complete packet with the supported power levels
    if (settings.version == SMARTAUDIO_SPEC_PROTOCOL_v21) {
        vtx.set_power_dbm(settings.power_in_dbm);
        // learn them all
        vtx.update_all_power_dbm(settings.num_power_levels, settings.power_levels);
    } else {
        vtx.set_power_level(settings.power, AP_VideoTX::PowerActive::Active);
    }
    // it seems like the spec is wrong, on a unify pro32 this setting is inverted
    _vtx_use_set_freq = !(settings.mode & 1);

    // PITMODE | UNLOCKED
    // SmartAudio 2.1 dropped support for outband pitmode so we won't support it
    uint8_t opts = ((settings.mode & 0x2) >> 1) | ((settings.mode & 0x10) >> 1);
    vtx.set_options(opts);

    // make sure the configured values now reflect reality
    vtx.set_defaults();

    _initialised = true;

    _vtx_power_change_pending = _vtx_freq_change_pending = _vtx_options_change_pending = false;
}

bool  AP_SmartAudio::parse_response_buffer(const uint8_t *buffer)
{
    const FrameHeader *header = (const FrameHeader *)buffer;
    const uint8_t fullFrameLength = sizeof(FrameHeader) + header->length;
    const uint8_t headerPayloadLength = fullFrameLength - 1; // subtract crc byte from length
    const uint8_t *startPtr = buffer + 2;
    const uint8_t *endPtr = buffer + headerPayloadLength;

    if ((crc8_dvb_s2_update(0x00, startPtr, headerPayloadLength-2)!=*(endPtr) && !ignore_crc())
        || header->headerByte != SMARTAUDIO_HEADER_BYTE
        || header->syncByte != SMARTAUDIO_SYNC_BYTE) {
        debug("parse_response_buffer() failed - invalid CRC or header");
        return false;
    }
    // SEND TO GCS A MESSAGE TO UNDERSTAND WHATS HAPPENING
    AP_VideoTX& vtx = AP::vtx();
    Settings settings {};

    switch (header->command) {
    case SMARTAUDIO_RSP_GET_SETTINGS_V1:
        _protocol_version = SMARTAUDIO_SPEC_PROTOCOL_v1;
        unpack_settings(&settings, (const SettingsResponseFrame *)buffer);
        settings.version = SMARTAUDIO_SPEC_PROTOCOL_v1;
        print_settings(&settings);
        update_vtx_settings(settings);
        break;

    case SMARTAUDIO_RSP_GET_SETTINGS_V2:
        _protocol_version = SMARTAUDIO_SPEC_PROTOCOL_v2;
        unpack_settings(&settings, (const SettingsResponseFrame *)buffer);
        settings.version = SMARTAUDIO_SPEC_PROTOCOL_v2;
        print_settings(&settings);
        update_vtx_settings(settings);
        break;

    case SMARTAUDIO_RSP_GET_SETTINGS_V21:
        _protocol_version = SMARTAUDIO_SPEC_PROTOCOL_v21;
        unpack_settings(&settings, (const SettingsExtendedResponseFrame *)buffer);
        settings.version = SMARTAUDIO_SPEC_PROTOCOL_v21;
        print_settings(&settings);
        update_vtx_settings(settings);
        break;

    case SMARTAUDIO_RSP_SET_FREQUENCY: {
        const U16ResponseFrame *resp = (const U16ResponseFrame *)buffer;
        unpack_frequency(&settings, resp->payload);
        vtx.set_frequency_mhz(settings.frequency);
        vtx.set_configured_frequency_mhz(vtx.get_frequency_mhz());
        vtx.update_configured_channel_and_band();
        debug("Frequency was set to %d", settings.frequency);
    }
        break;

    case SMARTAUDIO_RSP_SET_CHANNEL: {
        const U8ResponseFrame *resp = (const U8ResponseFrame *)buffer;
        vtx.set_band(resp->payload / VTX_MAX_CHANNELS);
        vtx.set_channel(resp->payload % VTX_MAX_CHANNELS);
        vtx.set_configured_channel(vtx.get_channel());
        vtx.set_configured_band(vtx.get_band());
        vtx.update_configured_frequency();
        debug("Channel was set to %d", resp->payload);
    }
        break;

    case SMARTAUDIO_RSP_SET_POWER: {
        const U16ResponseFrame *resp = (const U16ResponseFrame *)buffer;
        const uint8_t power = resp->payload & 0xFF;
        switch (_protocol_version) {
        case SMARTAUDIO_SPEC_PROTOCOL_v21:
            if (vtx.get_configured_power_dbm() != power) {
                vtx.update_power_dbm(vtx.get_configured_power_dbm(), AP_VideoTX::PowerActive::Inactive);
            }
            vtx.set_power_dbm(power);
            vtx.set_configured_power_mw(vtx.get_power_mw());
            break;
        case SMARTAUDIO_SPEC_PROTOCOL_v2:
            if (vtx.get_configured_power_level() != power) {
                vtx.update_power_dbm(vtx.get_configured_power_dbm(), AP_VideoTX::PowerActive::Inactive);
            }
            vtx.set_power_level(power);
            vtx.set_configured_power_mw(vtx.get_power_mw());
            break;
        default:
            if (vtx.get_configured_power_dac() != power) {
                vtx.update_power_dbm(vtx.get_configured_power_dbm(), AP_VideoTX::PowerActive::Inactive);
            }
            vtx.set_power_dac(power);
            vtx.set_configured_power_mw(vtx.get_power_mw());
            break;
        }
        debug("Power was set to %d", power);
    }
        break;

    case SMARTAUDIO_RSP_SET_MODE: {
        vtx.set_options(vtx.get_configured_options()); // easiest to just make them match
        debug("Mode was set to 0x%x", buffer[4]);
    }
        break;

    default:
        return false;
    }
    return true;
}

// we missed a response too many times - update the baud rate in case the temperature has increased
void AP_SmartAudio::update_baud_rate()
{
    // on my Unify Pro32 the VTX will respond immediately on power up to a settings request, so 10 packets is easily more than enough
    // we want to bias autobaud to only frequency hop when the current frequency is clearly exhausted, but after that hop quickly
    if (_packets_sent - _packets_rcvd < 10) {
        return;
    }

    if ((_smartbaud_direction == 1) && (_smartbaud == AP_SMARTAUDIO_SMARTBAUD_MAX)) {
        _smartbaud_direction = -1;
    } else if ((_smartbaud_direction == -1 && _smartbaud == AP_SMARTAUDIO_SMARTBAUD_MIN)) {
        _smartbaud_direction = 1;
    }

    _smartbaud += AP_SMARTAUDIO_SMARTBAUD_STEP * _smartbaud_direction;

    debug("autobaud: %d", int(_smartbaud));

    _port->begin(_smartbaud);
}

#endif // HAL_SMARTAUDIO_ENABLED
