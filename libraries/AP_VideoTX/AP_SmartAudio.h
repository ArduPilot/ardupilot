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
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

#ifndef HAL_SMARTAUDIO_ENABLED
#define HAL_SMARTAUDIO_ENABLED !HAL_MINIMIZE_FEATURES
#endif

#if HAL_SMARTAUDIO_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/RingBuffer.h>
#include "AP_VideoTX.h"

#define SMARTAUDIO_BUFFER_CAPACITY 5

// SmartAudio Serial Protocol
#define AP_SMARTAUDIO_UART_BAUD            4800
#define AP_SMARTAUDIO_SMARTBAUD_MIN        4560     // -5%
#define AP_SMARTAUDIO_SMARTBAUD_MAX        5040     // +5%
#define AP_SMARTAUDIO_SMARTBAUD_STEP       60
#define AP_SMARTAUDIO_UART_BUFSIZE_RX      16
#define AP_SMARTAUDIO_UART_BUFSIZE_TX      16
#define AP_SMARTAUDIO_MAX_PACKET_SIZE      32

#define SMARTAUDIO_SYNC_BYTE            0xAA
#define SMARTAUDIO_HEADER_BYTE          0x55
#define SMARTAUDIO_START_CODE           SMARTAUDIO_SYNC_BYTE + SMARTAUDIO_HEADER_BYTE
#define SMARTAUDIO_GET_PITMODE_FREQ     (1 << 14)
#define SMARTAUDIO_SET_PITMODE_FREQ     (1 << 15)
#define SMARTAUDIO_FREQUENCY_MASK       0x3FFF

#define SMARTAUDIO_CMD_GET_SETTINGS     0x03
#define SMARTAUDIO_CMD_SET_POWER        0x05
#define SMARTAUDIO_CMD_SET_CHANNEL      0x07
#define SMARTAUDIO_CMD_SET_FREQUENCY    0x09
#define SMARTAUDIO_CMD_SET_MODE         0x0B

#define SMARTAUDIO_COMMAND_FRAME_SIZE   (sizeof(FrameHeader) + 1)
#define SMARTAUDIO_U8_COMMAND_FRAME_SIZE   (sizeof(FrameHeader) + 2)
#define SMARTAUDIO_U16_COMMAND_FRAME_SIZE   (sizeof(FrameHeader) + 3)

#define SMARTAUDIO_RSP_GET_SETTINGS_V1  SMARTAUDIO_CMD_GET_SETTINGS >> 1
#define SMARTAUDIO_RSP_GET_SETTINGS_V2  (SMARTAUDIO_CMD_GET_SETTINGS >> 1) | 0x08
#define SMARTAUDIO_RSP_GET_SETTINGS_V21 (SMARTAUDIO_CMD_GET_SETTINGS >> 1) | 0x10
#define SMARTAUDIO_RSP_SET_POWER        SMARTAUDIO_CMD_SET_POWER >> 1
#define SMARTAUDIO_RSP_SET_CHANNEL      SMARTAUDIO_CMD_SET_CHANNEL >> 1
#define SMARTAUDIO_RSP_SET_FREQUENCY    SMARTAUDIO_CMD_SET_FREQUENCY >> 1
#define SMARTAUDIO_RSP_SET_MODE         SMARTAUDIO_CMD_SET_MODE >> 1

#define SMARTAUDIO_BANDCHAN_TO_INDEX(band, channel) (band * VTX_MAX_CHANNELS + (channel))

// #define SA_DEBUG

class AP_SmartAudio
{
public:
    enum ProtocolVersion {
        SMARTAUDIO_SPEC_PROTOCOL_v1 = 0,
        SMARTAUDIO_SPEC_PROTOCOL_v2 = 1,
        SMARTAUDIO_SPEC_PROTOCOL_v21 = 2
    };

    struct Settings {
        uint8_t  version;
        uint8_t  mode;
        uint8_t  channel;
        uint8_t  power;
        uint16_t frequency;
        uint8_t  band;

        uint8_t* power_levels;
        uint8_t  power_in_dbm;

        uint16_t pitmodeFrequency;
        bool userFrequencyMode;     // user is setting freq
        bool initialized;
    };

    struct FrameHeader {
        uint8_t syncByte;
        uint8_t headerByte;
        uint8_t command;
        uint8_t length;

        void init(uint8_t cmd, uint8_t payloadLength) {
            syncByte = SMARTAUDIO_SYNC_BYTE;
            headerByte= SMARTAUDIO_HEADER_BYTE;
            length = payloadLength;
            command = cmd;
        }
    } PACKED;

    struct Frame {
        FrameHeader header;
        uint8_t payload[3];
    } PACKED;

    struct U8ResponseFrame {
        FrameHeader header;
        uint8_t payload;
        //uint8_t reserved;
        //uint8_t crc;
    } PACKED;

    struct U16ResponseFrame {
        FrameHeader header;
        uint16_t payload;
        //uint8_t reserved;
        //uint8_t crc;
    } PACKED;

    struct SettingsResponseFrame {
        FrameHeader header;
        uint8_t channel;
        uint8_t power;
        uint8_t operationMode;
        uint16_t frequency;
        //uint8_t crc;
    } PACKED;

    struct SettingsExtendedResponseFrame {
        FrameHeader header;
        uint8_t channel;
        uint8_t power;
        uint8_t operationMode;
        uint16_t frequency;
        uint8_t power_dbm;
        uint8_t power_levels_len;
        uint8_t power_dbm_levels;   // first in the list of dbm levels
        //uint8_t crc;
    } PACKED;

    // v 2.1 additions to response frame
    //0x0E (current power in dBm) 0x03 (amount of power levels) 0x00(dBm level 1) 0x0E (dBm level 2) 0x14 (dBm level 3) 0x1A (dBm level 4) 0x01(CRC8)

    // request packet to be processed
    struct Packet {
        Frame frame;
        uint8_t frame_size;
    } PACKED;

    AP_SmartAudio();

    static AP_SmartAudio *get_singleton(void)
    {
        return _singleton;
    }

    /* Do not allow copies */
    AP_SmartAudio(const AP_SmartAudio &other) = delete;

    AP_SmartAudio &operator=(const AP_SmartAudio&) = delete;

    // init threads and lookup for io uart.
    bool init();

private:
    // serial interface
    AP_HAL::UARTDriver *_port;                  // UART used to send data to SmartAudio VTX

    //Pointer to singleton
    static AP_SmartAudio* _singleton;

    // response buffer length, permit splitted responses
    uint8_t _inline_buffer_length;
    // expected packet size
    uint8_t _packet_size;

    ProtocolVersion _protocol_version;
    // statistics
    uint16_t _packets_sent;
    uint16_t _packets_rcvd;

    bool _vtx_freq_change_pending; // a vtx command has been issued but not confirmed by a vtx broadcast frame
    bool _vtx_power_change_pending;
    bool _vtx_options_change_pending;
    bool _vtx_changes_pending;
    bool _vtx_use_set_freq; // should frequency set by band/channel or frequency

    // value for current baud adjust
    int32_t _smartbaud = AP_SMARTAUDIO_UART_BAUD;
    int32_t _smartbaud_direction = 1;

    // hw vtx state control with 2 elements array use methods _push _peek
    uint8_t _vtx_state_idx;
    Settings _vtx_states_buffer[2];
    Settings *_vtx_current_state;

    // ready to go
    volatile bool _initialised;

    // RingBuffer to store outgoing request.
    ObjectBuffer<Packet> requests_queue{SMARTAUDIO_BUFFER_CAPACITY};

    // time the last_request is process
    uint32_t _last_request_sent_ms;

    // loops is waiting a response after a request
    bool _is_waiting_response;

#ifdef SA_DEBUG
    // utility method for debugging.
    void print_bytes_to_hex_string(const char* msg, const uint8_t buf[], uint8_t x,uint8_t offset);
#endif
    void print_settings(const Settings* settings);

    void update_vtx_params();
    void update_vtx_settings(const Settings& settings);

    // looping over requests
    void loop();
    // send a frame over the wire
    void send_request(const Frame& requestFrame, uint8_t size);
    // receive a frame response over the wire
    bool read_response(uint8_t *response_buffer);
    // parses the response and updates the vtx settings
    bool parse_frame_response(const uint8_t *buffer);
    bool parse_response_buffer(const uint8_t *buffer);
    // get last reading from the fifo queue
    bool get_readings(AP_VideoTX *vtx_dest);

    // command functions
    // request settings
    void request_settings();
    // change the mode
    void set_operation_mode(uint8_t mode);
    // change the frequency
    void set_frequency(uint16_t frequency, bool isPitModeFreq);
    // change the channel
    void set_channel(uint8_t chan);
    // get the pitmode frequency
    void request_pit_mode_frequency();
    // set the power
    void set_power(uint16_t power_mw, uint16_t dbm);
    // set the power using power_level, spec versions 1 and 2 or dbm value for spec version 2.1
    void set_power(uint8_t power_level);
    // set the band and channel
    void set_band_channel(const uint8_t band, const uint8_t channel);

    // command functions
    void push_command_only_frame(uint8_t command);
    void push_uint8_command_frame(uint8_t command, uint8_t data);
    void push_uint16_command_frame(uint8_t command, uint16_t data);

    static void unpack_frequency(Settings *settings, const uint16_t frequency);
    static void unpack_settings(Settings *settings, const SettingsResponseFrame *frame);
    static void unpack_settings(Settings *settings, const SettingsExtendedResponseFrame *frame);

    // change baud automatically when request-response fails many times
    void update_baud_rate();

    void set_configuration_pending(bool pending) { _vtx_changes_pending = pending; }
    bool is_configuration_pending(){ return _vtx_changes_pending;}
};
#endif
