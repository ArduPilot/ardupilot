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
/*
  GHST protocol decoder based on betaflight implementation
  Code by Andy Piper
 */

#include "AP_RCProtocol_config.h"

#if AP_RCPROTOCOL_GHST_ENABLED

#define CRSF_BAUDRATE      416666U

#include "AP_RCProtocol.h"
#include "AP_RCProtocol_GHST.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_RCTelemetry/AP_GHST_Telem.h>
#include <AP_SerialManager/AP_SerialManager.h>

/*
 * GHST protocol
 *
 * GHST protocol uses a single wire half duplex uart connection.
 *
 * 420000 baud
 * not inverted
 * 8 Bit
 * 1 Stop bit
 * Big endian
 * Max frame size is 14 bytes
 *
 * Every frame has the structure:
 * <Device address><Frame length><Type><Payload><CRC>
 *
 * Device address: (uint8_t)
 * Frame length:   length in  bytes including Type (uint8_t)
 * Type:           (uint8_t)
 * CRC:            (uint8_t)
 *
 */

extern const AP_HAL::HAL& hal;

//#define GHST_DEBUG
//#define GHST_DEBUG_CHARS
#ifdef GHST_DEBUG
# define debug(fmt, args...)	hal.console->printf("GHST: " fmt "\n", ##args)
static const char* get_frame_type(uint8_t byte, uint8_t subtype = 0)
{
    switch(byte) {
    case AP_RCProtocol_GHST::GHST_UL_RC_CHANS_HS4_5TO8:
    case AP_RCProtocol_GHST::GHST_UL_RC_CHANS_HS4_12_5TO8:
        return "RC5_8";
    case AP_RCProtocol_GHST::GHST_UL_RC_CHANS_HS4_9TO12:
    case AP_RCProtocol_GHST::GHST_UL_RC_CHANS_HS4_12_9TO12:
        return "RC9_12";
    case AP_RCProtocol_GHST::GHST_UL_RC_CHANS_HS4_13TO16:
    case AP_RCProtocol_GHST::GHST_UL_RC_CHANS_HS4_12_13TO16:
        return "RC13_16";
    case AP_RCProtocol_GHST::GHST_UL_RC_CHANS_RSSI:
    case AP_RCProtocol_GHST::GHST_UL_RC_CHANS_12_RSSI:
        return "RSSI";
    case AP_RCProtocol_GHST::GHST_UL_RC_VTX_CTRL:
        return "VTX_CTRL";
    case AP_RCProtocol_GHST::GHST_UL_VTX_SETUP:
        return "VTX_SETUP";
    }
    return "UNKNOWN";
}
#else
# define debug(fmt, args...)	do {} while(0)
#endif

#define GHST_MAX_FRAME_TIME_US       500U // 14 bytes @ 420k = ~450us
#define GHST_FRAME_TIMEOUT_US      10000U // 10ms to account for scheduling delays
#define GHST_INTER_FRAME_TIME_US    2000U // At fastest, frames are sent by the transmitter every 2 ms, 500 Hz
#define GHST_HEADER_TYPE_LEN     (GHST_HEADER_LEN + 1)           // header length including type

const uint16_t AP_RCProtocol_GHST::RF_MODE_RATES[RFMode::RF_MODE_MAX_MODES] = {
    55, 160, 250, 19, 250, 500, 150, 250,
};

AP_RCProtocol_GHST* AP_RCProtocol_GHST::_singleton;

AP_RCProtocol_GHST::AP_RCProtocol_GHST(AP_RCProtocol &_frontend) : AP_RCProtocol_Backend(_frontend)
{
#if !APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    if (_singleton != nullptr) {
        AP_HAL::panic("Duplicate GHST handler");
    }

    _singleton = this;
#else
    if (_singleton == nullptr) {
        _singleton = this;
    }
#endif
}

AP_RCProtocol_GHST::~AP_RCProtocol_GHST() {
    _singleton = nullptr;
}

// get the protocol string
const char* AP_RCProtocol_GHST::get_protocol_string() const {
    return "GHST";
}

// return the link rate as defined by the LinkStatistics
uint16_t AP_RCProtocol_GHST::get_link_rate() const {
    return RF_MODE_RATES[_link_status.rf_mode - GHST_RF_MODE_NORMAL];
}

void AP_RCProtocol_GHST::_process_byte(uint32_t timestamp_us, uint8_t byte)
{
    //debug("process_byte(0x%x)", byte);
    // we took too long decoding, start again - the RX will only send complete frames so this is unlikely to fail,
    // however thread scheduling can introduce longer delays even when the data has been received
    if (_frame_ofs > 0 && (timestamp_us - _start_frame_time_us) > GHST_FRAME_TIMEOUT_US) {
        _frame_ofs = 0;
    }

    // overflow check
    if (_frame_ofs >= GHST_FRAMELEN_MAX) {
        _frame_ofs = 0;
    }

    // start of a new frame
    if (_frame_ofs == 0) {
        _start_frame_time_us = timestamp_us;
    }

    add_to_buffer(_frame_ofs++, byte);

    // need a header to get the length
    if (_frame_ofs < GHST_HEADER_TYPE_LEN) {
        return;
    }

    if (_frame.device_address != DeviceAddress::GHST_ADDRESS_FLIGHT_CONTROLLER) {
        return;
    }

    // parse the length
    if (_frame_ofs == GHST_HEADER_TYPE_LEN) {
        _frame_crc = crc8_dvb_s2(0, _frame.type);
        // check for garbage frame
        if (_frame.length > GHST_FRAME_PAYLOAD_MAX) {
            _frame_ofs = 0;
        }
        return;
    }

    // update crc
    if (_frame_ofs < _frame.length + GHST_HEADER_LEN) {
        _frame_crc = crc8_dvb_s2(_frame_crc, byte);
    }

    // overflow check
    if (_frame_ofs > _frame.length + GHST_HEADER_LEN) {
        _frame_ofs = 0;
        return;
    }

    if (_frame.length < 2) {
        // invalid length, we subtract 2 below
        _frame_ofs = 0;
        return;
    }
    
    // decode whatever we got and expect
    if (_frame_ofs == _frame.length + GHST_HEADER_LEN) {
        log_data(AP_RCProtocol::GHST, timestamp_us, (const uint8_t*)&_frame, _frame_ofs - GHST_HEADER_LEN);

        // we consumed the partial frame, reset
        _frame_ofs = 0;

        // bad CRC (payload start is +1 from frame start, so need to subtract that from frame length to get index)
        if (_frame_crc != _frame.payload[_frame.length - 2]) {
            return;
        }

        _last_frame_time_us = _last_rx_frame_time_us = timestamp_us;
        // decode here
        if (decode_ghost_packet()) {
            _last_tx_frame_time_us = timestamp_us;  // we have received a frame from the transmitter
            add_input(MAX_CHANNELS, _channels, false, _link_status.rssi, _link_status.link_quality);
        }
    }
}

void AP_RCProtocol_GHST::update(void)
{
}

// write out a frame of any type
bool AP_RCProtocol_GHST::write_frame(Frame* frame)
{
    AP_HAL::UARTDriver *uart = get_current_UART();

    if (!uart) {
        return false;
    }

    // check that we haven't been too slow in responding to the new UART data. If we respond too late then we will
    // corrupt the next incoming control frame. incoming packets at max 126bits @500Hz @420k baud gives total budget of 2ms
    // per packet of which we need 300us to receive a packet. outgoing packets are 126bits which require 300us to send
    // leaving at most 1.4ms of delay that can be tolerated
    uint64_t tend = uart->receive_time_constraint_us(1);
    uint64_t now = AP_HAL::micros64();
    uint64_t tdelay = now - tend;
    if (tdelay > 1000) {
        // we've been too slow in responding
        return false;
    }

    // calculate crc
    uint8_t crc = crc8_dvb_s2(0, frame->type);
    for (uint8_t i = 0; i < frame->length - 2; i++) {
        crc = crc8_dvb_s2(crc, frame->payload[i]);
    }
    frame->payload[frame->length - 2] = crc;

    uart->write((uint8_t*)frame, frame->length + 2);
    uart->flush();

#ifdef GHST_DEBUG
    hal.console->printf("GHST: writing %s:", get_frame_type(frame->type, frame->payload[0]));
    for (uint8_t i = 0; i < frame->length + 2; i++) {
        uint8_t val = ((uint8_t*)frame)[i];
#ifdef GHST_DEBUG_CHARS
        if (val >= 32 && val <= 126) {
            hal.console->printf(" 0x%x '%c'", val, (char)val);
        } else {
#endif
            hal.console->printf(" 0x%x", val);
#ifdef GHST_DEBUG_CHARS
        }
#endif
    }
    hal.console->printf("\n");
#endif
    return true;
}

bool AP_RCProtocol_GHST::decode_ghost_packet()
{
#ifdef GHST_DEBUG
    hal.console->printf("GHST: received %s:", get_frame_type(_frame.type));
    uint8_t* fptr = (uint8_t*)&_frame;
    for (uint8_t i = 0; i < _frame.length + 2; i++) {
#ifdef GHST_DEBUG_CHARS
        if (fptr[i] >= 32 && fptr[i] <= 126) {
            hal.console->printf(" 0x%x '%c'", fptr[i], (char)fptr[i]);
        } else {
#endif
            hal.console->printf(" 0x%x", fptr[i]);
#ifdef GHST_DEBUG_CHARS
        }
#endif
    }
    hal.console->printf("\n");
#endif

    const RadioFrame* radio_frame = (const RadioFrame*)(&_frame.payload);
    const Channels12Bit_4Chan* channels = &(radio_frame->channels);
    const uint8_t* lowres_channels = radio_frame->lowres_channels;
    bool rc_frame = false;

    // Scaling from Betaflight
    // Scaling 12bit channels (8bit channels in brackets)
    //      OpenTx          RC   PWM (BF)
    // min  -1024        0(  0)     988us
    // ctr      0     2048(128)    1500us
    // max   1024     4096(256)    2012us
    //

    // Scaling legacy (nearly 10bit)
    // derived from original SBus scaling, with slight correction for offset
    // now symmetrical around OpenTx 0 value
    // scaling is:
    //      OpenTx         RC   PWM (BF)
    // min  -1024     172( 22)     988us
    // ctr      0     992(124)    1500us
    // max   1024    1811(226)    2012us

#define CHANNEL_RESCALE(x) (((5 * x) >> 2) - 430)
#define CHANNEL_SCALE(x) (int32_t(x) / 4 + 988)
#define CHANNEL_SCALE_LEGACY(x) CHANNEL_SCALE(CHANNEL_RESCALE(x))

    // legacy scaling
    if (_frame.type >= GHST_UL_RC_CHANS_HS4_5TO8 && _frame.type <= GHST_UL_RC_CHANS_RSSI) {
        _channels[0] = CHANNEL_SCALE_LEGACY(channels->ch0);
        _channels[1] = CHANNEL_SCALE_LEGACY(channels->ch1);
        _channels[2] = CHANNEL_SCALE_LEGACY(channels->ch2);
        _channels[3] = CHANNEL_SCALE_LEGACY(channels->ch3);
    } else {
        _channels[0] = CHANNEL_SCALE(channels->ch0);
        _channels[1] = CHANNEL_SCALE(channels->ch1);
        _channels[2] = CHANNEL_SCALE(channels->ch2);
        _channels[3] = CHANNEL_SCALE(channels->ch3);
    }

#define CHANNEL_LR_RESCALE(x) (5 * x - 108)
#define CHANNEL_LR_SCALE(x) (int32_t(x) * 2 + 988)
#define CHANNEL_LR_SCALE_LEGACY(x) (CHANNEL_LR_RESCALE(x) + 988)

    switch (_frame.type) {
        case GHST_UL_RC_CHANS_HS4_5TO8:
        case GHST_UL_RC_CHANS_HS4_9TO12:
        case GHST_UL_RC_CHANS_HS4_13TO16: {
            uint8_t offset = (_frame.type - GHST_UL_RC_CHANS_HS4_5TO8 + 1) * 4;
            _channels[offset++] = CHANNEL_LR_SCALE_LEGACY(lowres_channels[0]);
            _channels[offset++] = CHANNEL_LR_SCALE_LEGACY(lowres_channels[1]);
            _channels[offset++] = CHANNEL_LR_SCALE_LEGACY(lowres_channels[2]);
            _channels[offset++] = CHANNEL_LR_SCALE_LEGACY(lowres_channels[3]);
            rc_frame = true;
            break;
        }
        case GHST_UL_RC_CHANS_HS4_12_5TO8:
        case GHST_UL_RC_CHANS_HS4_12_9TO12:
        case GHST_UL_RC_CHANS_HS4_12_13TO16: {
            uint8_t offset = (_frame.type - GHST_UL_RC_CHANS_HS4_12_5TO8 + 1) * 4;
            _channels[offset++] = CHANNEL_LR_SCALE(lowres_channels[0]);
            _channels[offset++] = CHANNEL_LR_SCALE(lowres_channels[1]);
            _channels[offset++] = CHANNEL_LR_SCALE(lowres_channels[2]);
            _channels[offset++] = CHANNEL_LR_SCALE(lowres_channels[3]);
            rc_frame = true;
            break;
        }
        case GHST_UL_RC_CHANS_RSSI:
        case GHST_UL_RC_CHANS_12_RSSI:
            process_link_stats_frame((uint8_t*)&_frame.payload);
            break;

        default:
            break;
    }

#if AP_GHST_TELEM_ENABLED
    if (AP_GHST_Telem::process_frame(FrameType(_frame.type), (uint8_t*)&_frame.payload)) {
        process_telemetry();
    }
#endif

    return rc_frame;
}

// send out telemetry
bool AP_RCProtocol_GHST::process_telemetry(bool check_constraint)
{

    AP_HAL::UARTDriver *uart = get_current_UART();
    if (!uart) {
        return false;
    }

    if (!telem_available) {
#if AP_GHST_TELEM_ENABLED
        if (AP_GHST_Telem::get_telem_data(&_telemetry_frame, is_tx_active())) {
            telem_available = true;
        } else {
            return false;
        }
#else
        return false;
#endif
    }
    if (!write_frame(&_telemetry_frame)) {
        return false;
    }

    // get fresh telem_data in the next call
    telem_available = false;

    return true;
}

// process link statistics to get RSSI
void AP_RCProtocol_GHST::process_link_stats_frame(const void* data)
{
    const LinkStatisticsFrame* link = (const LinkStatisticsFrame*)data;

    uint8_t rssi_dbm;
    rssi_dbm = link->rssi_dbm;
    _link_status.link_quality = link->link_quality;
    if (_use_lq_for_rssi) {
        _link_status.rssi = derive_scaled_lq_value(link->link_quality);
    } else{
        // AP rssi: -1 for unknown, 0 for no link, 255 for maximum link
        if (rssi_dbm < 50) {
            _link_status.rssi = 255;
        } else if (rssi_dbm > 120) {
            _link_status.rssi = 0;
        } else {
            // this is an approximation recommended by Remo from TBS
            _link_status.rssi = int16_t(roundf((1.0f - (rssi_dbm - 50.0f) / 70.0f) * 255.0f));
        }
    }

    _link_status.rf_mode = link->protocol;
}

bool AP_RCProtocol_GHST::is_telemetry_supported() const
{
    return _link_status.rf_mode == AP_RCProtocol_GHST::GHST_RF_MODE_NORMAL
           || _link_status.rf_mode == AP_RCProtocol_GHST::GHST_RF_MODE_RACE
           || _link_status.rf_mode == AP_RCProtocol_GHST::GHST_RF_MODE_LR
           || _link_status.rf_mode == AP_RCProtocol_GHST::GHST_RF_MODE_RACE250;
}

// process a byte provided by a uart
void AP_RCProtocol_GHST::process_byte(uint8_t byte, uint32_t baudrate)
{
    // reject RC data if we have been configured for standalone mode
    if (baudrate != CRSF_BAUDRATE && baudrate != GHST_BAUDRATE) {
        return;
    }
    _process_byte(AP_HAL::micros(), byte);
}

// change the bootstrap baud rate to Ghost standard if configured
void AP_RCProtocol_GHST::process_handshake(uint32_t baudrate)
{
    AP_HAL::UARTDriver *uart = get_current_UART();

    // only change the baudrate if we are specifically bootstrapping Ghost
    if (uart == nullptr
        || baudrate != CRSF_BAUDRATE
        || baudrate == GHST_BAUDRATE
        || uart->get_baud_rate() == GHST_BAUDRATE
        || !protocol_enabled(AP_RCProtocol::GHST)
        || protocol_enabled(AP_RCProtocol::CRSF)) {
        return;
    }

    uart->begin(GHST_BAUDRATE);
}

//returns uplink link quality on 0-255 scale
int16_t AP_RCProtocol_GHST::derive_scaled_lq_value(uint8_t uplink_lq)
{
    return int16_t(roundf(constrain_float(uplink_lq*2.5f,0,255)));
}

namespace AP {
    AP_RCProtocol_GHST* ghost() {
        return AP_RCProtocol_GHST::get_singleton();
    }
};

#endif  // AP_RCPROTOCOL_GHST_ENABLED
