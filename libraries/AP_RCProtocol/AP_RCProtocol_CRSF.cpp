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
  CRSF protocol decoder based on betaflight implementation
  Code by Andy Piper
 */

#include "AP_RCProtocol.h"
#include "AP_RCProtocol_SRXL.h"
#include "AP_RCProtocol_CRSF.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_RCTelemetry/AP_CRSF_Telem.h>
#include <AP_SerialManager/AP_SerialManager.h>

/*
 * CRSF protocol
 *
 * CRSF protocol uses a single wire half duplex uart connection.
 * The master sends one frame every 4ms and the slave replies between two frames from the master.
 *
 * 420000 baud
 * not inverted
 * 8 Bit
 * 1 Stop bit
 * Big endian
 * 416666 bit/s = 46667 byte/s (including stop bit) = 21.43us per byte
 * Max frame size is 64 bytes
 * A 64 byte frame plus 1 sync byte can be transmitted in 1393 microseconds.
 *
 * CRSF_TIME_NEEDED_PER_FRAME_US is set conservatively at 1500 microseconds
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

// #define CRSF_DEBUG
#ifdef CRSF_DEBUG
# define debug(fmt, args...)	hal.console->printf("CRSF: " fmt "\n", ##args)
static const char* get_frame_type(uint8_t byte)
{
    switch(byte) {
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_GPS:
        return "GPS";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_BATTERY_SENSOR:
        return "BATTERY";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_HEARTBEAT:
        return "HEARTBEAT";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_VTX:
        return "VTX";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_VTX_TELEM:
        return "VTX_TELEM";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAM_DEVICE_PING:
        return "PING";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_COMMAND:
        return "COMMAND";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_ATTITUDE:
        return "ATTITUDE";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_FLIGHT_MODE:
        return "FLIGHT_MODE";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAM_DEVICE_INFO:
        return "DEVICE_INFO";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAMETER_READ:
        return "PARAM_READ";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY:
        return "SETTINGS_ENTRY";
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_LINK_STATISTICS:
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
    case AP_RCProtocol_CRSF::CRSF_FRAMETYPE_PARAMETER_WRITE:
        return "UNKNOWN";
    }
    return "UNKNOWN";
}
#else
# define debug(fmt, args...)	do {} while(0)
#endif

#define CRSF_FRAME_TIMEOUT_US      10000U // 10ms to account for scheduling delays
#define CRSF_INTER_FRAME_TIME_US_250HZ    4000U // At fastest, frames are sent by the transmitter every 4 ms, 250 Hz
#define CRSF_INTER_FRAME_TIME_US_150HZ    6667U // At medium, frames are sent by the transmitter every 6.667 ms, 150 Hz
#define CRSF_INTER_FRAME_TIME_US_50HZ    20000U // At slowest, frames are sent by the transmitter every 20ms, 50 Hz
#define CSRF_HEADER_LEN     2

#define CRSF_DIGITAL_CHANNEL_MIN 172
#define CRSF_DIGITAL_CHANNEL_MAX 1811


AP_RCProtocol_CRSF* AP_RCProtocol_CRSF::_singleton;

AP_RCProtocol_CRSF::AP_RCProtocol_CRSF(AP_RCProtocol &_frontend) : AP_RCProtocol_Backend(_frontend)
{
#if !APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    if (_singleton != nullptr) {
        AP_HAL::panic("Duplicate CRSF handler");
    }

    _singleton = this;
#else
    if (_singleton == nullptr) {
        _singleton = this;
    }
#endif
#if HAL_CRSF_TELEM_ENABLED && !APM_BUILD_TYPE(APM_BUILD_iofirmware) && !APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_CRSF, 0);
    if (_uart) {
        start_uart();
    }
#endif
}

AP_RCProtocol_CRSF::~AP_RCProtocol_CRSF() {
    _singleton = nullptr;
}

void AP_RCProtocol_CRSF::process_pulse(uint32_t width_s0, uint32_t width_s1)
{
    uint8_t b;
    if (ss.process_pulse(width_s0, width_s1, b)) {
        _process_byte(ss.get_byte_timestamp_us(), b);
    }
}

void AP_RCProtocol_CRSF::_process_byte(uint32_t timestamp_us, uint8_t byte)
{
    //debug("process_byte(0x%x)", byte);
    // we took too long decoding, start again - the RX will only send complete frames so this is unlikely to fail,
    // however thread scheduling can introduce longer delays even when the data has been received
    if (_frame_ofs > 0 && (timestamp_us - _start_frame_time_us) > CRSF_FRAME_TIMEOUT_US) {
        _frame_ofs = 0;
    }

    _last_rx_time_us = timestamp_us;

    // overflow check
    if (_frame_ofs >= CRSF_FRAMELEN_MAX) {
        _frame_ofs = 0;
    }

    // start of a new frame
    if (_frame_ofs == 0) {
        _start_frame_time_us = timestamp_us;
    }

    add_to_buffer(_frame_ofs++, byte);

    // need a header to get the length
    if (_frame_ofs < CSRF_HEADER_LEN) {
        return;
    }

    // parse the length
    if (_frame_ofs == CSRF_HEADER_LEN) {
        // check for garbage frame
        if (_frame.length > CRSF_FRAMELEN_MAX) {
            _frame_ofs = 0;
        }
        return;
    }

    // overflow check
    if (_frame_ofs > _frame.length + CSRF_HEADER_LEN) {
        _frame_ofs = 0;
        return;
    }

    // decode whatever we got and expect
    if (_frame_ofs == _frame.length + CSRF_HEADER_LEN) {
        log_data(AP_RCProtocol::CRSF, timestamp_us, (const uint8_t*)&_frame, _frame_ofs - CSRF_HEADER_LEN);

        // we consumed the partial frame, reset
        _frame_ofs = 0;

        uint8_t crc = crc8_dvb_s2(0, _frame.type);
        for (uint8_t i = 0; i < _frame.length - 2; i++) {
            crc = crc8_dvb_s2(crc, _frame.payload[i]);
        }

        // bad CRC
        if (crc != _frame.payload[_frame.length - CSRF_HEADER_LEN]) {
            return;
        }

        _last_frame_time_us = timestamp_us;
        // decode here
        if (decode_crsf_packet()) {
            add_input(MAX_CHANNELS, _channels, false, _link_status.rssi);
        }
    }
}

void AP_RCProtocol_CRSF::update(void)
{
    // if we are in standalone mode, process data from the uart
    if (_uart) {
        uint32_t now = AP_HAL::millis();
        // for some reason it's necessary to keep trying to start the uart until we get data
        if (now - _last_uart_start_time_ms > 1000U && _last_frame_time_us == 0) {
            start_uart();
            _last_uart_start_time_ms = now;
        }
        uint32_t n = _uart->available();
        n = MIN(n, 255U);
        for (uint8_t i = 0; i < n; i++) {
            int16_t b = _uart->read();
            if (b >= 0) {
                _process_byte(now, uint8_t(b));
            }
        }
    }

    // never received RC frames, but have received CRSF frames so make sure we give the telemetry opportunity to run
    uint32_t now = AP_HAL::micros();
    if (_last_frame_time_us > 0 && !get_rc_frame_count() && now - _last_frame_time_us > CRSF_INTER_FRAME_TIME_US_250HZ) {
        process_telemetry(false);
        _last_frame_time_us = now;
    }
}

// write out a frame of any type
void AP_RCProtocol_CRSF::write_frame(Frame* frame)
{
    AP_HAL::UARTDriver *uart = get_current_UART();

    if (!uart) {
        return;
    }
    // calculate crc
    uint8_t crc = crc8_dvb_s2(0, frame->type);
    for (uint8_t i = 0; i < frame->length - 2; i++) {
        crc = crc8_dvb_s2(crc, frame->payload[i]);
    }
    frame->payload[frame->length - 2] = crc;

    uart->write((uint8_t*)frame, frame->length + 2);

#ifdef CRSF_DEBUG
    hal.console->printf("CRSF: writing %s:", get_frame_type(frame->type));
    for (uint8_t i = 0; i < frame->length + 2; i++) {
        uint8_t val = ((uint8_t*)frame)[i];
        if (val >= 32 && val <= 126) {
            hal.console->printf(" 0x%x '%c'", val, (char)val);
        } else {
            hal.console->printf(" 0x%x", val);
        }
    }
    hal.console->printf("\n");
#endif
}

bool AP_RCProtocol_CRSF::decode_crsf_packet()
{
#ifdef CRSF_DEBUG
    hal.console->printf("CRSF: received %s:", get_frame_type(_frame.type));
    uint8_t* fptr = (uint8_t*)&_frame;
    for (uint8_t i = 0; i < _frame.length + 2; i++) {
        hal.console->printf(" 0x%x", fptr[i]);
    }
    hal.console->printf("\n");
#endif

    bool rc_active = false;

    switch (_frame.type) {
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            // scale factors defined by TBS - TICKS_TO_US(x) ((x - 992) * 5 / 8 + 1500)
            decode_11bit_channels((const uint8_t*)(&_frame.payload), CRSF_MAX_CHANNELS, _channels, 5U, 8U, 880U);
            rc_active = !_uart; // only accept RC data if we are not in standalone mode
            break;
        case CRSF_FRAMETYPE_LINK_STATISTICS:
            process_link_stats_frame((uint8_t*)&_frame.payload);
            break;
        default:
            break;
    }
#if HAL_CRSF_TELEM_ENABLED && !APM_BUILD_TYPE(APM_BUILD_iofirmware)
    if (AP_CRSF_Telem::process_frame(FrameType(_frame.type), (uint8_t*)&_frame.payload)) {
        process_telemetry();
    }
#endif

    return rc_active;
}

// send out telemetry
bool AP_RCProtocol_CRSF::process_telemetry(bool check_constraint)
{

    AP_HAL::UARTDriver *uart = get_current_UART();
    if (!uart) {
        return false;
    }

    if (!telem_available) {
#if HAL_CRSF_TELEM_ENABLED && !APM_BUILD_TYPE(APM_BUILD_iofirmware)
        if (AP_CRSF_Telem::get_telem_data(&_telemetry_frame)) {
            telem_available = true;
        } else {
            return false;
        }
#else
        return false;
#endif
    }
    write_frame(&_telemetry_frame);
    // get fresh telem_data in the next call
    telem_available = false;

    return true;
}

// process link statistics to get RSSI
void AP_RCProtocol_CRSF::process_link_stats_frame(const void* data)
{
    const LinkStatisticsFrame* link = (const LinkStatisticsFrame*)data;

    uint8_t rssi_dbm;
    if (link->active_antenna == 0) {
        rssi_dbm = link->uplink_rssi_ant1;
    } else {
        rssi_dbm = link->uplink_rssi_ant2;
    }
     // AP rssi: -1 for unknown, 0 for no link, 255 for maximum link
    if (rssi_dbm < 50) {
        _link_status.rssi = 255;
    } else if (rssi_dbm > 120) {
        _link_status.rssi = 0;
    } else {
        // this is an approximation recommended by Remo from TBS
        _link_status.rssi = int16_t(roundf((1.0f - (rssi_dbm - 50.0f) / 70.0f) * 255.0f));
    }

    _link_status.rf_mode = static_cast<RFMode>(MIN(link->rf_mode, 3U));
}

// process a byte provided by a uart
void AP_RCProtocol_CRSF::process_byte(uint8_t byte, uint32_t baudrate)
{
    // reject RC data if we have been configured for standalone mode
    if (baudrate != CRSF_BAUDRATE || _uart) {
        return;
    }
    _process_byte(AP_HAL::micros(), byte);
}

// start the uart if we have one
void AP_RCProtocol_CRSF::start_uart()
{
    _uart->configure_parity(0);
    _uart->set_stop_bits(1);
    _uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    _uart->set_blocking_writes(false);
    _uart->set_options(_uart->get_options() & ~AP_HAL::UARTDriver::OPTION_RXINV);
    _uart->begin(CRSF_BAUDRATE, 128, 128);
}

namespace AP {
    AP_RCProtocol_CRSF* crsf() {
        return AP_RCProtocol_CRSF::get_singleton();
    }
};
