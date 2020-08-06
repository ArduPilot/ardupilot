/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Crossfire constants provided by Team Black Sheep under terms of the 2-Clause BSD License
 */
#pragma once

#include "AP_RCProtocol.h"
#include <AP_Math/AP_Math.h>
#include "SoftSerial.h"

#define CRSF_MAX_CHANNELS   16U      // Maximum number of channels from crsf datastream
#define CRSF_FRAMELEN_MAX   64U      // maximum possible framelength
#define CRSF_BAUDRATE       416666

class AP_RCProtocol_CRSF : public AP_RCProtocol_Backend {
public:
    AP_RCProtocol_CRSF(AP_RCProtocol &_frontend);
    virtual ~AP_RCProtocol_CRSF();
    void process_byte(uint8_t byte, uint32_t baudrate) override;
    void process_pulse(uint32_t width_s0, uint32_t width_s1) override;
    void update(void) override;
    // get singleton instance
    static AP_RCProtocol_CRSF* get_singleton() {
        return _singleton;
    }

    enum FrameType {
        CRSF_FRAMETYPE_GPS = 0x02,
        CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
        CRSF_FRAMETYPE_HEARTBEAT = 0x0B,
        CRSF_FRAMETYPE_VTX = 0x0F,
        CRSF_FRAMETYPE_VTX_TELEM = 0x10,
        CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
        CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
        CRSF_FRAMETYPE_ATTITUDE = 0x1E,
        CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
        // Extended Header Frames, range: 0x28 to 0x96
        CRSF_FRAMETYPE_PARAM_DEVICE_PING = 0x28,
        CRSF_FRAMETYPE_PARAM_DEVICE_INFO = 0x29,
        CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
        CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
        CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,
        CRSF_FRAMETYPE_COMMAND = 0x32,
    };

    // Command IDs for CRSF_FRAMETYPE_COMMAND
    enum CommandID {
        CRSF_COMMAND_FC = 0x01,
        CRSF_COMMAND_BLUETOOTH = 0x03,
        CRSF_COMMAND_OSD = 0x05,
        CRSF_COMMAND_VTX = 0x08,
        CRSF_COMMAND_LED = 0x09,
        CRSF_COMMAND_FW_UPDATE = 0x0A,
        CRSF_COMMAND_RX = 0x10,
    };

    // Commands for CRSF_COMMAND_FC
    enum CommandFC {
        CRSF_COMMAND_FC_DISARM = 0x01,
        CRSF_COMMAND_SCALE_CHANNEL = 0x02,
    };

    // Commands for CRSF_COMMAND_BLUETOOTH
    enum CommandBluetooth {
        CRSF_COMMAND_BLUETOOTH_RESET = 0x01,
        CRSF_COMMAND_BLUETOOTH_ENABLE = 0x02,
        CRSF_COMMAND_BLUETOOTH_ECHO = 0x64,
    };

    // Commands for CRSF_COMMAND_OSD
    enum CommandOSD {
        CRSF_COMMAND_OSD_SEND = 0x01,
    };

    // Commands for CRSF_COMMAND_VTX
    enum CommandVTX {
        CRSF_COMMAND_VTX_CHANNEL = 0x01,
        CRSF_COMMAND_VTX_FREQ = 0x02,
        CRSF_COMMAND_VTX_POWER = 0x03,
        CRSF_COMMAND_VTX_PITMODE = 0x04,
        CRSF_COMMAND_VTX_PITMODE_POWERUP = 0x05,
        CRSF_COMMAND_VTX_POWER_DBM = 0x08,
    };

    // Commands for CRSF_COMMAND_LED
    enum CommandLED {
        CRSF_COMMAND_LED_SET_DEFAULT = 0x01,
        CRSF_COMMAND_LED_COLOR = 0x02,
        CRSF_COMMAND_LED_PULSE = 0x03,
        CRSF_COMMAND_LED_BLINK = 0x04,
        CRSF_COMMAND_LED_SHIFT = 0x05,
    };

    // Commands for CRSF_COMMAND_FW_UPDATE
    enum CommandFirmwareUpdate {
        CRSF_COMMAND_FIRMWARE_UPDATE_BOOTLOADER = 0x0A,
        CRSF_COMMAND_FIRMWARE_UPDATE_ERASE = 0x0B,
    };

    // Commands for CRSF_COMMAND_RX
    enum CommandRX {
        CRSF_COMMAND_RX_BIND = 0x01,
    };

    enum DeviceAddress {
        CRSF_ADDRESS_BROADCAST = 0x00,
        CRSF_ADDRESS_USB = 0x10,
        CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,
        CRSF_ADDRESS_RESERVED1 = 0x8A,
        CRSF_ADDRESS_PNP_PRO_CURRENT_SENSOR = 0xC0,
        CRSF_ADDRESS_PNP_PRO_GPS = 0xC2,
        CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
        CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
        CRSF_ADDRESS_RESERVED2 = 0xCA,
        CRSF_ADDRESS_RACE_TAG = 0xCC,
        CRSF_ADDRESS_VTX = 0xCE,
        CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,
        CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,
        CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE
    };

    enum ExtendedFrameOffset {
        CRSF_EXTENDED_FRAME_LENGTH_OFFSET = 1,
        CRSF_EXTENDED_FRAME_TYPE_OFFSET = 2,
        CRSF_EXTENDED_FRAME_DESTINATION_OFFSET = 3,
        CRSF_EXTENDED_FRAME_ORIGIN_OFFSET = 4,
        CRSF_EXTENDED_FRAME_PAYLOAD_OFFSET = 5,
    };

    struct Frame {
        uint8_t device_address;
        uint8_t length;
        uint8_t type;
        uint8_t payload[CRSF_FRAMELEN_MAX - 3]; // +1 for crc
    } PACKED;

    struct LinkStatisticsFrame {
        uint8_t uplink_rssi_ant1; // ( dBm * -1 )
        uint8_t uplink_rssi_ant2; // ( dBm * -1 )
        uint8_t uplink_status; // Package success rate / Link quality ( % )
        int8_t uplink_snr; // ( db )
        uint8_t active_antenna; // Diversity active antenna ( enum ant. 1 = 0, ant. 2 )
        uint8_t rf_mode; // ( enum 4fps = 0 , 50fps, 150hz)
        uint8_t uplink_tx_power; // ( enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW )
        uint8_t downlink_rssi; // ( dBm * -1 )
        uint8_t downlink_status; // Downlink package success rate / Link quality ( % )
        int8_t downlink_dnr; // ( db )
    } PACKED;

private:
    struct Frame _frame;
    struct Frame _telemetry_frame;
    uint8_t _frame_ofs;

    const uint8_t MAX_CHANNELS = MIN((uint8_t)CRSF_MAX_CHANNELS, (uint8_t)MAX_RCIN_CHANNELS);

    static AP_RCProtocol_CRSF* _singleton;

    void _process_byte(uint32_t timestamp_us, uint8_t byte);
    bool decode_csrf_packet();
    bool process_telemetry(bool check_constraint = true);
    void process_link_stats_frame(const void* data);
    void write_frame(Frame* frame);
    void start_uart();
    AP_HAL::UARTDriver* get_current_UART() { return (_uart ? _uart : get_available_UART()); }

    uint16_t _channels[CRSF_MAX_CHANNELS];    /* buffer for extracted RC channel data as pulsewidth in microseconds */

    void add_to_buffer(uint8_t index, uint8_t b) { ((uint8_t*)&_frame)[index] = b; }

    uint32_t _last_frame_time_us;
    uint32_t _last_uart_start_time_ms;
    uint32_t _last_rx_time_us;
    uint32_t _start_frame_time_us;
    bool telem_available;
    bool _fast_telem; // is 150Hz telemetry active
    int16_t _current_rssi = -1;

    AP_HAL::UARTDriver *_uart;

    SoftSerial ss{CRSF_BAUDRATE, SoftSerial::SERIAL_CONFIG_8N1};
};

namespace AP {
    AP_RCProtocol_CRSF* crsf();
};
