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
  AP_IBus2_Slave: ArduPilot as an IBUS2 slave device (telemetry adapter role).

  In this role a FlySky receiver is the master. ArduPilot:
    - receives Frame 1 → extracts RC channel values
    - receives Frame 2 → parses command
    - sends    Frame 3 → vehicle telemetry (voltage, GPS, attitude, etc.)

  The device identity matches the FlySky telemetry adapter:
    VID = 0x01, PID = 0x03  (Telemetry_Adapter_Protocol___20240923_en_.pdf §4)

  Serial port: SerialProtocol_IBUS2_Slave (52), 1.5 Mbit/s, OPTION_HDPLEX.

  Usage:
    sim_vehicle.py -v ArduCopter -A --serial5=sim:ibus2master
    param set SERIAL5_PROTOCOL 52
    param set SIM_IBUS2M_ENA 1
    reboot
*/

#pragma once

#include <AP_IBus2/AP_IBus2_config.h>

// Debug: log every valid Frame 1 as an RCDA dataflash message (same format as
// AP_RCProtocol_Backend::log_data).  Set to 0 to exclude from production builds.
#define AP_IBUS2_LOG_RAW_FRAMES 1

#if AP_IBUS2_SLAVE_ENABLED

#include <AP_IBus2/AP_IBus2.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_AHRS/AP_AHRS_config.h>
#include <AP_BattMonitor/AP_BattMonitor_config.h>
#include <AP_GPS/AP_GPS_config.h>

class AP_IBus2_Slave
{
public:
    AP_IBus2_Slave();

    CLASS_NO_COPY(AP_IBus2_Slave);

    void init();

    static const struct AP_Param::GroupInfo var_info[];

    // Singleton accessor (set in constructor)
    static AP_IBus2_Slave *get_singleton() { return _singleton; }

    // Number of valid RC channels received from the master
    uint8_t get_rc_channel_count() const;

    // Read received channel values (PWM µs). Returns true if any channels are valid.
    bool get_rc_channels(uint16_t *channels, uint8_t &count) const;

    // Timestamp (ms) of the last time fresh RC data was stored; 0 if none received yet.
    // A stale or zero value indicates the link may be down.
    uint32_t get_rc_last_update_ms() const;

    // True if the master reported failsafe or sync-lost in the most recent Frame 1.
    // sync_lost=1: receiver has lost radio link to transmitter.
    // failsafe=1:  transmitter-side failsafe has triggered (e.g. TX powered off).
    // Either condition means the pilot has no live control; use get_rc_last_update_ms()
    // to additionally detect complete loss of the receiver (no Frame 1 at all).
    bool get_failsafe() const;

private:
    AP_HAL::UARTDriver *_port;

    // Device type reported in the GET_TYPE enumeration response.  0xF8
    // (digital servo) is the only value the spec documents; the type a
    // genuine telemetry adapter reports is unknown, so this is settable
    // to allow experimentation with sensor discovery.
    AP_Int16 _device_type;

    void thread_main();

    static AP_IBus2_Slave *_singleton;

    // Shared RC state — written by the IBUS2 thread, read by the main task.
    // All fields protected by rc_state.sem.
    struct RCState {
        HAL_Semaphore sem;
        uint16_t channels[AP_IBUS2_MAX_CHANNELS];
        uint8_t  channel_count;
        // Last time fresh channel data was stored (ms); 0 if never received.
        // Goes stale if Frame 1 stops arriving (receiver powered off / disconnected).
        uint32_t last_update_ms;
        // sync_lost || failsafe from the most recent Frame 1 header.
        bool failsafe;
    } mutable _rc_state;

    // Frame reception state machine
    enum class RxState : uint8_t {
        WAIT_HEADER,
        IN_FRAME1,
        IN_FRAME2,
    };
    RxState _rx_state;

    // Silence threshold for mid-frame resynchronisation (µs): longer than
    // any intra-frame byte gap, shorter than the master's inter-frame gap.
    static const uint32_t RX_GAP_RESET_US = 500;
    // Time bytes were last available in process_rx()
    uint32_t _last_rx_us;

    // Echo of our own transmission can only arrive within this long of the
    // write (µs); older echo debt is forgiven rather than discarding the
    // start of the master's next frame.  Must be shorter than the gap
    // between our Frame 3 and the master's next Frame 1.
    static const uint32_t ECHO_TIMEOUT_US = 700;
    // Time of our last Frame 3 write (µs)
    uint32_t _last_tx_us;

    // Receive buffer large enough for the largest frame
    uint8_t _rx_buf[IBUS2_FRAME1_MAX];
    uint8_t _rx_len;
    uint8_t _frame1_expected_len;  // decoded from Frame 1 length byte

    // Bitmask of IBUS2Cmd values for which a GCS log has been emitted (bit N = cmd N)
    uint8_t _logged_cmds;

    // Half-duplex echo: bytes sent that will be echoed back to RX
    uint16_t _tx_pending_echo;

    // Pending Frame 2 command
    bool _response_pending;
    IBUS2_Pkt<IBUS2_Frame2> _pending_cmd;
    uint32_t _frame2_end_us;       // micros() when Frame 2 reception completed

    // Delay from the end of Frame 2 on the wire to writing Frame 3 (µs):
    // the spec's window is 160±25 µs (FlySky "IBUS2 communication
    // protocol" V2.4, appendix "Data frame timing").  The wire end is
    // estimated via receive_time_constraint_us(), so this is the full spec
    // figure rather than a platform-tuned compensation.
    static const uint32_t RESPONSE_DELAY_US = 160;

    // Transmit-start handoff consumed between the write and the first bit
    // on the wire, subtracted from the delay.  55µs is the measured floor
    // on an H7 — the fastest MCU we run on — so on any real board the
    // response cannot land before the window opens at 135µs; slower boards
    // only push it later, which receivers measurably tolerate.
    static const uint32_t TX_START_FLOOR_US = 55;

    // Which telemetry packet of the multi-packet GET_VALUE cycle to send next
    uint8_t _telem_pkt_index;

    void process_rx();
    void handle_frame1(const uint8_t *buf, uint8_t len);
#if AP_IBUS2_LOG_RAW_FRAMES
    void log_raw_frame(const uint8_t *buf, uint8_t len) const;
#endif
    struct {
        bool have_decompression_key;
        uint8_t channel_types[AP_IBUS2_MAX_CHANNELS];  // 5-bit ChannelType per channel from subtype=1
        uint8_t channel_count;      // active channels (stops at first NbBits<2)
    } frame1_handling;

    // Resources we still require from the master, reported in the GET_TYPE
    // response.  Each bit is cleared as the resource is delivered; the
    // master only begins sensor polling (GET_VALUE) once we report zero.
    static const uint8_t REQUIRED_CHANNEL_TYPES = (1U << 0);
    static const uint8_t REQUIRED_FAILSAFE      = (1U << 1);
    uint8_t _required_resources = REQUIRED_CHANNEL_TYPES | REQUIRED_FAILSAFE;

    void handle_frame2(const IBUS2_Pkt<IBUS2_Frame2> *f2);
    void send_frame3();
    void send_resp_get_type();
    void send_resp_get_value();
    void send_resp_get_param(const IBUS2_Cmd_GetParam *cmd);
    void send_resp_set_param(const IBUS2_Cmd_SetParam *cmd);

    void port_write(const uint8_t *buf, size_t size) {
        _tx_pending_echo += _port->write(buf, size);
        _last_tx_us = AP_HAL::micros();
    }

    // Populate sensor data into a GET_VALUE response value[14] buffer.
    // Returns number of data points written.
    uint8_t populate_sensor_data(uint8_t *value14);
};

#endif  // AP_IBUS2_SLAVE_ENABLED
