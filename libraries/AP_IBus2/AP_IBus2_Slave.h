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
    param set SERIAL5_BAUD 1500000
    param set SIM_IBUS2M_ENA 1
    reboot
*/

#pragma once

#include <AP_IBus2/AP_IBus2_config.h>

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
    void update();

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
    bool _initialized;

    static AP_IBus2_Slave *_singleton;

    // Shared RC state — written by the timer task, read by the main task.
    // All fields protected by rc_state.sem.
    struct RCState {
        HAL_Semaphore sem;
        uint16_t channels[32];
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

    // Receive buffer large enough for the largest frame
    uint8_t _rx_buf[IBUS2_FRAME1_MAX];
    uint8_t _rx_len;
    uint8_t _frame1_expected_len;  // decoded from Frame 1 length byte

    // Bitmask of IBUS2Cmd values for which a GCS log has been emitted (bit N = cmd N)
    uint8_t _logged_cmds;

    // Half-duplex echo: bytes sent that will be echoed back to RX
    uint16_t _tx_pending_echo;

    // Lag diagnostics: reset each second
    uint16_t _diag_max_avail;       // peak _port->available() seen this second
    uint16_t _diag_echo_stalls;     // times process_rx returned early due to echo pending
    uint32_t _diag_frame1_count;    // subtype=0 frames decoded this second

    // Pending Frame 2 command
    bool _response_pending;
    IBUS2_Pkt<IBUS2_Frame2> _pending_cmd;
    uint32_t _frame2_end_us;       // micros() when Frame 2 reception completed

    // Response delay before sending Frame 3 (µs)
    static const uint32_t RESPONSE_DELAY_US = 160;

    void process_rx();
    void handle_frame1(const uint8_t *buf, uint8_t len);
    struct {
        bool have_decompression_key;
        uint8_t channel_types[32];  // 5-bit ChannelType per channel from subtype=1
        uint8_t channel_count;      // active channels (stops at first NbBits<2)
    } frame1_handling;

    void handle_frame2(const IBUS2_Pkt<IBUS2_Frame2> *f2);
    void send_frame3();
    void send_resp_get_type();
    void send_resp_get_value();
    void send_resp_get_param(const IBUS2_Cmd_GetParam *cmd);
    void send_resp_set_param(const IBUS2_Cmd_SetParam *cmd);

    void port_write(const uint8_t *buf, size_t size) {
        _tx_pending_echo += _port->write(buf, size);
    }

    // Populate sensor data into a GET_VALUE response value[14] buffer.
    // Returns number of data points written.
    uint8_t populate_sensor_data(uint8_t *value14);
};

#endif  // AP_IBUS2_SLAVE_ENABLED
